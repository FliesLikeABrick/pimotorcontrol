import argparse
import json
import logging
import os
import sys
import time

from pimotorcontrol import MCP3008PositionSensor, pimc, voltage_to_pct
from ds18b20 import ds18b20

# -----------------------------------------------------------------------------
# greenhouse.py - Greenhouse vent temperature controller
# -----------------------------------------------------------------------------
#
# This module provides GreenhouseVentController, which uses a pimc motor
# controller instance to incrementally open or close greenhouse vents based
# on temperature readings, with a configurable deadband.
#
# INTENDED USAGE (cronjob):
#
#   This controller is designed to be invoked by cron on a regular interval
#   (e.g. every 5 minutes). Each invocation performs exactly one
#   sample-and-respond cycle, then exits. There is no daemon loop here.
#
#   Example crontab entry (runs every 5 minutes):
#     */5 * * * * /usr/bin/python3 /home/pi/greenhouse.py --config greenhouse.yaml >> /var/log/greenhouse.log 2>&1
#
#   For verbose logging during testing:
#     python3 greenhouse.py --config greenhouse.yaml --debug
#
# CONFIGURATION:
#
#   All deployment-specific values (voltage thresholds, pin numbers, temperature
#   thresholds, Elasticsearch credentials) live in greenhouse.yaml. CLI arguments
#   override config file values when both are provided. Values absent from the
#   config file fall back to argparse defaults.
#
# FAULT SAFETY AND JOURNAL FILE:
#
#   If an unexpected condition occurs (motor fault, ADC read failure, etc.),
#   the controller writes a fault state to its own journal file and exits
#   with a non-zero status code. Subsequent cron invocations will read this
#   fault state, log an error, and exit immediately without touching the motor.
#
#   This is intentional. When something unexpected happens to hardware with
#   real-world consequences, the safest behavior is to stop acting and wait
#   for a human to investigate and clear the fault manually.
#
#   To recover from a fault: inspect the journal file, address the root cause,
#   then delete the journal file or set "status" to "ok" manually.
#
# TEMPERATURE FUNCTION:
#
#   The controller does not contain any temperature-reading logic. Instead,
#   a callable is passed at instantiation. This callable takes no arguments
#   and returns a float temperature in Fahrenheit.
#
# DEADBAND:
#
#   The controller uses a simple deadband to avoid hunting:
#
#     temperature >= temp_open  => open one increment (unless fully open)
#     temperature <= temp_close => close one increment (unless fully closed)
#     temp_close < temperature < temp_open => do nothing
#
# TELEMETRY:
#
#   If a TelemetryClient instance is provided at instantiation, the controller
#   will emit events to Elasticsearch after each control cycle. Telemetry
#   failures are non-fatal — a failure to emit never faults the controller.
#   Provide Elasticsearch credentials in greenhouse.yaml or via CLI args.
# -----------------------------------------------------------------------------


def load_config(config_path):
    """Load and return a YAML configuration file as a dict.

    If the file does not exist or cannot be parsed, logs an error and returns
    an empty dict so that all values fall back to argparse defaults.

    Args:
        config_path(str): Path to the YAML config file.

    Returns:
        config(dict): Parsed config contents, or empty dict on failure.
    """
    try:
        import yaml
    except ImportError:
        logging.error("PyYAML is not installed. Install with: pip install pyyaml --break-system-packages")
        return {}

    if not os.path.exists(config_path):
        logging.error("Config file not found: %s", config_path)
        return {}

    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logging.error("Failed to parse config file %s: %s", config_path, e)
        return {}


def cfg(config, *keys, default=None):
    """Safely retrieve a nested value from a config dict.

    Args:
        config(dict): The config dict returned by load_config().
        *keys: Sequence of keys to traverse, e.g. cfg(config, 'motor', 'vref').
        default: Value to return if any key is missing. Defaults to None.

    Returns:
        value: The config value, or default if not found.
    """
    val = config
    for key in keys:
        if not isinstance(val, dict) or key not in val:
            return default
        val = val[key]
    return val


class GreenhouseVentController:

    # -------------------------------------------------------------------------
    # Telemetry field value constants
    # -------------------------------------------------------------------------
    # All action, result, and event_type values are prefixed with
    # 'greenhouse_vent_' to namespace them within the shared pi-events index.
    # level values and description are intentionally un-prefixed.
    # -------------------------------------------------------------------------
    EVENT_TYPE = "greenhouse_vent_action"

    ACTION_OPENED       = "greenhouse_vent_opened_increment"
    ACTION_CLOSED       = "greenhouse_vent_closed_increment"
    ACTION_NO_ACTION    = "greenhouse_vent_no_action"

    RESULT_SUCCESS      = "greenhouse_vent_success"
    RESULT_FAULT        = "greenhouse_vent_fault"
    RESULT_AT_LIMIT     = "greenhouse_vent_at_limit"
    RESULT_DEADBAND     = "greenhouse_vent_deadband"

    def __init__(self, motor, get_temperature,
                 temp_open=85.0,
                 temp_close=75.0,
                 journal_file="greenhouse_status.json",
                 telemetry=None,
                 logger=None):
        """Initialize the GreenhouseVentController.

        Args:
            motor(pimc): An initialized pimc instance with a position_sensor configured.
            get_temperature(callable): A callable that takes no arguments and returns
                                       the current temperature as a float in Fahrenheit.
            temp_open(float): Temperature at or above which vents open one increment. Default 85.0F.
            temp_close(float): Temperature at or below which vents close one increment. Default 75.0F.
            journal_file(str): Path to this controller's JSON journal file. Default 'greenhouse_status.json'.
            telemetry(TelemetryClient or None): Optional TelemetryClient for Elasticsearch. Default None.
            logger(obj): Logger to use. Uses root logger if None.
        """
        if temp_close >= temp_open:
            raise ValueError(
                f"temp_close ({temp_close}F) must be less than temp_open ({temp_open}F) "
                f"to create a valid deadband."
            )

        if not callable(get_temperature):
            raise ValueError("get_temperature must be a callable that returns a float temperature in Fahrenheit")

        if not isinstance(motor, pimc):
            raise ValueError("motor must be a pimc instance")

        if motor.position_sensor is None:
            raise ValueError(
                "The provided pimc instance has no position_sensor configured. "
                "GreenhouseVentController requires voltage-based position feedback."
            )

        self.motor = motor
        self.get_temperature = get_temperature
        self.temp_open = temp_open
        self.temp_close = temp_close
        self.journal_file = journal_file
        self.telemetry = telemetry
        self.logger = logger or logging.getLogger(__name__)

    # -------------------------------------------------------------------------
    # Journal file format
    # -------------------------------------------------------------------------
    # Normal (no fault):
    #   { "status": "ok", "last_run": "...", "last_temp_f": 82.4, "last_action": "opened_increment" }
    #
    # Fault state:
    #   { "status": "fault", "fault_time": "...", "fault_reason": "..." }
    #
    # If the journal file does not exist, that is treated as a clean first run.
    # -------------------------------------------------------------------------

    def _read_journal(self):
        """Read and return the journal file contents as a dict.

        Args:
            None

        Returns:
            journal(dict or None): Parsed journal contents, or None if file absent.
        """
        if not os.path.exists(self.journal_file):
            self.logger.debug("No journal file found at %s, treating as first run", self.journal_file)
            return None

        try:
            with open(self.journal_file, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            self.logger.error(
                "Failed to read journal file %s: %s. Treating as fault to prevent "
                "unsafe operation with unknown prior state.", self.journal_file, e
            )
            return {"status": "fault", "fault_reason": f"Unreadable journal: {e}"}

    def _write_journal(self, data):
        """Write a dict to the journal file as JSON, with OS sync for reliability.

        Args:
            data(dict): Data to write to the journal file.

        Returns:
            None
        """
        try:
            with open(self.journal_file, 'w') as f:
                json.dump(data, f, indent=2)
            os.sync()
        except OSError as e:
            self.logger.error("Failed to write journal file %s: %s", self.journal_file, e)

    def _write_fault(self, reason):
        """Write a fault state to the journal file and log the condition.

        Args:
            reason(str): Human-readable description of the fault condition.

        Returns:
            None
        """
        fault_time = time.strftime("%Y-%m-%dT%H:%M:%S")
        self.logger.error("FAULT: %s. Writing fault state to %s. Manual intervention required.", reason, self.journal_file)
        self._write_journal({
            "status": "fault",
            "fault_time": fault_time,
            "fault_reason": reason,
        })

    def _write_ok(self, temperature, action):
        """Write a successful-run state to the journal file.

        Args:
            temperature(float or None): Temperature reading from this run, in Fahrenheit.
            action(str): Description of the action taken this run.

        Returns:
            None
        """
        self._write_journal({
            "status": "ok",
            "last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "last_temp_f": temperature,
            "last_action": action,
        })

    def _emit_event(self, action, result, temperature, level="info",
                    description=None, timestamp=None):
        """Emit a vent action event to Elasticsearch via the telemetry client.

        No-op if no telemetry client is configured. Failures are logged but
        never raised — a failure to emit must never fault the controller.

        Args:
            action(str): What was attempted. Use ACTION_* class constants.
            result(str): Outcome of the action. Use RESULT_* class constants.
            temperature(float or None): Temperature reading that drove the decision.
            level(str): Event severity: 'info', 'warn', or 'error'. Default 'info'.
            description(str or None): Human-readable context. Default None.
            timestamp(str or datetime or None): Event timestamp. Default None (generated at emit time).

        Returns:
            None
        """
        if self.telemetry is None:
            return

        # Read current position voltage once and reuse for both voltage and
        # percentage fields, avoiding two ADC reads with potentially differing
        # values due to pot wiper noise.
        position_voltage = None
        position_pct = None
        try:
            position_voltage = self.motor.read_position()
            position_pct = voltage_to_pct(
                position_voltage,
                self.motor.voltage_fully_closed,
                self.motor.voltage_fully_open,
                self.motor.voltage_tolerance,
            )
        except Exception as e:
            self.logger.debug("_emit_event: could not read position for telemetry: %s", e)

        document = {
            "event_type":   self.EVENT_TYPE,
            "action":       action,
            "result":       result,
            "level":        level,
        }

        if description is not None:
            document["description"] = description

        if temperature is not None:
            document["temperature_f"] = temperature

        if position_voltage is not None:
            document["position_voltage"] = position_voltage

        if position_pct is not None:
            document["position_pct"] = round(position_pct, 1)

        self.telemetry.emit_event(document, timestamp=timestamp)

    def check_and_adjust(self):
        """Perform one temperature sample-and-respond cycle.

        Reads temperature, evaluates against deadband thresholds, calls
        open_increment() or close_increment() if needed, emits telemetry,
        and returns (action, temperature).

        Args:
            None

        Returns:
            (action, temperature)(tuple):
                action(str): One of 'opened_increment', 'closed_increment',
                             'no_action_deadband', 'no_action_fully_open',
                             'no_action_fully_closed', or 'fault'.
                temperature(float or None): Temperature reading in Fahrenheit,
                             or None if the read failed.
        """
        try:
            temperature = self.get_temperature()
        except Exception as e:
            self.logger.error("Failed to read temperature: %s", e)
            self._emit_event(
                action=self.ACTION_NO_ACTION,
                result=self.RESULT_FAULT,
                temperature=None,
                level="error",
                description=f"Temperature read failed: {e}",
            )
            return 'fault', None

        self.logger.info("Temperature reading: %.1fF (open threshold: %.1fF, close threshold: %.1fF)",
                         temperature, self.temp_open, self.temp_close)

        # --- Deadband evaluation ---
        #
        # Three zones:
        #   temperature >= temp_open  : too hot, open an increment if possible
        #   temperature <= temp_close : too cold, close an increment if possible
        #   between the two           : within deadband, do nothing
        #
        # The deadband prevents hunting — without it, a temperature sitting
        # near a single threshold would cause the motor to open and close on
        # every cron run.

        if temperature >= self.temp_open:
            self.logger.info("Temperature %.1fF >= open threshold %.1fF, attempting to open one increment",
                             temperature, self.temp_open)

            if self.motor.is_fully_open():
                pos_v = self.motor.read_position()
                pos_pct = voltage_to_pct(pos_v, self.motor.voltage_fully_closed, self.motor.voltage_fully_open, self.motor.voltage_tolerance)
                self.logger.info(
                    "Vents are already fully open (%.3fV, %.1f%% open), no action taken",
                    pos_v, pos_pct
                )
                self._emit_event(
                    action=self.ACTION_OPENED,
                    result=self.RESULT_AT_LIMIT,
                    temperature=temperature,
                    level="warn",
                    description="Temperature above open threshold but vents already at fully open position.",
                )
                return 'no_action_fully_open', temperature

            # Disable limit check since it was explicitly checked above. A duplicate
            # check could only pick up on noise and confuse matters.
            result = self.motor.open_increment(check_limits=False)

            if result is False:
                self.logger.error("open_increment() returned failure — see pimc logs for motor-level detail")
                self._emit_event(
                    action=self.ACTION_OPENED,
                    result=self.RESULT_FAULT,
                    temperature=temperature,
                    level="error",
                    description="open_increment() returned failure — see pimc logs for motor-level detail.",
                )
                return 'fault', temperature

            elif result is None:
                pos_v = self.motor.read_position()
                pos_pct = voltage_to_pct(pos_v, self.motor.voltage_fully_closed, self.motor.voltage_fully_open, self.motor.voltage_tolerance)
                self.logger.info(
                    "open_increment() reported already at limit (%.3fV, %.1f%% open), no movement occurred",
                    pos_v, pos_pct
                )
                self._emit_event(
                    action=self.ACTION_OPENED,
                    result=self.RESULT_AT_LIMIT,
                    temperature=temperature,
                    level="warn",
                    description="open_increment() reported already at or within tolerance of fully open position.",
                )
                return 'no_action_fully_open', temperature

            else:
                # pimc's open_increment already logged the final voltage and
                # percentage via its own post-move log. No duplicate read needed here.
                self.logger.info("Opened one increment successfully")
                self._emit_event(
                    action=self.ACTION_OPENED,
                    result=self.RESULT_SUCCESS,
                    temperature=temperature,
                    level="info",
                )
                return 'opened_increment', temperature

        elif temperature <= self.temp_close:
            self.logger.info("Temperature %.1fF <= close threshold %.1fF, attempting to close one increment",
                             temperature, self.temp_close)

            if self.motor.is_fully_closed():
                pos_v = self.motor.read_position()
                pos_pct = voltage_to_pct(pos_v, self.motor.voltage_fully_closed, self.motor.voltage_fully_open, self.motor.voltage_tolerance)
                self.logger.info(
                    "Vents are already fully closed (%.3fV, %.1f%% open), no action taken",
                    pos_v, pos_pct
                )
                self._emit_event(
                    action=self.ACTION_CLOSED,
                    result=self.RESULT_AT_LIMIT,
                    temperature=temperature,
                    level="warn",
                    description="Temperature below close threshold but vents already at fully closed position.",
                )
                return 'no_action_fully_closed', temperature

            # Disable limit check since it was explicitly checked above. A duplicate
            # check could only pick up on noise and confuse matters.
            result = self.motor.close_increment(check_limits=False)

            if result is False:
                self.logger.error("close_increment() returned failure — see pimc logs for motor-level detail")
                self._emit_event(
                    action=self.ACTION_CLOSED,
                    result=self.RESULT_FAULT,
                    temperature=temperature,
                    level="error",
                    description="close_increment() returned failure — see pimc logs for motor-level detail.",
                )
                return 'fault', temperature

            elif result is None:
                pos_v = self.motor.read_position()
                pos_pct = voltage_to_pct(pos_v, self.motor.voltage_fully_closed, self.motor.voltage_fully_open, self.motor.voltage_tolerance)
                self.logger.info(
                    "close_increment() reported already at limit (%.3fV, %.1f%% open), no movement occurred",
                    pos_v, pos_pct
                )
                self._emit_event(
                    action=self.ACTION_CLOSED,
                    result=self.RESULT_AT_LIMIT,
                    temperature=temperature,
                    level="warn",
                    description="close_increment() reported already at or within tolerance of fully closed position.",
                )
                return 'no_action_fully_closed', temperature

            else:
                # pimc's close_increment already logged the final voltage and
                # percentage via its own post-move log. No duplicate read needed here.
                self.logger.info("Closed one increment successfully")
                self._emit_event(
                    action=self.ACTION_CLOSED,
                    result=self.RESULT_SUCCESS,
                    temperature=temperature,
                    level="info",
                )
                return 'closed_increment', temperature

        else:
            pos_v = self.motor.read_position()
            pos_pct = voltage_to_pct(pos_v, self.motor.voltage_fully_closed, self.motor.voltage_fully_open, self.motor.voltage_tolerance)
            self.logger.info(
                "Temperature %.1fF is within deadband (%.1fF - %.1fF), no action taken. "
                "Current position: %.3fV (%.1f%% open)",
                temperature, self.temp_close, self.temp_open, pos_v, pos_pct
            )
            self._emit_event(
                action=self.ACTION_NO_ACTION,
                result=self.RESULT_DEADBAND,
                temperature=temperature,
                level="info",
                description=f"Temperature {temperature:.1f}F is within deadband "
                            f"({self.temp_close:.1f}F - {self.temp_open:.1f}F).",
            )
            return 'no_action_deadband', temperature

    def run(self):
        """Run one complete cron-invocation cycle.

        Checks for prior fault, calls check_and_adjust(), writes journal,
        exits with 0 on success or 1 on fault.

        Args:
            None

        Returns:
            None (exits the process)
        """
        journal = self._read_journal()
        if journal is not None and journal.get("status") == "fault":
            self.logger.error(
                "Prior fault detected in journal %s. Reason: %s. "
                "Refusing to operate until fault is manually cleared.",
                self.journal_file,
                journal.get("fault_reason", "unknown")
            )
            sys.exit(1)

        action, temperature = self.check_and_adjust()

        if action == 'fault':
            self._write_fault("check_and_adjust returned fault — see prior log entries for details")
            sys.exit(1)

        self._write_ok(temperature, action)
        self.logger.info("Run complete. Action: %s", action)
        sys.exit(0)


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Greenhouse vent temperature controller.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  python3 greenhouse.py --config greenhouse.yaml
  python3 greenhouse.py --config greenhouse.yaml --debug
  python3 greenhouse.py --config greenhouse.yaml --temp-open 80 --temp-close 70
        """
    )

    parser.add_argument("--config", action="store", default=None,
                        help="Path to YAML config file. CLI args override config values.")
    parser.add_argument("--debug", action="store_true",
                        help="Enable verbose debug logging.")

    # --- Telemetry args ---
    parser.add_argument("--es-host", action="store", default=None,
                        help="Elasticsearch host. Overrides config file.")
    parser.add_argument("--es-port", action="store", type=int, default=None,
                        help="Elasticsearch port. Overrides config file.")
    parser.add_argument("--es-api-key", action="store", default=None,
                        help="Elasticsearch API key. Overrides config file.")

    # --- Controller args ---
    parser.add_argument("--temp-open", action="store", type=float, default=None,
                        help="Temperature threshold to open vents (F). Overrides config.")
    parser.add_argument("--temp-close", action="store", type=float, default=None,
                        help="Temperature threshold to close vents (F). Overrides config.")
    parser.add_argument("--journal-file", action="store", default=None,
                        help="Path to greenhouse controller journal file. Overrides config.")
    parser.add_argument("--pimc-journal", action="store", default=None,
                        help="Path to pimc motor controller journal file. Overrides config.")

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s"
    )
    logger = logging.getLogger(__name__)

    # --- Load config file ---
    config = {}
    if args.config:
        config = load_config(args.config)

    # --- Resolve values: CLI args take precedence over config, config over defaults ---
    def resolve(cli_val, *config_keys, default=None):
        """Return CLI value if provided, else config value, else default."""
        if cli_val is not None:
            return cli_val
        config_val = cfg(config, *config_keys)
        return config_val if config_val is not None else default

    es_host    = resolve(args.es_host,    'elasticsearch', 'host',    default='localhost')
    es_port    = resolve(args.es_port,    'elasticsearch', 'port',    default=9200)
    es_api_key = resolve(args.es_api_key, 'elasticsearch', 'api_key', default=None)

    temp_open    = resolve(args.temp_open,    'controller', 'temp_open',    default=85.0)
    temp_close   = resolve(args.temp_close,   'controller', 'temp_close',   default=75.0)
    journal_file = resolve(args.journal_file, 'controller', 'journal_file', default='greenhouse_status.json')
    pimc_journal = resolve(args.pimc_journal, 'controller', 'pimc_journal', default='pimc_status')

    # Motor / ADC values — no CLI overrides for these, config or defaults only.
    voltage_fully_closed     = cfg(config, 'motor', 'voltage_fully_closed',     default=None)
    voltage_fully_open       = cfg(config, 'motor', 'voltage_fully_open',       default=None)
    voltage_increment        = cfg(config, 'motor', 'voltage_increment',        default=None)
    voltage_tolerance        = cfg(config, 'motor', 'voltage_tolerance',        default=0.1)
    direction_fault_threshold= cfg(config, 'motor', 'direction_fault_threshold',default=0.1)
    direction_settle_seconds = cfg(config, 'motor', 'direction_settle_seconds', default=0.1)
    poll_interval_seconds    = cfg(config, 'motor', 'poll_interval_seconds',    default=0.01)
    maxtime                  = cfg(config, 'motor', 'maxtime',                  default=30)
    ch1_pin                  = cfg(config, 'motor', 'ch1_pin',                  default=26)
    ch2_pin                  = cfg(config, 'motor', 'ch2_pin',                  default=20)
    relay_mode               = cfg(config, 'motor', 'relay_mode',               default='independent')

    vref        = cfg(config, 'adc', 'vref',    default=None)
    adc_channel = cfg(config, 'adc', 'channel', default=0)

    # Validate required values that have no safe default.
    missing = [name for name, val in [
        ('motor.voltage_fully_closed', voltage_fully_closed),
        ('motor.voltage_fully_open',   voltage_fully_open),
        ('motor.voltage_increment',    voltage_increment),
        ('motor.relay_mode',           relay_mode),
        ('adc.vref',                   vref),
    ] if val is None]
    if missing:
        logger.error(
            "The following required values are missing from the config file and "
            "have no CLI override: %s", ', '.join(missing)
        )
        sys.exit(1)

    # --- Telemetry client ---
    telemetry = None
    if es_api_key:
        from telemetry import TelemetryClient
        telemetry = TelemetryClient(
            api_key=es_api_key,
            host=es_host,
            port=es_port,
            source="greenhouse",
            logger=logger,
        )
        logger.info("Telemetry enabled. host=%s port=%s", es_host, es_port)
    else:
        logger.debug("No Elasticsearch API key configured, telemetry disabled.")

    # --- Temperature sensor ---
    def get_temperature():
        d = ds18b20.DS18B20()
        return d.get_temperature(unit=2)

    # --- ADC position sensor ---
    sensor = MCP3008PositionSensor(channel=adc_channel, vref=vref)

    # --- Motor controller ---
    motor = pimc(
        journal_filename=pimc_journal,
        position_sensor=sensor,
        voltage_fully_closed=voltage_fully_closed,
        voltage_fully_open=voltage_fully_open,
        voltage_increment=voltage_increment,
        voltage_tolerance=voltage_tolerance,
        direction_fault_threshold=direction_fault_threshold,
        direction_settle_seconds=direction_settle_seconds,
        poll_interval_seconds=poll_interval_seconds,
        maxtime=maxtime,
        ch1_pin=ch1_pin,
        ch2_pin=ch2_pin,
        relay_mode=relay_mode,
        logger=logger,
    )

    # --- Vent controller ---
    controller = GreenhouseVentController(
        motor=motor,
        get_temperature=get_temperature,
        temp_open=temp_open,
        temp_close=temp_close,
        journal_file=journal_file,
        telemetry=telemetry,
        logger=logger,
    )

    controller.run()
