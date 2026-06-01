import json
import logging
import os
import sys
import time

from pimotorcontrol import MCP3008PositionSensor, pimc
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
#     */5 * * * * /usr/bin/python3 /home/pi/greenhouse.py >> /var/log/greenhouse.log 2>&1
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
#   To recover from a fault: inspect the journal file (default:
#   greenhouse_status.json), address the root cause, then delete or reset
#   the file. The controller will resume normal operation on the next
#   cron invocation.
#
# TEMPERATURE FUNCTION:
#
#   The controller does not contain any temperature-reading logic. Instead,
#   a callable is passed at instantiation. This callable takes no arguments
#   and returns a float temperature in Fahrenheit. This decouples the
#   controller from any specific sensor library or hardware, and makes it
#   straightforward to test with a mock function.
#
#   Example using an existing DS18B20 read function:
#     from my_sensor_module import read_temperature_f
#     controller = GreenhouseVentController(motor=mc, get_temperature=read_temperature_f)
#
# DEADBAND:
#
#   The controller uses a simple deadband to avoid hunting (rapidly
#   opening and closing in response to small temperature fluctuations):
#
#     temperature >= temp_open  => open one increment (unless fully open)
#     temperature <= temp_close => close one increment (unless fully closed)
#     temp_close < temperature < temp_open => do nothing
#
#   The gap between temp_close and temp_open is the deadband. Default is
#   70F (close) to 80F (open), giving a 10F deadband.
# -----------------------------------------------------------------------------


class GreenhouseVentController:

    def __init__(self, motor, get_temperature,
                 temp_open=80.0,
                 temp_close=70.0,
                 journal_file="greenhouse_status.json",
                 logger=None):
        """Initialize the GreenhouseVentController.

        Args:
            motor(pimc): An initialized pimc instance with a position_sensor
                         configured. The motor's voltage limits and increment
                         are set on the pimc instance, not here.
            get_temperature(callable): A callable that takes no arguments and
                                       returns the current temperature as a
                                       float in Fahrenheit. This is intentionally
                                       injected rather than hard-coded, so that
                                       any sensor library or mock can be used
                                       without modifying this class.
            temp_open(float): Temperature in Fahrenheit at or above which the
                              vents should be opened one increment. Default 80.0.
            temp_close(float): Temperature in Fahrenheit at or below which the
                               vents should be closed one increment. Default 70.0.
            journal_file(str): Path to this controller's JSON journal file.
                               Used to persist fault state across cron invocations.
                               Distinct from pimc's own journal file. Default is
                               'greenhouse_status.json' in the current directory.
            logger(obj): Logger to use. Uses root logger if None.
        """
        if temp_close >= temp_open:
            raise ValueError(
                f"temp_close ({temp_close}F) must be less than temp_open ({temp_open}F) "
                f"to create a valid deadband. With equal or reversed values, the controller "
                f"would attempt to open and close simultaneously."
            )

        if not callable(get_temperature):
            raise ValueError("get_temperature must be a callable that returns a float temperature in Fahrenheit")

        if not isinstance(motor, pimc):
            raise ValueError("motor must be a pimc instance")

        if motor.position_sensor is None:
            raise ValueError(
                "The provided pimc instance has no position_sensor configured. "
                "GreenhouseVentController requires voltage-based position feedback. "
                "Pass an MCP3008PositionSensor (or compatible object) as position_sensor "
                "when constructing the pimc instance."
            )

        self.motor = motor
        self.get_temperature = get_temperature
        self.temp_open = temp_open
        self.temp_close = temp_close
        self.journal_file = journal_file
        self.logger = logger or logging.getLogger(__name__)

    # -------------------------------------------------------------------------
    # Journal file format
    # -------------------------------------------------------------------------
    # The journal is a small JSON file with the following structure:
    #
    # Normal (no fault):
    #   {
    #     "status": "ok",
    #     "last_run": "2024-06-01T14:35:00",
    #     "last_temp_f": 82.4,
    #     "last_action": "opened_increment"
    #   }
    #
    # Fault state:
    #   {
    #     "status": "fault",
    #     "fault_time": "2024-06-01T14:35:00",
    #     "fault_reason": "Motor fault during open_increment"
    #   }
    #
    # The "status" key is the only one checked on startup. All other fields
    # are informational and intended for human diagnosis.
    #
    # If the journal file does not exist, that is treated as a clean first run
    # (not a fault). This makes initial deployment straightforward — no need
    # to pre-create the file.
    # -------------------------------------------------------------------------

    def _read_journal(self):
        """Read and return the journal file contents as a dict.

        If the file does not exist, returns None (treated as first run, not fault).
        If the file exists but cannot be parsed, logs an error and returns a
        synthetic fault dict to prevent motor operation on a corrupt journal.

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
            # A corrupt or unreadable journal is treated as a fault condition.
            # We cannot safely know the prior state, so we refuse to act.
            self.logger.error(
                "Failed to read journal file %s: %s. Treating as fault to prevent "
                "unsafe operation with unknown prior state.", self.journal_file, e
            )
            return {"status": "fault", "fault_reason": f"Unreadable journal: {e}"}

    def _write_journal(self, data):
        """Write a dict to the journal file as JSON, with OS sync for reliability.

        Mirrors the write-and-sync pattern used in pimc for the same reasons:
        on a Raspberry Pi that may lose power unexpectedly, an unsynced write
        can leave a corrupt or empty file, which would then trigger the corrupt-
        journal fault path on the next run.

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
            # Log but do not raise — a failed journal write should not itself
            # cause a fault that prevents the controller from finishing its
            # current cycle. The next run may behave unexpectedly without a
            # journal, but that is preferable to crashing mid-operation.
            self.logger.error("Failed to write journal file %s: %s", self.journal_file, e)

    def _write_fault(self, reason):
        """Write a fault state to the journal file and log the condition.

        After this is called, all subsequent cron invocations will read the
        fault state and exit without touching the motor, until the fault is
        manually cleared.

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
            temperature(float or None): The temperature reading from this run,
                                        in Fahrenheit. The same value that drove
                                        the control decision in check_and_adjust(),
                                        passed through rather than re-read, so the
                                        journal accurately reflects what caused the
                                        action rather than a subsequent reading.
            action(str): Description of the action taken this run (e.g. 'opened_increment',
                         'closed_increment', 'no_action_deadband').

        Returns:
            None
        """
        self._write_journal({
            "status": "ok",
            "last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "last_temp_f": temperature,
            "last_action": action,
        })

    def check_and_adjust(self):
        """Perform one temperature sample-and-respond cycle.

        This is the core logic method. It:
          1. Reads the current temperature via get_temperature()
          2. Evaluates against the deadband thresholds
          3. Calls open_increment() or close_increment() on the motor if needed
          4. Returns a tuple of (action, temperature)

        Returning the temperature alongside the action avoids the need for a
        second sensor read in run() when writing the journal. The temperature
        in the journal should reflect the value that drove the control decision,
        not a re-read taken moments later — particularly since re-reading adds
        no useful information at the cron timescale and introduces an unnecessary
        second point of failure.

        This method is separated from run() so that it can be called and tested
        independently without the journal read/write and exit logic that run()
        adds for cron deployment.

        Args:
            None

        Returns:
            (action, temperature)(tuple):
                action(str): One of 'opened_increment', 'closed_increment',
                             'no_action_deadband', 'no_action_fully_open',
                             'no_action_fully_closed', or 'fault'.
                temperature(float or None): The temperature reading taken this
                             cycle, in Fahrenheit. None if the read failed.
        """
        # Read temperature from the injected callable.
        try:
            temperature = self.get_temperature()
        except Exception as e:
            self.logger.error("Failed to read temperature: %s", e)
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
                self.logger.info("Vents are already fully open, no action taken")
                return 'no_action_fully_open', temperature

            result = self.motor.open_increment()
            if not result:
                self.logger.error("open_increment() returned failure")
                return 'fault', temperature

            self.logger.info("Opened one increment successfully")
            return 'opened_increment', temperature

        elif temperature <= self.temp_close:
            self.logger.info("Temperature %.1fF <= close threshold %.1fF, attempting to close one increment",
                             temperature, self.temp_close)

            if self.motor.is_fully_closed():
                self.logger.info("Vents are already fully closed, no action taken")
                return 'no_action_fully_closed', temperature

            result = self.motor.close_increment()
            if not result:
                self.logger.error("close_increment() returned failure")
                return 'fault', temperature

            self.logger.info("Closed one increment successfully")
            return 'closed_increment', temperature

        else:
            # Temperature is within the deadband. This is the expected steady-state
            # outcome during a well-regulated day — most runs should land here.
            self.logger.info("Temperature %.1fF is within deadband (%.1fF - %.1fF), no action taken",
                             temperature, self.temp_close, self.temp_open)
            return 'no_action_deadband', temperature

    def run(self):
        """Run one complete cron-invocation cycle.

        This is the intended entry point when running from cron. It:
          1. Checks the journal for a prior fault state and exits immediately if found
          2. Calls check_and_adjust() to evaluate temperature and act
          3. Writes the outcome to the journal (fault or ok)
          4. Exits with status code 0 on success, 1 on fault

        Faults cause an immediate exit with sys.exit(1), which cron can be
        configured to alert on (e.g. MAILTO in crontab).

        Args:
            None

        Returns:
            None (exits the process)
        """
        # --- Check for prior fault state ---
        #
        # If a prior run wrote a fault to the journal, we refuse to operate
        # until the fault is manually cleared. This is a deliberate safety
        # choice: we do not know what state the physical system is in after
        # a fault, and taking further automated action could make things worse.
        #
        # To clear a fault: investigate the logged reason, address the root
        # cause, verify the physical vent position is safe, then delete the
        # journal file or set "status" to "ok" manually.
        journal = self._read_journal()
        if journal is not None and journal.get("status") == "fault":
            self.logger.error(
                "Prior fault detected in journal %s. Reason: %s. "
                "Refusing to operate until fault is manually cleared.",
                self.journal_file,
                journal.get("fault_reason", "unknown")
            )
            sys.exit(1)

        # --- Run the control cycle ---
        action, temperature = self.check_and_adjust()

        if action == 'fault':
            # check_and_adjust already logged the specific error.
            # Write fault to journal so subsequent runs also abort.
            self._write_fault("check_and_adjust returned fault — see prior log entries for details")
            sys.exit(1)

        # The temperature passed here is the same value that drove the control
        # decision — not a re-read. See _write_ok() and check_and_adjust() for
        # the reasoning.
        self._write_ok(temperature, action)
        self.logger.info("Run complete. Action: %s", action)
        sys.exit(0)


# -----------------------------------------------------------------------------
# Example usage / entry point
# -----------------------------------------------------------------------------
#
# This block shows how to wire up the controller for the greenhouse use case.
# Adjust pin numbers, voltage thresholds, and temperature thresholds to match
# your physical installation.
#
# To use from cron, either call this file directly (python3 greenhouse.py)
# or import GreenhouseVentController and construct it from your own script.
# -----------------------------------------------------------------------------

if __name__ == "__main__":

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s"
    )

    # --- Temperature sensor ---
    # Replace this placeholder with your actual DS18B20 (or other) read function.
    # The function must take no arguments and return a float in Fahrenheit.
    # Example: from my_sensor import read_temp_f
    def get_temperature():
        d = ds18b20.DS18B20()
        return d.get_temperature(unit=2)
        raise NotImplementedError(
            "Replace this placeholder with your actual temperature read function. "
            "It should take no arguments and return a float in Fahrenheit."
        )

    # --- ADC position sensor ---
    # MCP3008 channel 0, 5V reference. Adjust channel and vref to match your wiring.
    sensor = MCP3008PositionSensor(channel=0, vref=3.3)

    # --- Motor controller ---
    # Adjust voltage thresholds and increment to match your potentiometer's
    # actual voltage range once you have measured it on your physical system.
    #
    # voltage_fully_closed: voltage when vents are physically closed
    # voltage_fully_open:   voltage when vents are physically fully open
    # voltage_increment:    how many volts to move per temperature check cycle
    # voltage_tolerance:    acceptable margin for "close enough" to a target
    #
    # All values are in volts. Measure with a multimeter or by reading the ADC
    # directly:
    #   python3 -c "from pimotorcontrol import MCP3008PositionSensor; s = MCP3008PositionSensor(); print(s.read())"
    motor = pimc(
        journal_filename="pimc_status",
        position_sensor=sensor,
        voltage_fully_closed=1.9,   # replace with measured value
        voltage_fully_open=2.3,     # replace with measured value
        voltage_increment=0.1,      # replace with desired step size
        voltage_tolerance=0.05,
        direction_fault_threshold=0.3,
        direction_settle_seconds=0.02,
        maxtime=2,
        ch1_pin=21,
        ch2_pin=20,
        relay_mode="enable_direction"
    )

    # --- Vent controller ---
    controller = GreenhouseVentController(
        motor=motor,
        get_temperature=get_temperature,
        temp_open=80.0,
        temp_close=70.0,
        journal_file="greenhouse_status.json",
    )

    controller.run()
