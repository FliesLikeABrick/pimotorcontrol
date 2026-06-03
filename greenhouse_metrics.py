import argparse
import logging
import os
import sys

from pimotorcontrol import MCP3008PositionSensor, voltage_to_pct
from ds18b20 import ds18b20
from telemetry import TelemetryClient
from greenhouse import load_config, cfg

# -----------------------------------------------------------------------------
# greenhouse_metrics.py - Periodic greenhouse metric telemetry emitter
# -----------------------------------------------------------------------------
#
# Reads temperature and optionally vent position voltage from the greenhouse Pi
# and emits them as a metric document to Elasticsearch.
#
# Intentionally separate from greenhouse.py because:
#   - Metric sampling and vent control run on independent intervals.
#   - A telemetry failure here must never affect the vent controller cron job.
#
# INTENDED USAGE (cronjob):
#
#   */1 * * * * python3 /home/pi/greenhouse_metrics.py --config greenhouse.yaml >> /var/log/greenhouse_metrics.log 2>&1
#   */5 * * * * python3 /home/pi/greenhouse.py --config greenhouse.yaml >> /var/log/greenhouse.log 2>&1
#
# CONFIGURATION:
#
#   Shares greenhouse.yaml with greenhouse.py. CLI args override config values.
#   --es-api-key is required either via config or CLI.
#   --vref enables ADC position reading; if absent only temperature is emitted.
#
# DOCUMENT FORMAT (pi-metrics index):
#
#   {
#     "@timestamp":      "...",
#     "host":            "pi-greenhouse",
#     "source":          "greenhouse",
#     "temperature_f":   82.4,
#     "position_voltage": 2.14,   # only when --vref / adc.vref is configured
#     "position_pct":    65.0     # only when voltage limits are also configured
#   }
# -----------------------------------------------------------------------------


def read_temperature():
    """Read current temperature from the DS18B20 sensor in Fahrenheit.

    Args:
        None

    Returns:
        temperature_f(float): Current temperature in Fahrenheit.
    """
    d = ds18b20.DS18B20()
    return d.get_temperature(unit=2)


def main():
    parser = argparse.ArgumentParser(
        description="Greenhouse metric telemetry emitter.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  # using config file (recommended)
  python3 greenhouse_metrics.py --config greenhouse.yaml

  # temperature only, no config file
  python3 greenhouse_metrics.py --es-host 172.28.11.170 --es-api-key <key>

  # with position voltage and percentage
  python3 greenhouse_metrics.py --config greenhouse.yaml --vref 3.3

  # verbose logging
  python3 greenhouse_metrics.py --config greenhouse.yaml --debug
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

    # --- ADC args ---
    # --vref enables position voltage reading. If absent, only temperature is emitted.
    # Voltage limits are read from config to compute position_pct.
    parser.add_argument("--vref", action="store", type=float, default=None,
                        help="ADC reference voltage in volts. Overrides config adc.vref. "
                             "If provided, vent position voltage and percentage are included.")
    parser.add_argument("--adc-channel", action="store", type=int, default=None,
                        help="MCP3008 analog input channel. Overrides config adc.channel.")

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

    # --- Resolve values ---
    def resolve(cli_val, *config_keys, default=None):
        if cli_val is not None:
            return cli_val
        config_val = cfg(config, *config_keys)
        return config_val if config_val is not None else default

    es_host    = resolve(args.es_host,    'elasticsearch', 'host',    default='localhost')
    es_port    = resolve(args.es_port,    'elasticsearch', 'port',    default=9200)
    es_api_key = resolve(args.es_api_key, 'elasticsearch', 'api_key', default=None)

    vref        = resolve(args.vref,        'adc', 'vref',    default=None)
    adc_channel = resolve(args.adc_channel, 'adc', 'channel', default=0)

    # Voltage limits — needed for position_pct calculation.
    voltage_fully_closed = cfg(config, 'motor', 'voltage_fully_closed', default=None)
    voltage_fully_open   = cfg(config, 'motor', 'voltage_fully_open',   default=None)
    voltage_tolerance    = cfg(config, 'motor', 'voltage_tolerance',    default=0.0)

    # --- Validate required values ---
    if not es_api_key:
        logger.error(
            "Elasticsearch API key is required. Provide via --es-api-key or "
            "elasticsearch.api_key in the config file."
        )
        sys.exit(1)

    # --- Telemetry client ---
    telemetry = TelemetryClient(
        api_key=es_api_key,
        host=es_host,
        port=es_port,
        source="greenhouse",
        logger=logger,
    )

    # --- Build metric document ---
    # Read all sensors first, then emit once. This keeps @timestamp as close
    # as possible to the actual moment of reading.
    document = {}

    # Temperature — always attempted.
    try:
        temperature_f = read_temperature()
        document["temperature_f"] = temperature_f
        logger.info("Temperature: %.1fF", temperature_f)
    except Exception as e:
        logger.error("Failed to read temperature: %s", e)

    # Position voltage and percentage — only if vref is configured.
    if vref is not None:
        try:
            sensor = MCP3008PositionSensor(channel=adc_channel, vref=vref)
            position_voltage = sensor.read()
            sensor.close()
            document["position_voltage"] = position_voltage

            # Compute percentage from the cached reading rather than re-reading
            # the ADC, to avoid a second read returning a different value due to
            # pot wiper noise. Requires voltage limits from config.
            if voltage_fully_closed is not None and voltage_fully_open is not None:
                position_pct = voltage_to_pct(
                    position_voltage,
                    voltage_fully_closed,
                    voltage_fully_open,
                    voltage_tolerance,
                )
                document["position_pct"] = round(position_pct, 1)
                logger.info("Position voltage: %.3fV (%.1f%% open)", position_voltage, position_pct)
            else:
                logger.info(
                    "Position voltage: %.3fV (percentage unavailable — "
                    "motor.voltage_fully_closed and motor.voltage_fully_open "
                    "must be set in the config file)",
                    position_voltage
                )
        except Exception as e:
            logger.error("Failed to read position voltage: %s", e)
            # Not fatal — emit temperature without position if ADC read fails.

    # --- Emit ---
    if not document:
        logger.error("No metrics collected, nothing to emit.")
        sys.exit(1)

    result = telemetry.emit_metric(document)

    if not result:
        logger.error("Failed to emit metric document to Elasticsearch.")
        sys.exit(1)

    logger.info("Metric emitted successfully.")
    sys.exit(0)


if __name__ == "__main__":
    main()
