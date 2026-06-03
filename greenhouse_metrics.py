import argparse
import logging
import sys

from pimotorcontrol import MCP3008PositionSensor, voltage_to_pct
from ds18b20 import ds18b20
from telemetry import TelemetryClient

# -----------------------------------------------------------------------------
# greenhouse_metrics.py - Periodic greenhouse metric telemetry emitter
# -----------------------------------------------------------------------------
#
# This script reads temperature and optionally vent position voltage from the
# greenhouse Pi and emits them as a metric document to Elasticsearch.
#
# It is intentionally separate from greenhouse.py (the vent controller) because:
#   - Metric sampling and vent control run on independent intervals. Temperature
#     may be sampled every minute for graph resolution while vent control runs
#     every 5 minutes.
#   - A telemetry failure should never gate vent control. Keeping them separate
#     means a failure here has no effect on the vent controller's cron job.
#
# INTENDED USAGE (cronjob):
#
#   Example crontab entries:
#     */1 * * * * /usr/bin/python3 /home/pi/greenhouse_metrics.py --es-host 172.28.11.170 --es-api-key <key> >> /var/log/greenhouse_metrics.log 2>&1
#     */5 * * * * /usr/bin/python3 /home/pi/greenhouse.py --es-host 172.28.11.170 --es-api-key <key> >> /var/log/greenhouse.log 2>&1
#
# DOCUMENT FORMAT:
#
#   Emits to the pi-metrics index with named fields:
#     {
#       "@timestamp":      "...",    # time of reading, generated on the Pi
#       "host":            "pi-greenhouse",
#       "source":          "greenhouse",
#       "temperature_f":   82.4,     # always present if sensor is readable
#       "position_voltage": 2.14     # present only if --vref is provided
#     }
#
#   temperature_f and position_voltage use the same field names as pi-events
#   documents from greenhouse.py, allowing cross-index Kibana queries on these
#   fields to return both periodic readings and event-driven readings together.
# -----------------------------------------------------------------------------


def read_temperature():
    """Read current temperature from the DS18B20 sensor.

    Args:
        None

    Returns:
        temperature_f(float): Current temperature in Fahrenheit.
    """
    d = ds18b20.DS18B20()
    return d.get_temperature(unit=2)


def main():
    parser = argparse.ArgumentParser(
        description="Greenhouse metric telemetry emitter. "
                    "Reads temperature and optionally vent position and emits to Elasticsearch.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  # temperature only
  python3 greenhouse_metrics.py --es-host 172.28.11.170 --es-api-key <key>

  # temperature and vent position voltage
  python3 greenhouse_metrics.py --es-host 172.28.11.170 --es-api-key <key> --vref 3.3 --fully-closed-voltage 1.75 --fully-open-voltage 2.3

  # verbose logging
  python3 greenhouse_metrics.py --es-host 172.28.11.170 --es-api-key <key> --debug
        """
    )

    parser.add_argument("--debug", action="store_true",
                        help="Enable verbose debug logging.")

    # --- Telemetry args ---
    parser.add_argument("--es-host", action="store", default="localhost",
                        help="Elasticsearch host (default: localhost).")
    parser.add_argument("--es-port", action="store", type=int, default=9200,
                        help="Elasticsearch port (default: 9200).")
    parser.add_argument("--es-api-key", action="store", required=True,
                        help="Elasticsearch API key. REQUIRED.")

    # --- ADC args ---
    # --vref is optional. If provided, vent position voltage is read from the
    # MCP3008 and included in the metric document. If absent, only temperature
    # is emitted. This allows the script to run even if the ADC is not wired
    # or available.
    parser.add_argument("--vref", action="store", type=float, default=None,
                        help="ADC reference voltage in volts. If provided, vent position "
                             "voltage is read from the MCP3008 and included in the metric "
                             "document. Must match the MCP3008 VREF pin voltage exactly. "
                             "If not provided, only temperature is emitted.")
    parser.add_argument("--adc-channel", action="store", type=int, default=0,
                        help="MCP3008 analog input channel (default: 0).")
    parser.add_argument("--fully-closed-voltage", action="store", type=float, default=None,
                        help="Voltage at the fully closed position, in volts. Required to "
                             "calculate position_pct alongside position_voltage. "
                             "Must match the value configured in greenhouse.py.")
    parser.add_argument("--fully-open-voltage", action="store", type=float, default=None,
                        help="Voltage at the fully open position, in volts. Required to "
                             "calculate position_pct alongside position_voltage. "
                             "Must match the value configured in greenhouse.py.")

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s"
    )
    logger = logging.getLogger(__name__)

    # --- Telemetry client ---
    telemetry = TelemetryClient(
        api_key=args.es_api_key,
        host=args.es_host,
        port=args.es_port,
        source="greenhouse",
        logger=logger,
    )

    # --- Build metric document ---
    # Read all sensors first, then emit once. This ensures the document
    # represents a single point in time as closely as possible, and that
    # @timestamp (generated by the telemetry client at emit time) reflects
    # when the readings were taken rather than after any processing.
    document = {}
    success = True

    # Temperature — always attempted.
    try:
        temperature_f = read_temperature()
        document["temperature_f"] = temperature_f
        logger.info("Temperature: %.1fF", temperature_f)
    except Exception as e:
        logger.error("Failed to read temperature: %s", e)
        success = False

    # Position voltage — only if --vref was provided.
    if args.vref is not None:
        try:
            sensor = MCP3008PositionSensor(channel=args.adc_channel, vref=args.vref)
            position_voltage = sensor.read()
            sensor.close()
            document["position_voltage"] = position_voltage
            # Compute percentage from the cached reading rather than re-reading
            # the ADC, to avoid a second read returning a different value due to
            # pot wiper noise. Requires --fully-closed-voltage and
            # --fully-open-voltage to be provided.
            if args.fully_closed_voltage is not None and args.fully_open_voltage is not None:
                position_pct = voltage_to_pct(position_voltage, args.fully_closed_voltage, args.fully_open_voltage)
                document["position_pct"] = round(position_pct, 1)
                logger.info("Position voltage: %.3fV (%.1f%% open)", position_voltage, position_pct)
            else:
                logger.info("Position voltage: %.3fV (percentage unavailable — provide --fully-closed-voltage and --fully-open-voltage)", position_voltage)
        except Exception as e:
            logger.error("Failed to read position voltage: %s", e)
            # Not fatal — emit temperature without position if ADC read fails.

    # --- Emit ---
    if not document:
        # Nothing to emit — all reads failed.
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
