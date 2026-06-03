import logging
import socket
from datetime import datetime, timezone

# -----------------------------------------------------------------------------
# telemetry.py - Elasticsearch telemetry client for Raspberry Pi projects
# -----------------------------------------------------------------------------
#
# This module provides a lightweight Elasticsearch telemetry client for
# emitting metrics and events from Raspberry Pi projects to a central
# Elasticsearch instance.
#
# INTENDED USAGE:
#
#   Instantiate TelemetryClient once at startup, passing the Elasticsearch
#   host and API key. Call emit() to submit documents to any index.
#
#   from telemetry import TelemetryClient
#   telemetry = TelemetryClient(api_key="your-api-key")
#   telemetry.emit("pi-metrics", {"temperature_f": 82.4, "source": "greenhouse"})
#
# FAULT SAFETY:
#
#   Telemetry failures are logged but never raised to the caller. A failure
#   to submit a document must never cause the calling code to fault — the
#   physical control logic (motor movement, vent control) is always more
#   important than the telemetry record of it. This is enforced by the
#   try/except in emit().
#
# TIMESTAMPS:
#
#   The caller should pass a timestamp representing when the event or
#   measurement actually occurred, not when emit() is called. This matters
#   for retry/buffering scenarios and ensures that temperature readings
#   align accurately with vent events on Kibana timelines.
#
#   If no timestamp is passed, emit() generates one at call time, which
#   is acceptable for simple callers where submission latency is negligible.
#
# TLS:
#
#   Elasticsearch 8.x uses TLS by default. This client disables certificate
#   verification (verify=False) because the default Elasticsearch install
#   uses a self-signed certificate. This is acceptable for a private LAN
#   deployment but should be revisited if the deployment is ever exposed
#   beyond a trusted network. See the verify parameter on emit() for details.
#
# INDEX CONVENTIONS:
#
#   Two indices are used across all Pi projects:
#     pi-metrics  — periodic numeric measurements (temperature, water level, etc.)
#     pi-events   — electromechanical events (vent actions, pump start/stop, etc.)
#
#   Both indices use named fields. Common fields across all documents:
#     @timestamp  — ISO 8601 UTC timestamp of when the event/measurement occurred
#     host        — hostname of the Pi that generated the document
#     source      — logical source name (e.g. 'greenhouse', 'sump')
#
#   pi-events additionally uses:
#     level       — severity: 'info', 'warn', or 'error'
#     event_type  — category of event (e.g. 'vent_action', 'pump_event')
#     action      — specific action taken (e.g. 'opened_increment')
#
#   Field names should be consistent across documents of the same logical
#   type to avoid Elasticsearch mapping conflicts. For example, temperature
#   is always 'temperature_f' whether it appears in pi-metrics or pi-events.
# -----------------------------------------------------------------------------


class TelemetryClient:

    # Default index names. Callers may pass any index name to emit(), but
    # these are the conventional names for the two shared indices.
    INDEX_METRICS = "pi-metrics"
    INDEX_EVENTS = "pi-events"

    def __init__(self, api_key, host="localhost", port=9200, source=None, logger=None):
        """Initialize the TelemetryClient.

        Args:
            api_key(str): Elasticsearch API key for authentication. REQUIRED.
                          Create via Kibana > Stack Management > API Keys.
                          Format is 'id:api_key' as shown by Kibana after creation.
                          No default — an absent or incorrect key will cause all
                          emit() calls to fail with an authentication error, which
                          is logged but not raised.
            host(str): Elasticsearch host. Defaults to 'localhost'. For a remote
                       Elasticsearch instance, pass the IP or hostname.
            port(int): Elasticsearch port. Defaults to 9200.
            source(str): Logical source name for this client instance, added to
                         every document as the 'source' field. For example,
                         'greenhouse' or 'sump'. If None, the 'source' field is
                         not added automatically — the caller must include it in
                         the document payload. Defaults to None.
            logger(obj): Logger to use. Uses module logger if None.
        """
        if not api_key:
            raise ValueError(
                "api_key is required. Create one via Kibana > Stack Management > API Keys."
            )

        self.host = host
        self.port = port
        self.source = source
        self.logger = logger or logging.getLogger(__name__)

        # Determine the hostname of this Pi once at init time rather than on
        # every emit() call. Used as the 'host' field in every document.
        self.hostname = socket.gethostname()

        # Build the base URL for the Elasticsearch REST API.
        self.base_url = f"https://{host}:{port}"

        # Build the Authorization header value from the API key.
        # Elasticsearch API key auth uses the 'ApiKey' scheme with the
        # key value as provided by Kibana (id:api_key format, base64-encoded).
        import base64
        self.auth_header = f"ApiKey {api_key}"

        # Import requests here so that the import error is clear if the
        # library is not installed, rather than surfacing at emit() time.
        try:
            import requests
            self.requests = requests
        except ImportError:
            raise ImportError(
                "The 'requests' library is required by TelemetryClient. "
                "Install it with: pip install requests --break-system-packages"
            )

        self.logger.debug(
            "TelemetryClient initialized. host=%s port=%s source=%s hostname=%s",
            host, port, source, self.hostname
        )

    def _build_timestamp(self, timestamp=None):
        """Return a timestamp string suitable for Elasticsearch's @timestamp field.

        Args:
            timestamp(str or datetime or None): If a string, returned as-is (assumed
                to already be ISO 8601 UTC). If a datetime, converted to ISO 8601 UTC
                string. If None, the current UTC time is used.

        Returns:
            timestamp_str(str): ISO 8601 UTC timestamp string.
        """
        if timestamp is None:
            return datetime.now(timezone.utc).isoformat()
        if isinstance(timestamp, datetime):
            # Ensure timezone awareness. If a naive datetime is passed, assume UTC.
            if timestamp.tzinfo is None:
                self.logger.warning(
                    "A naive datetime was passed to emit(). Assuming UTC. "
                    "Pass timezone-aware datetimes to avoid ambiguity."
                )
                timestamp = timestamp.replace(tzinfo=timezone.utc)
            return timestamp.isoformat()
        # Assume string, return as-is.
        return timestamp

    def emit(self, index, document, timestamp=None):
        """Submit a document to Elasticsearch.

        Adds envelope fields (@timestamp, host, source) to the document and
        POSTs it to the specified index via the Elasticsearch REST API.

        Failures are logged but never raised. A telemetry failure must never
        fault the calling code — physical control logic takes priority.

        Args:
            index(str): Elasticsearch index to submit the document to.
                        Use TelemetryClient.INDEX_METRICS or INDEX_EVENTS for
                        the standard shared indices, or any string for a custom index.
            document(dict): Document payload. Should not include @timestamp, host,
                            or source — these are added automatically by emit().
                            If the caller includes them, they will be overwritten
                            by the envelope values.
            timestamp(str or datetime or None): Timestamp representing when the
                            event or measurement occurred. Should be passed by
                            callers that have a meaningful event time (e.g. the
                            moment a temperature reading was taken). If None,
                            the current UTC time at emit() call time is used.
                            See module docstring for reasoning on caller-supplied
                            timestamps.

        Returns:
            success(bool): True if the document was accepted by Elasticsearch,
                           False if any error occurred. The caller may check this
                           but is not required to — telemetry failures are
                           non-fatal by design.
        """
        # Build the envelope. These fields are added to every document
        # regardless of index or document type.
        envelope = {
            "@timestamp": self._build_timestamp(timestamp),
            "host": self.hostname,
        }

        # Only add 'source' if this client was configured with one.
        # Callers that manage multiple sources from one client can include
        # 'source' in their document payload instead.
        if self.source is not None:
            envelope["source"] = self.source

        # Merge envelope into document. Envelope fields take precedence —
        # this prevents callers from accidentally overriding @timestamp or host
        # with incorrect values.
        payload = {**document, **envelope}

        url = f"{self.base_url}/{index}/_doc"
        headers = {
            "Authorization": self.auth_header,
            "Content-Type": "application/json",
        }

        try:
            response = self.requests.post(
                url,
                json=payload,
                headers=headers,
                # TLS certificate verification is disabled because Elasticsearch's
                # default install uses a self-signed certificate. This is acceptable
                # for a private LAN deployment. If this client is ever used over an
                # untrusted network, pass verify='/path/to/ca-cert.pem' instead.
                verify=False,
                timeout=5,
            )

            if response.status_code in (200, 201):
                self.logger.debug(
                    "emit: document accepted. index=%s id=%s",
                    index, response.json().get("_id", "unknown")
                )
                return True
            else:
                self.logger.error(
                    "emit: Elasticsearch rejected document. index=%s status=%s response=%s",
                    index, response.status_code, response.text
                )
                return False

        except self.requests.exceptions.Timeout:
            self.logger.error(
                "emit: timed out submitting to Elasticsearch. index=%s host=%s port=%s",
                index, self.host, self.port
            )
            return False

        except self.requests.exceptions.ConnectionError:
            self.logger.error(
                "emit: connection error submitting to Elasticsearch. index=%s host=%s port=%s. "
                "Is Elasticsearch running and reachable?",
                index, self.host, self.port
            )
            return False

        except Exception as e:
            # Catch-all to ensure telemetry failures never propagate to the caller.
            # Log the full exception for diagnosis but return False gracefully.
            self.logger.error(
                "emit: unexpected error submitting to Elasticsearch. index=%s error=%s",
                index, e
            )
            return False

    def emit_metric(self, document, timestamp=None):
        """Convenience wrapper for emit() targeting the pi-metrics index.

        Args:
            document(dict): Metric document payload. Named fields, e.g.
                            {"temperature_f": 82.4} or
                            {"water_level_cm": 14.2, "pressure_psi": 12.1}
            timestamp(str or datetime or None): See emit().

        Returns:
            success(bool): See emit().
        """
        return self.emit(self.INDEX_METRICS, document, timestamp=timestamp)

    def emit_event(self, document, timestamp=None):
        """Convenience wrapper for emit() targeting the pi-events index.

        Args:
            document(dict): Event document payload. Should include at minimum
                            'level', 'event_type', and 'action' fields. e.g.
                            {
                              "level": "info",
                              "event_type": "vent_action",
                              "action": "opened_increment",
                              "temperature_f": 82.4,
                              "position_voltage": 2.14
                            }
            timestamp(str or datetime or None): See emit().

        Returns:
            success(bool): See emit().
        """
        return self.emit(self.INDEX_EVENTS, document, timestamp=timestamp)
