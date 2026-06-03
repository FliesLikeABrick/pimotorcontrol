import argparse
import concurrent.futures
import logging
import time
import os
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# motor leads
CH1 = 26
CH2 = 20

# pulse signal
# multiple pulses per turn is fine, it will just need to be factored in upstream of this
PULSE = 5

# max motor runtime in seconds


# -----------------------------------------------------------------------------
# MCP3008PositionSensor
# -----------------------------------------------------------------------------
# This class provides analog position feedback via an MCP3008 ADC chip, which
# is read over SPI using the Raspberry Pi's hardware SPI interface (spidev).
#
# The MCP3008 is an 8-channel, 10-bit ADC. It returns values from 0 to 1023,
# which this class converts to a real voltage using the configured Vref.
#
# Typical wiring for hardware SPI on Raspberry Pi:
#   MCP3008 VDD  -> 3.3V or 5V (match to Vref)
#   MCP3008 VREF -> same supply as VDD
#   MCP3008 AGND -> GND
#   MCP3008 DGND -> GND
#   MCP3008 CLK  -> GPIO 11 (SPI0 SCLK)
#   MCP3008 DOUT -> GPIO 9  (SPI0 MISO)
#   MCP3008 DIN  -> GPIO 10 (SPI0 MOSI)
#   MCP3008 CS   -> GPIO 8  (SPI0 CE0)
#
# The potentiometer wiper connects to one of the 8 analog input channels
# (CH0-CH7). The channel is selected in software by the 3-bit address sent
# during the SPI transaction.
#
# Note: SPI chip select (CE0/CE1 on the Pi) is separate from the MCP3008
# channel selection. Chip select is handled automatically by spidev when
# you open bus 0, device 0. Channel selection is part of the SPI message
# payload sent to the MCP3008.
# -----------------------------------------------------------------------------

class MCP3008PositionSensor:

    def __init__(self, channel=0, vref=5.0):
        """Initialize an MCP3008 ADC position sensor over hardware SPI.

        Args:
            channel(int): MCP3008 analog input channel to read (0-7).
                          Connect the potentiometer wiper to this channel.
                          Defaults to channel 0.
            vref(float): Reference voltage supplied to the MCP3008 VREF pin,
                         in volts. Raw ADC values are scaled against this to
                         produce a real voltage. Defaults to 5.0V.
                         Common values are 3.3 or 5.0 depending on your
                         supply wiring.
        """
        if channel < 0 or channel > 7:
            raise ValueError(f"MCP3008 channel must be 0-7, got {channel}")

        self.channel = channel
        self.vref = vref

        # Open hardware SPI bus 0, device 0 (CE0).
        # Requires SPI to be enabled on the Pi (raspi-config or /boot/config.txt).
        import spidev
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)

        # MCP3008 max SPI clock is 3.6MHz at 5V Vref, or 1.35MHz at 2.7V.
        # 1MHz is a safe conservative choice that works at both supply voltages.
        self.spi.max_speed_hz = 1000000

    def read(self):
        """Read the current voltage from the configured MCP3008 channel.

        Performs a single-ended reading using the standard 3-byte SPI
        transaction described in the MCP3008 datasheet (section 6.1).

        The 3-byte transaction:
          Byte 0: 0x01          - start bit
          Byte 1: (8 + channel) << 4  - single-ended mode + channel select
          Byte 2: 0x00          - don't care, clocks out the result

        The 10-bit result is reconstructed from bytes 1 and 2 of the response.

        Args:
            None

        Returns:
            voltage(float): Measured voltage in volts, scaled to vref.
        """
        # Build the 3-byte message for a single-ended read on self.channel.
        # Byte 1 sets the SGL/DIFF bit (1 = single-ended) and the channel.
        adc_request = [0x01, (0x08 + self.channel) << 4, 0x00]
        adc_response = self.spi.xfer2(adc_request)

        # The 10-bit result spans the low 2 bits of response byte 1
        # and all 8 bits of response byte 2.
        raw = ((adc_response[1] & 0x03) << 8) | adc_response[2]

        # Convert raw 10-bit value (0-1023) to voltage.
        # We divide by 1023.0 (not 1024) because 1023 is the maximum
        # representable value for a 10-bit ADC (2^10 - 1).
        voltage = (raw / 1023.0) * self.vref
        return voltage

    def close(self):
        """Release the SPI device.

        Args:
            None

        Returns:
            None
        """
        self.spi.close()


# -----------------------------------------------------------------------------
# voltage_to_pct
# -----------------------------------------------------------------------------
# Module-level helper that converts a potentiometer voltage reading into a
# percentage-open value given the configured fully-closed and fully-open
# voltage thresholds.
#
# This is a standalone function rather than a pimc method for two reasons:
#
#   1. Callers that already have a fresh ADC reading cached (e.g. open_increment
#      after a move completes) can pass it directly, avoiding a second ADC read
#      that could return a slightly different value due to pot wiper noise.
#      Consistent with the current_voltage caching pattern used elsewhere.
#
#   2. Scripts that use MCP3008PositionSensor directly without a pimc instance
#      (e.g. greenhouse_metrics.py) can import and call this function with their
#      own reading and configured thresholds, without needing a full pimc instance.
#
# The result is clamped to 0-100 to handle readings that drift slightly outside
# the configured voltage range due to potentiometer wear or ADC noise.
#
# When voltage_tolerance is provided, readings within tolerance of either limit
# are clamped to exactly 0.0 or 100.0, reflecting the same at-limit decision
# that is_fully_open() and is_fully_closed() would make for the same reading.
# -----------------------------------------------------------------------------

def voltage_to_pct(voltage, voltage_fully_closed, voltage_fully_open, voltage_tolerance=0.0):
    """Convert a potentiometer voltage reading to a percentage-open value.

    Calculates where the given voltage falls within the configured travel range
    (voltage_fully_closed to voltage_fully_open) and returns it as a percentage.
    Result is clamped to 0-100 to handle readings that drift slightly outside
    the configured range due to potentiometer wear or ADC noise.

    When voltage_tolerance is provided, readings within that tolerance of either
    limit are clamped to exactly 0.0 or 100.0. This reflects the real-world
    behavior of the motor control logic: if the system considers the motor to be
    fully open or fully closed (i.e. within tolerance of the limit), the reported
    percentage should reflect that state rather than showing e.g. 98.3% when the
    system has decided the motor is at the fully open position. Without this, the
    percentage would appear to contradict the control decisions being logged and
    emitted as telemetry events.

    This function does not read the ADC — the caller is responsible for
    providing a voltage reading. This allows callers that already have a cached
    reading to reuse it, avoiding a second ADC read that could return a
    slightly different value due to pot wiper noise.

    Args:
        voltage(float): The potentiometer voltage reading to convert, in volts.
        voltage_fully_closed(float): The voltage corresponding to 0% open (fully closed).
        voltage_fully_open(float): The voltage corresponding to 100% open (fully open).
        voltage_tolerance(float): Acceptable margin in volts at either limit. Readings
                                  within this tolerance of voltage_fully_open are reported
                                  as 100.0%; readings within tolerance of voltage_fully_closed
                                  are reported as 0.0%. Defaults to 0.0 for backward
                                  compatibility with callers that do not pass tolerance.

    Returns:
        pct(float): Percentage open, clamped to 0.0-100.0. Exactly 0.0 or 100.0
                    when within voltage_tolerance of either limit.
    """
    travel = voltage_fully_open - voltage_fully_closed
    if travel == 0:
        # Avoid division by zero if misconfigured thresholds are identical.
        return 0.0

    # Apply tolerance clamping at limits before calculating percentage.
    # This ensures the reported percentage reflects the same at-limit decision
    # that is_fully_open() and is_fully_closed() would make for the same reading.
    if voltage_tolerance > 0.0:
        if voltage >= (voltage_fully_open - voltage_tolerance):
            return 100.0
        if voltage <= (voltage_fully_closed + voltage_tolerance):
            return 0.0

    pct = (voltage - voltage_fully_closed) / travel * 100.0
    return max(0.0, min(100.0, pct))


class pimc:

    # Valid relay_mode values.
    # 'independent': original behavior. CH1 and CH2 each connect one motor lead
    #                to either a positive or negative supply. The motor is stopped
    #                by setting both leads to the same potential. This is the
    #                original wiper-motor wiring described in pimotorcontrol.
    # 'enable_direction': alternate wiring for motors where one wire is a shared
    #                neutral/common and two separate wires select direction (e.g.
    #                a single-phase AC motor with separate forward and reverse
    #                windings). CH1 is the enable relay (connects neutral/common
    #                to the motor). CH2 is the direction selector relay (selects
    #                which of the two directional leads is energized).
    #
    #                IMPORTANT — sequencing in enable_direction mode:
    #                Neutral (CH1) must always be dropped BEFORE any change to
    #                the direction relay (CH2). Changing direction while the motor
    #                is energized causes the motor to attempt to reverse under load,
    #                which produces mechanical shock and inrush current. The
    #                stop_and_housekeeping() method enforces this sequencing.
    #
    #                The physical mapping of CH2 states to "forward" and "reverse"
    #                is determined entirely by wiring and is intentionally not
    #                defined here. forward() sets CH2 LOW and reverse() sets CH2
    #                HIGH (with CH1 enabled in both cases); which physical direction
    #                corresponds to which CH2 state is the integrator's decision.
    RELAY_MODES = ('independent', 'enable_direction')

    def __init__(self, journal_filename="pimc_status", fake_it=False, open_pulses=11, close_pulses=11, maxtime=30, logger=None, resume=False, open_seconds=10, close_seconds=10,
                 # --- ADC/analog position feedback arguments ---
                 # These arguments enable voltage-based position feedback via an
                 # MCP3008 ADC (or any object with a .read() method returning volts).
                 # If position_sensor is None, the existing pulse-based behavior is
                 # used unchanged, preserving full backward compatibility.
                 position_sensor=None,
                 voltage_fully_open=None,
                 voltage_fully_closed=None,
                 voltage_increment=None,
                 voltage_tolerance=0.1,
                 direction_fault_threshold=0.1,
                 direction_settle_seconds=0.1,
                 poll_interval_seconds=0.01,
                 ch1_pin=CH1,
                 ch2_pin=CH2,
                 relay_mode='independent'):
        """Initialize a new pimc object

        Args:
            journal_filename(str): Absolute or relative (to cwd) path to journal file. Must exist and be non-empty with current system state
            fake_it(bool): If true, all GPIO/motor interactions are emulated for testing. WARNING: journal/state file will still be updated, ensure it is accurate before real usage
            open_pulses(int): Number of pulses to detect for transition from closed to open state
            close_pulses(int): Number of pulses to detect for transition from open to closed state
            logger(obj): Logger object to use; will use root logger if none is passed
            resume(bool): Resume interrupted actions from journal file instead of throwing error
            position_sensor(obj): Optional. An object with a .read() method that returns a float
                                  voltage representing the current motor position. When provided,
                                  voltage-based position feedback is used instead of pulse counting.
                                  An MCP3008PositionSensor instance is the intended use, but any
                                  object implementing .read() -> float will work, which also makes
                                  this easy to mock for testing.
            voltage_fully_open(float): Voltage (in volts) representing the fully open position.
                                       Required if position_sensor is provided. Higher voltage means
                                       more open. The motor will not be driven past this threshold.
            voltage_fully_closed(float): Voltage (in volts) representing the fully closed position.
                                         Required if position_sensor is provided.
            voltage_increment(float): Optional voltage delta per open/close step, used by
                                      open_increment() and close_increment(). Not required at
                                      construction time — only needed if those methods will be
                                      called. The CLI uses per-action voltage deltas instead and
                                      does not require this argument.
            voltage_tolerance(float): Acceptable margin (in volts) when determining whether a target
                                      voltage has been reached. Defaults to 0.1V. See the comment
                                      block in run_until_voltage() for important design notes on
                                      how this interacts with direction_fault_threshold.
            direction_fault_threshold(float): If the voltage moves this many volts in the wrong
                                              direction (after the settling period), the motor is
                                              stopped and a fault is raised. Defaults to 0.1V.
                                              Must be greater than voltage_tolerance; a warning is
                                              logged at init time if this is violated. See the
                                              comment block in run_until_voltage() for details.
            direction_settle_seconds(float): Seconds to wait after starting the motor before
                                             checking for wrong-direction movement. Allows for
                                             motor startup lag and initial ADC noise to settle
                                             before direction validation begins.
                                             Defaults to 0.1s (2x the default poll_interval_seconds).
                                             For applications where each increment moves the motor
                                             for only 50-200ms total, this should be kept small —
                                             a settling period that is a large fraction of total
                                             move time defeats the purpose of direction checking.
                                             A value of 2x-3x poll_interval_seconds is a reasonable
                                             starting point.
            poll_interval_seconds(float): How long to sleep between ADC reads inside the
                                          run_until_voltage() loop. Controls both CPU usage and
                                          the responsiveness of position feedback. Defaults to
                                          0.01s (10ms). For short moves (50-200ms total travel),
                                          smaller values give more position samples per move.
                                          Should be smaller than direction_settle_seconds so that
                                          at least one sample is taken during the settling period
                                          before direction checking begins.
            ch1_pin(int): BCM GPIO pin number for relay channel 1. Defaults to 26.
                          In 'independent' mode: controls motor lead 1.
                          In 'enable_direction' mode: controls the neutral/enable relay.
            ch2_pin(int): BCM GPIO pin number for relay channel 2. Defaults to 20.
                          In 'independent' mode: controls motor lead 2.
                          In 'enable_direction' mode: controls the direction selector relay.
            relay_mode(str): Relay wiring and control paradigm. One of:
                             'independent' (default): original pimotorcontrol behavior.
                                 CH1 and CH2 each connect one motor lead to a supply rail.
                                 Motor is stopped by setting both leads to the same potential.
                             'enable_direction': alternate mode for motors with a shared
                                 neutral/common and separate directional leads (e.g. single-phase
                                 AC motors with forward/reverse windings). CH1 enables the neutral;
                                 CH2 selects direction. Neutral is always dropped before direction
                                 changes. See class-level comment for full wiring notes.
                             NOTE: relay_mode defaults to 'independent' in the Python API for
                             backward compatibility with code that imports pimc directly. When
                             using the CLI, --relay-mode is mandatory — no default is assumed
                             there, because an incorrect relay mode can have unintended physical
                             consequences depending on wiring.
        """
        self.logger = logger or logging.getLogger(__name__)
        self.journal_filename = journal_filename
        self.gpio_initialized = False
        self.journal_executor = concurrent.futures.ThreadPoolExecutor(max_workers=8)
        self.journal_futures = {}  # dict of future to what they were writing
        self.motor_busy = False
        self.faking_it = fake_it
        self.open_pulses = open_pulses
        self.close_pulses = close_pulses
        self.open_seconds = open_seconds
        self.close_seconds = close_seconds
        self.maxtime = maxtime

        # Store configurable pin numbers. Defaulting to the original module-level
        # constants preserves backward compatibility for existing users.
        self.ch1_pin = ch1_pin
        self.ch2_pin = ch2_pin

        # Validate and store relay_mode.
        if relay_mode not in self.RELAY_MODES:
            raise ValueError(
                f"relay_mode must be one of {self.RELAY_MODES}, got '{relay_mode}'"
            )
        self.relay_mode = relay_mode

        # --- ADC/analog position feedback setup ---
        self.position_sensor = position_sensor

        if position_sensor is not None:
            # voltage_fully_open and voltage_fully_closed are required whenever a
            # position_sensor is provided, because they define the hard limits used
            # by is_fully_open(), is_fully_closed(), and run_until_voltage().
            # Without them, no voltage-based movement can be safely bounded.
            #
            # voltage_increment is intentionally NOT required here. It is only
            # needed by open_increment() and close_increment(), which are used by
            # GreenhouseVentController and similar callers that move in fixed steps.
            # The CLI voltage actions compute their own per-call targets from
            # per-action delta arguments and do not use voltage_increment at all.
            # Callers that do need open_increment()/close_increment() are responsible
            # for providing voltage_increment at construction time.
            missing = [name for name, val in [
                ("voltage_fully_open", voltage_fully_open),
                ("voltage_fully_closed", voltage_fully_closed),
            ] if val is None]
            if missing:
                raise ValueError(
                    f"position_sensor provided but the following required voltage "
                    f"arguments are missing: {', '.join(missing)}"
                )

            self.voltage_fully_open = voltage_fully_open
            self.voltage_fully_closed = voltage_fully_closed
            self.voltage_increment = voltage_increment  # may be None; checked in open/close_increment
            self.voltage_tolerance = voltage_tolerance
            self.direction_fault_threshold = direction_fault_threshold
            self.direction_settle_seconds = direction_settle_seconds
            self.poll_interval_seconds = poll_interval_seconds

            # --- Important: tolerance vs. direction_fault_threshold relationship ---
            #
            # These two parameters interact in a subtle but important way, and
            # the warning below exists to catch a misconfiguration that could
            # cause confusing behavior in the field.
            #
            # voltage_tolerance defines how close to the target voltage we consider
            # "close enough" — success. It exists because aging potentiometers,
            # mechanical hard stops, and ADC noise mean we can't always reach an
            # exact voltage target.
            #
            # direction_fault_threshold defines how far the voltage must move in
            # the WRONG direction before we declare a fault and stop the motor.
            # It exists as a safety check: if we commanded the motor to open
            # (voltage should increase) but voltage is falling, something is wrong.
            #
            # The problem arises when direction_fault_threshold <= voltage_tolerance:
            #
            #   Imagine: target = 3.0V, current = 2.95V, tolerance = 0.1V.
            #   We are already within tolerance (2.95 is within 0.1 of 3.0), so
            #   run_until_voltage() would declare success immediately — fine.
            #   But if direction_fault_threshold = 0.05V and the voltage is 2.94V
            #   (just outside tolerance), normal ADC jitter of 0.05V in the
            #   wrong direction would trigger a false fault before the motor even
            #   has a chance to move.
            #
            #   In short: if the fault threshold is tighter than the tolerance,
            #   normal noise near the target voltage looks like a wrong-direction
            #   fault. This is especially relevant for aging potentiometers in
            #   humid environments where ADC readings are less stable.
            #
            # The direction_settle_seconds delay (below) helps by not checking
            # direction until the motor has started moving, but the threshold
            # relationship is the more fundamental constraint.
            #
            # We warn rather than raise an error, because there may be edge cases
            # where a caller knowingly sets these values this way, and a warning
            # in the log is preferable to a hard failure at startup.
            if direction_fault_threshold <= voltage_tolerance:
                self.logger.warning(
                    "direction_fault_threshold (%.3fV) is <= voltage_tolerance (%.3fV). "
                    "This can cause false direction-fault events from ADC noise near the "
                    "target voltage. Consider setting direction_fault_threshold > voltage_tolerance.",
                    direction_fault_threshold, voltage_tolerance
                )

            # --- Important: poll_interval_seconds vs. direction_settle_seconds ---
            #
            # poll_interval_seconds should be smaller than direction_settle_seconds
            # so that at least one ADC sample is taken during the settling window.
            # If poll_interval >= direction_settle_seconds, the settling period is
            # effectively zero — the first sample after loop start is already past
            # the settle window. This is not a hard failure — the code will still
            # work — but it means the settling period provides no benefit and the
            # parameters are misleadingly configured.
            if poll_interval_seconds >= direction_settle_seconds:
                self.logger.warning(
                    "poll_interval_seconds (%.3fs) is >= direction_settle_seconds (%.3fs). "
                    "At least one poll interval should fit within the settling period. "
                    "Consider setting poll_interval_seconds < direction_settle_seconds.",
                    poll_interval_seconds, direction_settle_seconds
                )

        self.status = self.load_journal()
        if not self.faking_it:
            self.gpio_setup()

        if resume and self.status:
            if self.status.split()[0] in ('open', 'closed'):
                self.logger.info("Resume requested but there is no action to resume, current status: %s", self.status)
            elif not self.resume():
                self.logger.error("Resume failed, manual intervention is needed. Current status: %s", self.status)
        else:
            self.logger

    def resume(self):
        """Resume interrupted 'opening' or 'closing' operations from the journal file

        Args:
            None

        Returns:
            status(bool): True if opening/closing was successfully resumed. False if failed or invalid starting state was detected
        """
        remaining_pulses = None
        # load remaining turns from 'opening' or 'closing' states
        status_split = self.status.split()
        if status_split[0] in ('opening', 'closing') and len(status_split) > 1:
            try:
                remaining_pulses = int(status_split[1])
            except ValueError:
                self.logger.error('Cannot resume, opening/closing status has non-integer pulse count: %s', self.status)
                return False

        if status_split[0] == 'opening':
            self.logger.info("Resuming interrupted 'open' action...")
            self.open(remaining_pulses, resuming=True)
            self.logger.info("Resume complete.")
            return True

        if status_split[0] == 'closing':
            self.logger.info("Resuming interrupted 'close' action...")
            self.close(remaining_pulses, resuming=True)
            self.logger.info("Resume complete.")
            return True

        else:
            self.logger.error("Cannot resume from status %s", self.status)
            return False

    def update_status(self, new_status, use_future=True):
        """Update the internal state and journal file to the new value

        Args:
            new_status(str): New status such as: open, closed; opening or closing [followed by pulses remaining]; failure (optionally with additional description)
            use_future(bool): Use concurrent.futures for writing the journal file to avoid blocking

        Returns:
            None
        """
        self.status = new_status
        if use_future:
            self.logger.debug("Creating journal future")
            journal_future = self.journal_executor.submit(self.write_journal)
            self.journal_futures[journal_future] = f"{self.status}-{time.time()}"
            self.logger.debug("Journal future submitted")
        else:
            self.write_journal()

    def load_journal(self):
        """Read and test the journal file

        Args:
            None

        Returns:
            status(str): The system status from the journal file
        """
        if not os.path.exists(self.journal_filename):
            self.logger.critical("Journal file %s does not exist, this must exist to know the current status of the system", self.journal_filename)
            return None

        with open(self.journal_filename) as f:
            status = f.read().strip()

        if not status:
            self.logger.critical("Journal file %s is empty, this must exist to know the current status of the system", self.journal_filename)
            return None

        return status

    def write_journal(self):
        """Write status to the journal and force OS sync; blocking; should be called outside loops or threaded.

        Args:
            None

        Returns:
            None
        """
        with open(self.journal_filename, 'w') as f:
            f.write(self.status)
        self.logger.debug("Journal written")
        os.sync()
        self.logger.debug("Sync complete")

    def cleanup_completed_journal_futures(self):
        """Clean up completed futures used for journal updates

        Args:
            None

        Returns:
            None
        """
        futures_completed = []
        try:
            for future in concurrent.futures.as_completed(self.journal_futures, timeout=0):
                self.logger.debug("Cleaned up future %s", self.journal_futures[future])
                futures_completed.append(future)
        except TimeoutError:
            self.logger.debug("Caught timeouterror, cleaned up all we can")

        # purge the cleaned futures
        for future in futures_completed:
            self.journal_futures.pop(future)

    def wait_pulses(self, pulses, status=None):
        """Wait for the specified number of motor feedback pulses to occur

        Args:
            pulses(int): The number of pulses to wait for. Counting only starts after the first change in GPIO pulse state, and counts on transition from low to high
            status(str): If provided, this string status is used for status and journal updates

        Returns:
            success(bool): True if the operation was completed, False if the time limit was seen first
        """
        # pulse is counted on change from low to high
        pulses_seen = 0
        last_state = None
        start_time = time.time()
        last_pulse = start_time

        while time.time() - start_time < self.maxtime and pulses_seen < pulses:
            self.cleanup_completed_journal_futures()
            time.sleep(0.050)
            state = GPIO.input(PULSE)

            # on first iteration, just read the state
            if last_state is None:
                last_state = state
                continue

            if state > last_state:
                self.logger.debug("Pulse seen. Now %s/%s", pulses_seen, pulses)
                self.logger.debug("Time since last pulse: %s", time.time() - last_pulse)
                last_pulse = time.time()
                pulses_seen += 1
                if status:
                    self.update_status(f"{status} {pulses-pulses_seen}")

            last_state = state

        if pulses_seen >= pulses:
            return True  # saw the pulses
        return False  # hit max motor runtime

    def fake_wait_pulses(self, pulses, status=None):
        """Fake waiting for the specified number of motor feedback pulses to occur, 1 second per fake pulse

        Args:
            pulses(int): The number of pulses to wait for. Counting only starts after the first change in GPIO pulse state, and counts on transition from low to high
            status(str): If provided, this string status is used for status and journal updates

        Returns:
            success(bool): True if the operation was completed, False if the time limit was seen first
        """
        # pulse is counted on change from low to high
        pulses_seen = 0
        last_state = None
        start_time = time.time()

        while time.time() - start_time < self.maxtime and pulses_seen < pulses:
            self.cleanup_completed_journal_futures()
            time.sleep(1)
            last_state = 0  # for faking it
            state = 1

            # on first iteration, just read the state
            if last_state is None:
                last_state = state
                continue

            if state > last_state:
                self.logger.debug("Pulse seen. Now %s/%s", pulses_seen, pulses)
                pulses_seen += 1
                if status:
                    self.update_status(f"{status} {pulses-pulses_seen}")

            last_state = state

        if pulses_seen >= pulses:
            return True  # saw the pulses
        return False  # hit max motor runtime

    def read_position(self):
        """Read the current motor position as a voltage from the ADC position sensor.

        This method requires that a position_sensor was provided at instantiation.
        Callers should check that self.position_sensor is not None before calling,
        or catch the AttributeError that will result if it is.

        Args:
            None

        Returns:
            voltage(float): Current position in volts as returned by the sensor.
        """
        return self.position_sensor.read()

    def read_position_pct(self):
        """Read the current motor position as a percentage open.

        Convenience method for callers that want a percentage and do not already
        have a cached voltage reading. Calls read_position() once and passes the
        result to voltage_to_pct() along with the configured voltage_tolerance,
        so that readings within tolerance of either limit are reported as exactly
        0.0% or 100.0% — consistent with the at-limit decisions made by
        is_fully_open() and is_fully_closed().

        Callers that already have a fresh voltage reading (e.g. after a move
        completes) should call voltage_to_pct() directly with the cached value
        rather than calling this method, to avoid a second ADC read that could
        return a slightly different value due to pot wiper noise.

        Requires position_sensor, voltage_fully_closed, voltage_fully_open,
        and voltage_tolerance to be configured at instantiation.

        Args:
            None

        Returns:
            pct(float): Current position as percentage open, clamped to 0.0-100.0.
                        Exactly 0.0 or 100.0 when within voltage_tolerance of either limit.
        """
        return voltage_to_pct(
            self.read_position(),
            self.voltage_fully_closed,
            self.voltage_fully_open,
            self.voltage_tolerance,
        )

    def is_fully_open(self, current_voltage=None):
        """Determine whether the motor is at or past the fully open position.

        Uses voltage_tolerance so that a motor that is physically stopped
        just short of voltage_fully_open (due to potentiometer wear, mechanical
        hard stops, or ADC noise) is still considered fully open.

        Args:
            current_voltage(float): Optional voltage reading to re-use instead of sampling

        Returns:
            result(bool): True if current position is within tolerance of fully open
                          or beyond it. """
        current_voltage = current_voltage if current_voltage is not None else self.read_position()
        return current_voltage >= (self.voltage_fully_open - self.voltage_tolerance)

    def is_fully_closed(self, current_voltage=None):
        """Determine whether the motor is at or past the fully closed position.

        Uses voltage_tolerance for the same reasons as is_fully_open().

        Args:
            current_voltage(float): Optional voltage reading to re-use instead of sampling

        Returns:
            result(bool): True if current position is within tolerance of fully closed
                          or beyond it.
        """
        current_voltage = current_voltage if current_voltage is not None else self.read_position()
        return current_voltage <= (self.voltage_fully_closed + self.voltage_tolerance)

    def run_until_voltage(self, target_voltage, direction):
        """Run the motor until the ADC reads within tolerance of a target voltage.

        This is the core analog-feedback motor control method. It is the voltage-based
        counterpart to wait_pulses(), and follows a similar structure: run the motor,
        poll feedback, stop on success or timeout.

        The motor must already be running when this method is called (forward() or
        reverse() should be called first). This method only watches the ADC and
        decides when to declare success or fault — it does not start or stop the motor
        itself, to match the structure of the existing wait_pulses() design.

        Args:
            target_voltage(float): The voltage to move toward. Success is declared
                                   when abs(current - target) <= voltage_tolerance.
            direction(str): Expected direction of voltage change. Must be 'increasing'
                            (opening, voltage should rise) or 'decreasing' (closing,
                            voltage should fall). Used for wrong-direction fault detection.

        Returns:
            success(bool): True if target reached within tolerance and time limit.
                           False if timeout or direction fault occurred.

        # --- Design notes on tolerance and direction fault detection ---
        #
        # TOLERANCE:
        # We declare success when we are within voltage_tolerance of the target,
        # rather than requiring an exact match. This is intentional and important
        # for real-world hardware:
        #
        #   - Aging potentiometers may not produce perfectly repeatable voltages
        #     at the same physical position, especially in humid environments.
        #   - The motor may reach a mechanical hard stop slightly before or after
        #     the electrical target voltage.
        #   - ADC readings have inherent noise (typically a few millivolts at 10-bit
        #     resolution with a 5V reference).
        #
        #   Without tolerance, the motor could time out on every single operation
        #   because it can never hit the exact target. This would look like a fault
        #   even when the physical system is working perfectly.
        #
        # DIRECTION FAULT DETECTION:
        # After a settling period (direction_settle_seconds), we check whether the
        # voltage is moving in the expected direction. If it has moved more than
        # direction_fault_threshold volts the wrong way, we stop the motor and
        # return failure.
        #
        # The settling delay is critical: immediately after the relays engage, there
        # may be inrush effects, relay bounce, or mechanical lag before the motor
        # starts turning. Checking direction too early would produce false faults.
        #
        # For applications where each motor move is short (50-200ms total), the
        # settling period should be kept small relative to the total move time —
        # e.g. 2x-3x poll_interval_seconds. A settling period that is a large
        # fraction of total move time means direction faults can only be caught
        # very late in the move, reducing the safety value of the check.
        #
        # IMPORTANT INTERACTION between tolerance and direction_fault_threshold:
        # direction_fault_threshold must be greater than voltage_tolerance.
        # If it is not, then normal ADC noise near the target voltage (which is
        # within tolerance and should trigger success) could instead trigger a
        # direction fault before the success condition is evaluated. This is checked
        # at __init__ time with a logged warning. See __init__ for the full explanation.
        #
        # IMPORTANT INTERACTION between poll_interval_seconds and direction_settle_seconds:
        # poll_interval_seconds should be less than direction_settle_seconds so that
        # at least one ADC sample is taken during the settling window. If poll_interval
        # >= direction_settle_seconds, the settling period is effectively zero — the
        # first sample after loop start is already past the settle window. This is
        # checked at __init__ time with a logged warning.
        #
        # TIMEOUT / STALL DETECTION:
        # The timeout (self.maxtime) catches cases where the motor is running but
        # voltage is not changing — for example, a mechanical bind that stalls the
        # motor short of its target, or the motor hitting its internal hard limit
        # before reaching the electrical target voltage. The timeout exit logs the
        # final voltage, target, and tolerance so that the discrepancy is visible
        # in logs for diagnosis.
        """
        if direction not in ('increasing', 'decreasing'):
            raise ValueError(f"direction must be 'increasing' or 'decreasing', got '{direction}'")

        start_time = time.time()
        settling_complete = False
        # Record the voltage at the moment we start, so we can evaluate direction
        # of movement after the settling period has elapsed.
        voltage_at_settle = None

        while time.time() - start_time < self.maxtime:
            self.cleanup_completed_journal_futures()
            time.sleep(self.poll_interval_seconds)

            current_voltage = self.read_position()
            self.logger.debug("run_until_voltage: current=%.3fV target=%.3fV direction=%s", current_voltage, target_voltage, direction)

            # --- Success check ---
            # Check this before the direction fault check so that a reading that
            # is within tolerance but technically a tiny bit in the wrong direction
            # (pure ADC noise) does not trigger a false fault.
            if abs(current_voltage - target_voltage) <= self.voltage_tolerance:
                self.logger.info(
                    "run_until_voltage: reached target. current=%.3fV target=%.3fV tolerance=%.3fV",
                    current_voltage, target_voltage, self.voltage_tolerance
                )
                return True

            # --- Settling period ---
            # Do not perform direction checks until the motor has had time to
            # start moving. Record the voltage once settling is complete so we
            # have a stable baseline for direction comparison.
            elapsed = time.time() - start_time
            if not settling_complete:
                if elapsed >= self.direction_settle_seconds:
                    settling_complete = True
                    voltage_at_settle = current_voltage
                    self.logger.debug(
                        "run_until_voltage: settling complete. voltage at settle=%.3fV", voltage_at_settle
                    )
                else:
                    # Still in settling period, skip direction check this iteration.
                    continue

            # --- Direction fault check ---
            # Compare current voltage against the voltage recorded at end of
            # settling. If we have moved more than direction_fault_threshold in
            # the wrong direction, something is wrong — stop and fault.
            if direction == 'increasing' and current_voltage < (voltage_at_settle - self.direction_fault_threshold):
                self.logger.error(
                    "run_until_voltage: direction fault. Voltage is DECREASING but expected INCREASING. "
                    "current=%.3fV settle_baseline=%.3fV fault_threshold=%.3fV",
                    current_voltage, voltage_at_settle, self.direction_fault_threshold
                )
                return False

            if direction == 'decreasing' and current_voltage > (voltage_at_settle + self.direction_fault_threshold):
                self.logger.error(
                    "run_until_voltage: direction fault. Voltage is INCREASING but expected DECREASING. "
                    "current=%.3fV settle_baseline=%.3fV fault_threshold=%.3fV",
                    current_voltage, voltage_at_settle, self.direction_fault_threshold
                )
                return False

        # --- Timeout ---
        # The motor ran for maxtime seconds without reaching the target.
        # This may indicate a mechanical stall, a bind in the linkage, or the
        # motor hitting its internal hard limit before the electrical target was
        # reached. Log enough detail to diagnose the discrepancy.
        current_voltage = self.read_position()
        self.logger.error(
            "run_until_voltage: timed out before reaching target. "
            "current=%.3fV target=%.3fV tolerance=%.3fV maxtime=%ss",
            current_voltage, target_voltage, self.voltage_tolerance, self.maxtime
        )
        return False

    def open_increment(self, check_limits=True):
        """Move the motor open by one voltage increment.

        Reads current position, computes a target voltage one increment higher,
        caps it at voltage_fully_open, and runs the motor until that target is
        reached (within tolerance) or a fault occurs.

        Requires voltage_increment to be set at construction time. If it is None,
        a clear error is logged and False is returned.

        If the motor is already at or past the fully open position (within
        tolerance), no movement is attempted.

        Args:
            check_limits(bool): Check whether the target is already fully open before calculating and executing a move; default True. Limits will still not be exceeded by a calculated move increment, but a proactive check will be skipped.

        Returns:
            success(bool): True if the increment was completed successfully.
                           False if already fully open, voltage_increment not set,
                           or if a fault occurred. None if no movement is needed 
                           due to already being at limits (not a fault condition)
        """
        if self.voltage_increment is None:
            self.logger.error(
                "open_increment: voltage_increment is not set. "
                "Provide voltage_increment at construction time to use open_increment()."
            )
            return False
        # cache this starting position to ensure consistency for checks and logging
        # as fluttering marginal voltages could cause confusion during troubleshooting
        current_voltage = self.read_position()

        if check_limits and self.is_fully_open(current_voltage):
            self.logger.info("open_increment: already at or near enough fully open position (%.3fV), not moving", current_voltage)
            return None


        # Compute target: one increment up, but no further than fully open.
        # This means the final increment may be smaller than voltage_increment
        # if we are close to the open limit — we move to fully open rather than
        # stopping short just because a full increment would overshoot.
        target_voltage = min(current_voltage + self.voltage_increment, self.voltage_fully_open)
        self.logger.info(
            "open_increment: current=%.3fV target=%.3fV fully_open=%.3fV",
            current_voltage, target_voltage, self.voltage_fully_open
        )

        if not self.forward():
            self.logger.error("open_increment: aborted, motor busy")
            return False

        result = self.run_until_voltage(target_voltage, direction='increasing')
        self.stop_and_housekeeping()

        # Read final position once and reuse for both voltage and percentage
        # logging, avoiding two ADC reads that could return differing values
        # due to pot wiper noise.
        final_voltage = self.read_position()
        final_pct = voltage_to_pct(final_voltage, self.voltage_fully_closed, self.voltage_fully_open, self.voltage_tolerance)

        if not result:
            self.logger.error(
                "open_increment: failed to reach target voltage. "
                "final=%.3fV (%.1f%% open) target=%.3fV",
                final_voltage, final_pct, target_voltage
            )
            self.update_status("failed opening", use_future=False)
        else:
            self.logger.info(
                "open_increment: move complete. final=%.3fV (%.1f%% open)",
                final_voltage, final_pct
            )
        return result

    def close_increment(self, check_limits=True):
        """Move the motor closed by one voltage increment.

        Reads current position, computes a target voltage one increment lower,
        caps it at voltage_fully_closed, and runs the motor until that target is
        reached (within tolerance) or a fault occurs.

        Requires voltage_increment to be set at construction time. If it is None,
        a clear error is logged and False is returned.

        If the motor is already at or past the fully closed position (within
        tolerance), no movement is attempted.

        Args:
            check_limits(bool): Check whether the target is already fully closed before calculating and executing a move; default True.  Limits will still not be exceeded by a calculated move increment, but a proactive check will be skipped.

        Returns:
            success(bool): True if the increment was completed successfully.
                           False if already fully closed, voltage_increment not set,
                           or if a fault occurred.
        """
        if self.voltage_increment is None:
            self.logger.error(
                "close_increment: voltage_increment is not set. "
                "Provide voltage_increment at construction time to use close_increment()."
            )
            return False

        # cache this starting position to ensure consistency for checks and logging
        # as fluttering marginal voltages could cause confusion during troubleshooting
        current_voltage = self.read_position()

        if check_limits and self.is_fully_closed(current_voltage):
            self.logger.info("close_increment: already at or near enough to fully closed position (%.3fV), not moving", current_voltage)
            return None

        # Compute target: one increment down, but no lower than fully closed.
        # Same partial-increment logic as open_increment — move to fully closed
        # rather than stopping short when within one increment of the limit.
        target_voltage = max(current_voltage - self.voltage_increment, self.voltage_fully_closed)
        self.logger.info(
            "close_increment: current=%.3fV target=%.3fV fully_closed=%.3fV",
            current_voltage, target_voltage, self.voltage_fully_closed
        )

        if not self.reverse():
            self.logger.error("close_increment: aborted, motor busy")
            return False

        result = self.run_until_voltage(target_voltage, direction='decreasing')
        self.stop_and_housekeeping()

        # Read final position once and reuse for both voltage and percentage
        # logging, avoiding two ADC reads that could return differing values
        # due to pot wiper noise.
        final_voltage = self.read_position()
        final_pct = voltage_to_pct(final_voltage, self.voltage_fully_closed, self.voltage_fully_open, self.voltage_tolerance)

        if not result:
            self.logger.error(
                "close_increment: failed to reach target voltage. "
                "final=%.3fV (%.1f%% open) target=%.3fV",
                final_voltage, final_pct, target_voltage
            )
            self.update_status("failed closing", use_future=False)
        else:
            self.logger.info(
                "close_increment: move complete. final=%.3fV (%.1f%% open)",
                final_voltage, final_pct
            )
        return result

    def _require_position_sensor(self, action_name):
        """Check that a position_sensor is configured, logging a clear error if not.

        Used by voltage-based action methods to provide a consistent, informative
        error when the CLI is invoked with a voltage action but --vref (and therefore
        a position_sensor) was not provided.

        Args:
            action_name(str): The name of the calling action, for the error message.

        Returns:
            has_sensor(bool): True if position_sensor is configured, False otherwise.
        """
        if self.position_sensor is None:
            self.logger.error(
                "%s requires a position sensor (ADC). "
                "Provide --vref on the command line to enable ADC position feedback. "
                "Also ensure --fully-open-voltage and --fully-closed-voltage are specified.",
                action_name
            )
            return False
        return True

    def _require_positive_delta(self, action_name, delta, arg_name):
        """Validate that a voltage delta argument is a positive non-zero value.

        Voltage delta arguments (--open-voltage, --close-voltage) must be positive
        absolute values. The direction of movement is implied by the action chosen,
        not by the sign of the delta. A negative or zero value is always a user error:
          - Zero would be a no-op that silently appears to succeed.
          - Negative would move the motor in the opposite direction from what the
            action name implies, causing confusing and potentially unsafe behavior.

        Args:
            action_name(str): The name of the calling action, for the error message.
            delta(float or None): The delta value to validate.
            arg_name(str): The CLI argument name, for the error message.

        Returns:
            valid(bool): True if delta is a positive non-zero float, False otherwise.
        """
        if delta is None:
            self.logger.error(
                "%s requires %s to be specified. "
                "Provide a positive voltage delta (e.g. %s 0.5).",
                action_name, arg_name, arg_name
            )
            return False
        if delta <= 0:
            self.logger.error(
                "%s: %s must be a positive non-zero value, got %.3f. "
                "Always provide an absolute (positive) voltage delta — "
                "the direction of movement is determined by the action, not the sign of the delta.",
                action_name, arg_name, delta
            )
            return False
        return True

    def action_open_voltage(self):
        """Open the motor by a voltage delta from current position.

        Reads current position, adds the configured open_voltage delta, caps at
        voltage_fully_open, and drives the motor forward until the target is reached.

        Requires --vref (position sensor) and --open-voltage to be provided on the CLI.
        --open-voltage must be a positive absolute value; the open direction is implied
        by this action. --fully-open-voltage is used as the upper bound.

        Args:
            None

        Returns:
            result(bool or None): True on success, False on fault, None if misconfigured.
        """
        if not self._require_position_sensor("action_open_voltage"):
            return None
        if not self._require_positive_delta("action_open_voltage", self.open_voltage, "--open-voltage"):
            return None

        if self.is_fully_open():
            self.logger.info("action_open_voltage: already at fully open position, not moving")
            return True

        current_voltage = self.read_position()
        # Cap target at voltage_fully_open so a large delta cannot drive the motor
        # past its defined open limit.
        target_voltage = min(current_voltage + self.open_voltage, self.voltage_fully_open)
        self.logger.info(
            "action_open_voltage: current=%.3fV delta=%.3fV target=%.3fV fully_open=%.3fV",
            current_voltage, self.open_voltage, target_voltage, self.voltage_fully_open
        )

        if not self.forward():
            self.logger.error("action_open_voltage: aborted, motor busy")
            return False

        result = self.run_until_voltage(target_voltage, direction='increasing')
        self.stop_and_housekeeping()

        if not result:
            self.logger.error("action_open_voltage: failed to reach target voltage")
            self.update_status("failed opening", use_future=False)
        return result

    def action_close_voltage(self):
        """Close the motor by a voltage delta from current position.

        Reads current position, subtracts the configured close_voltage delta, caps at
        voltage_fully_closed, and drives the motor in reverse until the target is reached.

        Requires --vref (position sensor) and --close-voltage to be provided on the CLI.
        --close-voltage must be a positive absolute value; the close direction is implied
        by this action and the delta is subtracted internally. --fully-closed-voltage is
        used as the lower bound.

        Args:
            None

        Returns:
            result(bool or None): True on success, False on fault, None if misconfigured.
        """
        if not self._require_position_sensor("action_close_voltage"):
            return None
        if not self._require_positive_delta("action_close_voltage", self.close_voltage, "--close-voltage"):
            return None

        if self.is_fully_closed():
            self.logger.info("action_close_voltage: already at fully closed position, not moving")
            return True

        current_voltage = self.read_position()
        # Cap target at voltage_fully_closed so a large delta cannot drive the motor
        # past its defined closed limit.
        target_voltage = max(current_voltage - self.close_voltage, self.voltage_fully_closed)
        self.logger.info(
            "action_close_voltage: current=%.3fV delta=%.3fV target=%.3fV fully_closed=%.3fV",
            current_voltage, self.close_voltage, target_voltage, self.voltage_fully_closed
        )

        if not self.reverse():
            self.logger.error("action_close_voltage: aborted, motor busy")
            return False

        result = self.run_until_voltage(target_voltage, direction='decreasing')
        self.stop_and_housekeeping()

        if not result:
            self.logger.error("action_close_voltage: failed to reach target voltage")
            self.update_status("failed closing", use_future=False)
        return result

    def action_fully_open_voltage(self):
        """Drive the motor to the fully open voltage position.

        Requires a position_sensor (--vref must be provided on the CLI).
        The target position is set by --fully-open-voltage / voltage_fully_open.
        Drives the motor forward until voltage_fully_open is reached within
        voltage_tolerance, or until maxtime is exceeded.

        Args:
            None

        Returns:
            result(bool or None): True if fully open position reached, False on fault
                                  or timeout, None if no sensor configured.
        """
        if not self._require_position_sensor("action_fully_open_voltage"):
            return None

        if self.is_fully_open():
            self.logger.info("action_fully_open_voltage: already at fully open position, not moving")
            return True

        current_voltage = self.read_position()
        self.logger.info(
            "action_fully_open_voltage: current=%.3fV target=%.3fV (fully open)",
            current_voltage, self.voltage_fully_open
        )

        if not self.forward():
            self.logger.error("action_fully_open_voltage: aborted, motor busy")
            return False

        result = self.run_until_voltage(self.voltage_fully_open, direction='increasing')
        self.stop_and_housekeeping()

        if not result:
            self.logger.error("action_fully_open_voltage: failed to reach fully open position")
            self.update_status("failed opening", use_future=False)
        else:
            self.update_status("open", use_future=False)
        return result

    def action_fully_close_voltage(self):
        """Drive the motor to the fully closed voltage position.

        Requires a position_sensor (--vref must be provided on the CLI).
        The target position is set by --fully-closed-voltage / voltage_fully_closed.
        Drives the motor in reverse until voltage_fully_closed is reached within
        voltage_tolerance, or until maxtime is exceeded.

        Args:
            None

        Returns:
            result(bool or None): True if fully closed position reached, False on fault
                                  or timeout, None if no sensor configured.
        """
        if not self._require_position_sensor("action_fully_close_voltage"):
            return None

        if self.is_fully_closed():
            self.logger.info("action_fully_close_voltage: already at fully closed position, not moving")
            return True

        current_voltage = self.read_position()
        self.logger.info(
            "action_fully_close_voltage: current=%.3fV target=%.3fV (fully closed)",
            current_voltage, self.voltage_fully_closed
        )

        if not self.reverse():
            self.logger.error("action_fully_close_voltage: aborted, motor busy")
            return False

        result = self.run_until_voltage(self.voltage_fully_closed, direction='decreasing')
        self.stop_and_housekeeping()

        if not result:
            self.logger.error("action_fully_close_voltage: failed to reach fully closed position")
            self.update_status("failed closing", use_future=False)
        else:
            self.update_status("closed", use_future=False)
        return result

    def gpio_setup(self):
        """Perform GPIO input/output configuration. Updates self.gpio_initialized; will avoid duplicate setups.

        Args:
            None

        Returns:
            initialization_performed(bool): False if initialization was already done, True if it was performed on this call
        """
        if self.gpio_initialized:
            return False

        # Use instance pin attributes rather than module-level constants, so that
        # pins can be configured at instantiation without changing the defaults.
        GPIO.setup(self.ch1_pin, GPIO.OUT)
        GPIO.setup(self.ch2_pin, GPIO.OUT)

        # Only set up the pulse input pin if we are using pulse-based feedback.
        # When a position_sensor is provided, pulse counting is not used and
        # the PULSE pin is not needed.
        if self.position_sensor is None:
            GPIO.setup(PULSE, GPIO.IN)

        self.gpio_initialized = True
        return True

    def get_status(self):
        return self.status

    def forward(self):
        """Run motor in the forward direction.

        In 'independent' mode: sets CH1 LOW (original behavior).
        In 'enable_direction' mode: sets CH2 LOW (direction select), then
        sets CH1 LOW (enable neutral). CH2 is set before CH1 to ensure the
        direction is selected before the motor is energized.

        The physical meaning of "forward" (which direction the motor turns)
        is determined entirely by wiring and is not defined here.

        Args:
            None

        Returns:
            success(bool): False if motor is already busy, True otherwise.
        """
        if self.motor_busy:
            return False
        self.motor_busy = True
        self.stop_and_housekeeping()
        if self.faking_it:
            return True
        if self.relay_mode == 'enable_direction':
            # Set direction first, then enable neutral.
            # This ensures the motor starts in the correct direction
            # rather than briefly running the wrong way on energization.
            GPIO.output(self.ch2_pin, 0)   # direction: forward
            GPIO.output(self.ch1_pin, 0)   # enable neutral
        else:
            GPIO.output(self.ch1_pin, 0)
        return True

    def reverse(self):
        """Run motor in the reverse direction.

        In 'independent' mode: sets CH2 LOW (original behavior).
        In 'enable_direction' mode: sets CH2 HIGH (direction select), then
        sets CH1 LOW (enable neutral). CH2 is set before CH1 to ensure the
        direction is selected before the motor is energized.

        The physical meaning of "reverse" (which direction the motor turns)
        is determined entirely by wiring and is not defined here.

        Args:
            None

        Returns:
            success(bool): False if motor is already busy, True otherwise.
        """
        if self.motor_busy:
            return False
        self.motor_busy = True
        self.stop_and_housekeeping()
        if self.faking_it:
            return True
        if self.relay_mode == 'enable_direction':
            # Set direction first, then enable neutral.
            GPIO.output(self.ch2_pin, 1)   # direction: reverse
            GPIO.output(self.ch1_pin, 0)   # enable neutral
        else:
            GPIO.output(self.ch2_pin, 0)
        return True

    def stop_and_housekeeping(self):
        """Ensure output is stopped, and block to wait for any pending housekeeping/journal futures needing cleanup.

        In 'enable_direction' mode, the neutral/enable relay (CH1) is dropped
        before the direction relay (CH2) is changed. This is the correct
        sequencing for AC motors with separate directional windings: removing
        power before changing direction avoids the motor attempting to reverse
        under load, which causes mechanical shock and inrush current.

        In 'independent' mode, both relays are set HIGH simultaneously,
        preserving the original behavior.

        Args:
            None

        Returns:
            None
        """
        self.gpio_setup()

        if self.relay_mode == 'enable_direction':
            # Drop neutral first, then set direction relay to a known state.
            # The direction relay state after stop is intentionally left as HIGH
            # (same as independent mode idle state) for consistency, but its
            # value is irrelevant while neutral (CH1) is HIGH/disabled.
            GPIO.output(self.ch1_pin, 1)   # disable neutral first
            time.sleep(0.05)               # brief delay to ensure motor is de-energized
            GPIO.output(self.ch2_pin, 1)   # direction relay to idle state
        else:
            # Original behavior: both HIGH simultaneously.
            GPIO.output(self.ch1_pin, 1)
            GPIO.output(self.ch2_pin, 1)

        time.sleep(0.25)
        self.motor_busy = False

        for future in concurrent.futures.as_completed(self.journal_futures):
            self.logger.debug("Cleaned up future %s", self.journal_futures[future])
        # purge futures data structure
        self.journal_futures = {}

    def action_open_pulses(self):
        """
        Handle user open-pulses request based on configuration
        """
        return self.open(pulses=self.open_pulses)

    def action_open_seconds(self):
        """
        Handle user open-seconds request based on configuration
        """
        return self.open(seconds=self.open_seconds)

    def open(self, pulses=None, seconds=None, resuming=False):
        """Run the motor the specified number of pulses to the fully-opened position

        Args:
            pulses(int): The number of pulses to run. If not specified, the full number of pulses is used
            resuming(bool): Specifies whether this operation is resuming an interrupted operation, to perform proper checks and status/journal updates
        """
        if not resuming:
            if self.status != "closed":
                print(f"Journal says status is {self.status}, not opening")
                return False

        if not self.forward():
            self.logger("Aborted open: motor busy")
            return False

        if not resuming:
            self.update_status("opening")

        if pulses:
            self.logger.debug("Waiting %s pulses", pulses)
            if self.faking_it:
                result = self.fake_wait_pulses(pulses, status="opening")
            else:
                result = self.wait_pulses(pulses, status="opening")

        if seconds:
            self.logger.debug("Waiting %s seconds", seconds)
            time.sleep(seconds)
            result = True

        self.stop_and_housekeeping()

        if result:
            print("Opened")
            self.update_status("open", use_future=False)
            return True
        else:
            print("FAILED during open, hit max runtime")
            self.update_status("failed opening", use_future=False)
            return False

    def action_close_pulses(self):
        """
        Handle user close-pulses request based on configuration
        """
        return self.close(pulses=self.close_pulses)

    def action_close_seconds(self):
        """
        Handle user close-seconds request based on configuration
        """
        return self.close(seconds=self.close_seconds)

    def close(self, pulses=None, seconds=None, resuming=False):
        """Run the motor the specified number of pulses to the fully-closed position

        Args:
            pulses(int): The number of pulses to run. If not specified, the full number of pulses is used
            resuming(bool): Specifies whether this operation is resuming an interrupted operation, to perform proper checks and status/journal updates
        """
        self.logger.info("Closing....")
        if not resuming:
            if self.status != "open":
                print(f"Journal says status is {self.status}, not closing")
                return False

        if not self.reverse():
            self.logger("Aborted close: motor busy")
            return False

        if not resuming:
            self.update_status("closing")

        if pulses:
            if self.faking_it:
                result = self.fake_wait_pulses(pulses, status="closing")
            else:
                result = self.wait_pulses(pulses, "closing")

        if seconds:
            time.sleep(seconds)
            result = True

        self.stop_and_housekeeping()

        if result:
            print("Closed")
            self.update_status("closed", use_future=False)
            return True
        else:
            print("FAILED during close, hit max runtime")
            self.update_status("failed closing", use_future=False)
            return False

    def action_status(self):
        """Print the current system status"""
        print(self.status)

    def run(self, action):
        """
        Run the requested action

        Args:
            action(str): Requested action

        Returns:
            action_output: Return value from the action's callable
        """
        callable_name = f"action_{action.replace('-','_')}"
        self.logger.debug("action callable_name: %s", callable_name)
        action_callable = getattr(motorcontrol, callable_name) if hasattr(motorcontrol, callable_name) else None

        if action_callable is None:
            self.logger.error("Could not find method for requested action `%s`", action)
            return None

        if not callable(action_callable):
            self.logger.error("%s is not callable", callable_name)
        action_callable()


if __name__ == "__main__":

    def get_action_choices():
        action_attributes = [attribute for attribute in dir(pimc) if attribute[0:7] == 'action_']
        available_actions = []
        for attribute in action_attributes:
            if len(attribute) <= 7:
                continue
            available_actions.append(attribute[7:].replace('_', '-'))
        return available_actions

    parser = argparse.ArgumentParser(
        description="pimotorcontrol CLI — motor control swiss army knife for Raspberry Pi relay hats.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
actions:
  open-pulses           Open motor by pulse count (--open-pulses)
  close-pulses          Close motor by pulse count (--close-pulses)
  open-seconds          Open motor by time (--open-seconds)
  close-seconds         Close motor by time (--close-seconds)
  open-voltage          Open motor by voltage delta from current position (--open-voltage, requires --vref)
  close-voltage         Close motor by voltage delta from current position (--close-voltage, requires --vref)
  fully-open-voltage    Drive motor to fully open voltage position (--fully-open-voltage, requires --vref)
  fully-close-voltage   Drive motor to fully closed voltage position (--fully-closed-voltage, requires --vref)
  status                Print current journal status

relay modes:
  independent           CH1 and CH2 each drive one motor lead (original DC wiper motor wiring)
  enable_direction      CH1 enables neutral/common; CH2 selects direction (AC motor wiring)

voltage delta arguments (--open-voltage, --close-voltage):
  Always provide a positive absolute value. The direction of movement is determined
  by the action chosen, not the sign of the delta. The code subtracts the delta
  internally for close operations. A negative or zero value is an error.
        """
    )
    parser.add_argument("action", action="store", choices=get_action_choices(),
                        help="Action to perform (see action list below)")

    # --- required args (no defaults, intentional) ---
    parser.add_argument("--relay-mode", action="store", required=True,
                        choices=pimc.RELAY_MODES,
                        help="Relay wiring mode. REQUIRED: no default assumed due to physical wiring "
                             "consequences. 'independent' for DC motors (original behavior); "
                             "'enable_direction' for AC motors with separate forward/reverse leads.")

    # --- journal / general ---
    parser.add_argument("--resume", action="store_true",
                        help="Resume any prior journaled action before taking new action")
    parser.add_argument("--journal-filename", default="pimc_status", action="store",
                        help="Path to the journal file (default: pimc_status)")
    parser.add_argument("--fake", action="store_true",
                        help="Fake all motor/GPIO interactions for testing")
    parser.add_argument("--debug", action="store_true",
                        help="Verbose logging for debugging")
    parser.add_argument("--max-time", action="store", type=int, default=30,
                        help="Maximum motor runtime per operation in seconds (default: 30)")

    # --- pulse-based args ---
    parser.add_argument("--close-pulses", action="store", type=int, default=11,
                        help="Number of pulses for close operation (default: 11)")
    parser.add_argument("--open-pulses", action="store", type=int, default=11,
                        help="Number of pulses for open operation (default: 11)")

    # --- time-based args ---
    parser.add_argument("--close-seconds", action="store", type=float, default=None,
                        help="Seconds to run motor for close operation")
    parser.add_argument("--open-seconds", action="store", type=float, default=None,
                        help="Seconds to run motor for open operation")

    # --- voltage/ADC args ---
    # --vref is mandatory when any voltage-based action is used. It is not marked
    # required=True in argparse because argparse cannot express conditional requirements
    # based on the chosen action. Instead, the action methods check for a configured
    # position_sensor via _require_position_sensor() and emit a clear error if absent.
    # --vref is the trigger: when provided, an MCP3008PositionSensor is constructed
    # and passed to pimc, enabling all voltage-based actions.
    parser.add_argument("--vref", action="store", type=float, default=None,
                        help="ADC reference voltage in volts. REQUIRED for all voltage-based actions. "
                             "Must match the voltage supplied to the MCP3008 VREF pin exactly. "
                             "No default is provided — an incorrect value produces incorrect "
                             "position readings and unpredictable motor behavior.")
    parser.add_argument("--adc-channel", action="store", type=int, default=0,
                        help="MCP3008 analog input channel for position feedback (default: 0)")
    parser.add_argument("--open-voltage", action="store", type=float, default=None,
                        help="Positive voltage delta to move toward open from current position. "
                             "Must be a positive absolute value — the open direction is implied "
                             "by the open-voltage action. The motor runs forward until "
                             "current_voltage + OPEN_VOLTAGE is reached (capped at --fully-open-voltage).")
    parser.add_argument("--close-voltage", action="store", type=float, default=None,
                        help="Positive voltage delta to move toward closed from current position. "
                             "Must be a positive absolute value — the close direction is implied "
                             "by the close-voltage action and the delta is subtracted internally. "
                             "The motor runs in reverse until current_voltage - CLOSE_VOLTAGE is "
                             "reached (capped at --fully-closed-voltage).")
    parser.add_argument("--fully-open-voltage", action="store", type=float, default=None,
                        help="Voltage at the fully open position. Used as the target for "
                             "fully-open-voltage action and as the upper bound for open-voltage. "
                             "Required for all voltage-based actions.")
    parser.add_argument("--fully-closed-voltage", action="store", type=float, default=None,
                        help="Voltage at the fully closed position. Used as the target for "
                             "fully-close-voltage action and as the lower bound for close-voltage. "
                             "Required for all voltage-based actions.")
    parser.add_argument("--voltage-tolerance", action="store", type=float, default=0.1,
                        help="Acceptable margin in volts for reaching a target voltage (default: 0.1). "
                             "Accounts for potentiometer wear and ADC noise.")
    parser.add_argument("--direction-fault-threshold", action="store", type=float, default=0.1,
                        help="Volts of wrong-direction movement that triggers a fault (default: 0.1). "
                             "Should be greater than --voltage-tolerance to avoid false faults from "
                             "ADC noise near the target voltage.")
    parser.add_argument("--direction-settle", action="store", type=float, default=0.1,
                        help="Seconds to wait after motor start before checking direction (default: 0.1). "
                             "Should be greater than --poll-interval.")
    parser.add_argument("--poll-interval", action="store", type=float, default=0.05,
                        help="Seconds between ADC reads in the position feedback loop (default: 0.05). "
                             "Should be less than --direction-settle.")

    # --- GPIO pin overrides ---
    parser.add_argument("--ch1-pin", action="store", type=int, default=CH1,
                        help=f"BCM GPIO pin for relay channel 1 (default: {CH1})")
    parser.add_argument("--ch2-pin", action="store", type=int, default=CH2,
                        help=f"BCM GPIO pin for relay channel 2 (default: {CH2})")

    logger = logging.getLogger(__name__)
    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)s %(funcName)s: %(message)s (%(filename)s %(lineno)d)")
    else:
        logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

    # --- Construct position sensor if --vref was provided ---
    # All four voltage actions require a sensor. If --vref is absent and a voltage
    # action is requested, the action method will emit a clear error via
    # _require_position_sensor() rather than failing with a less informative traceback.
    position_sensor = None
    if args.vref is not None:
        position_sensor = MCP3008PositionSensor(channel=args.adc_channel, vref=args.vref)

    motorcontrol = pimc(
        fake_it=args.fake,
        open_pulses=args.open_pulses,
        close_pulses=args.close_pulses,
        maxtime=args.max_time,
        logger=logger,
        resume=args.resume,
        journal_filename=args.journal_filename,
        open_seconds=args.open_seconds,
        close_seconds=args.close_seconds,
        position_sensor=position_sensor,
        voltage_fully_open=args.fully_open_voltage,
        voltage_fully_closed=args.fully_closed_voltage,
        voltage_tolerance=args.voltage_tolerance,
        direction_fault_threshold=args.direction_fault_threshold,
        direction_settle_seconds=args.direction_settle,
        poll_interval_seconds=args.poll_interval,
        ch1_pin=args.ch1_pin,
        ch2_pin=args.ch2_pin,
        relay_mode=args.relay_mode,
    )

    # Store per-action voltage deltas directly on the instance so that
    # action_open_voltage and action_close_voltage can access them.
    # These are CLI-only concepts and are not constructor parameters.
    motorcontrol.open_voltage = args.open_voltage
    motorcontrol.close_voltage = args.close_voltage

    action = args.action.lower().strip()
    try:
        motorcontrol.run(action)
    except KeyboardInterrupt:
        logging.warning('Interrupted')
        motorcontrol.stop_and_housekeeping()
