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
class pimc:
    def __init__(self, journal_filename="pimc_status",fake_it=False, open_pulses=11, close_pulses=11, maxtime=30, logger=None, resume=False):
        """Initialize a new pimc object
        Args:
            journal_filename(str): Absolute or relative (to cwd) path to journal file. Must exist and be non-empty with current system state
            fake_it(bool): If true, all GPIO/motor interactions are emulated for testing.  WARNING: journal/state file will still be updated, ensure it is accurate before real usage
            open_pulses(int): Number of pulses to detect for transition from closed to open state
            close_pulses(int): Number of pulses to detect for transition from open to closed state
            logger(obj): Logger object to use; will use root logger if none is passed
            resume(bool): Resume interrupted actions from journal file instead of throwing error"""
        self.logger = logger or logging.getLogger()
        self.journal_filename = journal_filename
        self.gpio_initialized = False
        self.journal_executor = concurrent.futures.ThreadPoolExecutor(max_workers=8)
        self.journal_futures = {} # dict of future to what they were writing
        self.motor_busy = False
        self.faking_it = fake_it
        self.open_pulses = open_pulses
        self.close_pulses = close_pulses
        self.maxtime = 30
        self.status = self.load_journal()
        if not self.faking_it:
            self.gpio_setup()
        if resume and self.status:
            if self.status.split()[0] in ('open','closed'):
                self.logger.info("Resume requested but there is no action to resume, current status: %s", self.status)
            elif not self.resume():
                self.logger.error("Resume failed, manual intervention is needed.  Current status: %s", self.status)
            else:
                self.logger
    def resume(self):
        """Resume interrupted 'opening' or 'closing' operations from the journal file
        Args:
            None
        Returns:
            status(bool): True if opening/closing was successfully resumed.  False if failed or invalid starting state was detected
        """
        remaining_pulses = None
        # load remaining turns from 'opening' or 'closing' states
        status_split = self.status.split()
        if status_split[0] in ('opening','closing') and len(status_split) > 1:
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
        with open(self.journal_filename,'w') as f:
            f.write(self.status)
        self.logger.debug("Journal written")
        os.sync()
        self.logger.debug("Sync complete")
    def cleanup_completed_journal_futures(self):
        """Clean up completed futures  used for journal updates
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
            pulses(int): The number of pulses to wait for.  Counting only starts after the first change in GPIO pulse state, and counts on transition from low to high
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
            time.sleep(0.050)
            state = GPIO.input(PULSE)
            #print(f"{time.time() - start_time} {state}" )
            # on first iteration, just read the state
            if last_state is None:
                last_state = state
                continue
            if state > last_state:
                self.logger.debug("Pulse seen.  Now %s/%s", pulses_seen, pulses)
                pulses_seen += 1
                if status:
                    self.update_status(f"{status} {pulses-pulses_seen}")
            last_state = state
        if pulses_seen >= pulses:
            return True # saw the pulses
        return False # hit max motor runtime
    def fake_wait_pulses(self, pulses, status=None):
        """Fake waiting for the specified number of motor feedback pulses to occur, 1 second per fake pulse
        Args:
            pulses(int): The number of pulses to wait for.  Counting only starts after the first change in GPIO pulse state, and counts on transition from low to high
            status(str): If provided, this string status is used for status and journal updates
        Returns:
            success(bool): True if the operation was completed, False if the time limit was seen first
        """        # pulse is counted on change from low to high
        pulses_seen = 0
        last_state = None
        start_time = time.time()
        while time.time() - start_time < self.maxtime and pulses_seen < pulses:
            self.cleanup_completed_journal_futures()
            time.sleep(1)
            last_state = 0 # for faking it
            state = 1
            #print(f"{time.time() - start_time} {state}" )
            # on first iteration, just read the state
            if last_state is None:
                last_state = state
                continue
            if state > last_state:
                self.logger.debug("Pulse seen.  Now %s/%s", pulses_seen, pulses)
                pulses_seen += 1
                if status:
                    self.update_status(f"{status} {pulses-pulses_seen}")
            last_state = state
        if pulses_seen >= pulses:
            return True # saw the pulses
        return False # hit max motor runtime
    def gpio_setup(self):
        """Perform GPIO input/output configuration.  Updates self.gpio_initialized; will avoid duplicate setups.
        Args:
            None
        Returns:
            initialization_performed(bool): False if initialization was already done, True if it was performed on this call
        """
        if self.gpio_initialized:
            return False
        # 26, CH1, is lead1
        # 20, CH2,  is lead2
        # LOW enables the relay, sends power
        # HIGH disables the relay, sends ground
        GPIO.setup(CH1, GPIO.OUT)
        GPIO.setup(CH2, GPIO.OUT)
        GPIO.setup(PULSE, GPIO.IN)
        self.gpio_initialized = True
        return True
    def get_status(self):
        return self.status
    def forward(self):
        """Run motor in the forward direction
        Args:
            None
        Returns:
            None
        """
        # call stop() to ensure both are HIGH
        # then set CH1 to LOW
        if self.motor_busy:
            return False
        self.motor_busy = True
        self.stop_and_housekeeping()
        if self.faking_it:
            return True
        GPIO.output(CH1, 0)
        return True
    def reverse(self):
        """Run motor in the reverse direction
        Args:
            None
        Returns:
            None
        """
        # call stop() to ensure both are HIGH
        # then set CH2 to LOW
        if self.motor_busy:
            return False
        self.motor_busy = True
        self.stop_and_housekeeping()
        if self.faking_it:
            return True
        GPIO.output(CH2, 0)
        return True

    def stop_and_housekeeping(self):
        """Ensure output is stopped, and block to wait for any pending housekeeping/journal futures needing cleanup
        Args:
            None
        Returns:
            None
        """
        # both HIGH => ground/negative
        self.gpio_setup()
        GPIO.output(CH1, 1)
        GPIO.output(CH2, 1)
        time.sleep(0.25)
        self.motor_busy = False
        for future in concurrent.futures.as_completed(self.journal_futures):
            self.logger.debug("Cleaned up future %s", self.journal_futures[future])
        # purge futures data structure
        self.journal_futures = {}

    def open(self, pulses=None, resuming=False):
        """Run the motor the specified number of pulses to the fully-opened position
        Args:
            pulses(int): The number of pulses to run.  If not specified, the full number of pulses is used
            resuming(bool): Specifies whether this operation is resuming an interrupted operation, to perform proper checks and status/journal updates
        """
        pulses = self.open_pulses if pulses is None else pulses
        if not resuming:
            if self.status != "closed":
                print(f"Journal says status is {self.status}, not opening")
                return False
        if not self.forward():
            self.logger("Aborted open: motor busy")
            return False
        if not resuming:
            self.update_status("opening")
        if self.faking_it:
            result = self.fake_wait_pulses(pulses, status="opening")
        else:
            result = self.wait_pulses(pulses, status="opening")
        self.stop_and_housekeeping()
        if result:
            print("Opened")
            self.update_status("open", use_future=False)
        else:
            print("FAILED during open, hit max runtime")
            self.update_status("failed opening", use_future=False)
    def close(self, pulses=None, resuming=False):
        """Run the motor the specified number of pulses to the fully-closed position
        Args:
            pulses(int): The number of pulses to run.  If not specified, the full number of pulses is used
            resuming(bool): Specifies whether this operation is resuming an interrupted operation, to perform proper checks and status/journal updates
        """
        pulses = self.close_pulses if pulses is None else pulses
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
        if self.faking_it:
            result = self.fake_wait_pulses(pulses, status="closing")
        else:
            result = self.wait_pulses(pulses, "closing")
        self.stop_and_housekeeping()
        if result:
            print("Closed")
            self.update_status("closed", use_future=False)
        else:
            print("FAILED during close, hit max runtime")
            self.update_status("failed closing", use_future=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", action="store", choices=['open','close','status'])
    parser.add_argument("--resume", action="store_true", help="Resume any prior journaled action before taking new action")
    parser.add_argument("--close-pulses", action="store", type=int, default=11, help="Override the number of pulses to close")
    parser.add_argument("--open-pulses", action="store", type=int, default=11, help="Override the number of pulses to open")
    parser.add_argument("--max-time", action="store", type=int, default=30, help="Maximum motor runtime per operation, in seconds")
    parser.add_argument("--journal-filename", default="pimc_status", action="store", help="Path to the journal file")
    parser.add_argument("--fake", action="store_true", help="Fake all motor/GPIO interactions")
    parser.add_argument("--debug", action="store_true", help="Verbose logging for debugging")
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger()
    args = parser.parse_args()

    if args.debug:
        logger.setLevel(logging.DEBUG)
    motorcontrol = pimc(fake_it=args.fake, open_pulses=args.open_pulses, close_pulses=args.close_pulses, maxtime=args.max_time, logger=logger, resume=args.resume, journal_filename=args.journal_filename)
    if args.action.lower() == "open":
        motorcontrol.open()
    elif args.action.lower() == "close":
        motorcontrol.close()
    elif args.action == 'status':
        print(motorcontrol.status)

