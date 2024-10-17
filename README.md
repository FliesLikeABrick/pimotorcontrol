# pimotorcontrol
Raspbery Pi motor control, with pulse-based position feedback.  Great for use with automotive wiper motors and a raspberry pi relay hat

# Background
This was originally written to automate the swinging door on our chicken coop, using a 4-lead automotive windshield wiper motor.

With a Raspberry Pi relay hat controlled by GPIO, two relays are able to change the direction of the motor by swapping the polarity.

Many wiper motors have a built-in park switch.  In the vehicle this is used by a relay or computerized module to cut power to the motor when the wiper is in the park position.  However, this also provides a simple way to count the number of turns of a motor, for basic positional feedback.

This code is written in such a way that the desired positions are defined by pulses, so it can use other feedback mechanisms, such as multiple pulses per turn for more precise control - like if a motor is equipped with a hall effect sensor or optical feedback mechanism.

# Features
- Out of the box should work with any DC/reversible motor, due to using the relays to change polarity of power to the motor
- Provides a reasonably flexible but basic command line; or can be imported and used from other code for back to back movements without reinitialization
- System status/movements are journaled to disk to allow for recovery from power or other interruptions to the system.  This is useful in open-loop systems that cannot self-home
- Journaling to disk is done with explicit disk synchronization to increase reliability, but without blocking timing of the motor pulse monitoring loop (by using concurrent.futures thread executors)
- dry-run "fake" mode to allow for testing and observing behavior before engaging GPIO for motor outputs or pulse inputs


# Assumptions
 - The motor direction can be controlled by swapping two relays between their NC and NO positions (with Off being achieved by putting both relays in their NC positions)
 - A basic circuit is provided between the Pi and the feedback mechanism providing the pulses in on a 3.3v GPIO pin

# Examples

Go from open to close position by watching for 3 pulses. Resume any partial operation that was interrupted on the last run.  In this case the system (according to the state/journal file) was already in the closed position.

    flieslikeabrick@coop:~ $ python3 motorcontrol.py  --resume --close-pulses=3 close
    INFO:root:Resume requested but there is no action to resume, current status: closed
    INFO:root:Closing....
    Journal says status is closed, not closing
    flieslikeabrick@coop:~ $ 

Go from the closed to open position by watching for the default number of pulses.  However, do this in dry-run mode (will not read or write from GPIO)

    flieslikeabrick@coop:~ $ python3 motorcontrol.py --fake open
    [10 second pause here]
    Opened
    flieslikeabrick@coop:~ $ 


Go from open to closed, fake/dryrun, with debug messages:

    flieslikeabrick@coop:~ $ python3 motorcontrol.py --fake  --debug --close-pulses=1 close
    INFO:root:Closing....
    DEBUG:root:Creating journal future
    DEBUG:root:Journal written
    DEBUG:root:Journal future submitted
    DEBUG:root:Caught timeouterror, cleaned up all we can
    DEBUG:root:Sync complete
    DEBUG:root:Pulse seen.  Now 0/1
    DEBUG:root:Creating journal future
    DEBUG:root:Journal future submitted
    DEBUG:root:Journal written
    DEBUG:root:Sync complete
    DEBUG:root:Cleaned up future closing 0-1729130580.0544508
    DEBUG:root:Cleaned up future closing-1729130579.0074098
    Closed
    DEBUG:root:Journal written
    DEBUG:root:Sync complete
    flieslikeabrick@coop:~ $ 

Move to the open position, resuming a prior interrupted operation (if applicable), which in this case there was.  That is, the journal file before this command said "opening 4" which meant "opening, waiting for 4 more pulses".  If the interrupted operation was "opening" but the current command was "close", then the system would finish opening and then close normally.

    flieslikeabrick@coop:~ $ python3 motorcontrol.py --fake  --resume --open-pulses=5 open
    INFO:root:Resuming interrupted 'open' action...
    Opened
    INFO:root:Resume complete.
    Journal says status is open, not opening
    flieslikeabrick@coop:~ $


# Possible future work
 - Continue generalizing some hard-coded behaviors
 - Add feedback mechanisms such as limit switches for fully-open or fully-closed positions
 - Possibly generalize "positions" by their "pulse" location, instead of just a hard-coded logical "open" and "close".  This can already be achieved to some degree by using the per-command --open-pulses and --close-pulses arguments
 - Basic Flask interface
