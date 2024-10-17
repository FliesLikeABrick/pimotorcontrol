import argparse
import flask
import logging
import pimotorcontrol
app = flask.Flask(__name__)
backend = None


@app.route('/door/action/<name>')
def action(name):
    method = getattr(backend, name)
    if not method:
        return f"Cannot found {name} method on backend"
    return method()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
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
    backend = pimotorcontrol.pimc(fake_it=args.fake, open_pulses=args.open_pulses, close_pulses=args.close_pulses, maxtime=args.max_time, logger=logger, resume=args.resume, journal_filename=args.journal_filename)
    app.run(debug=True, host="0.0.0.0")
