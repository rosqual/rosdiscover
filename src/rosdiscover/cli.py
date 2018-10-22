import logging
import argparse

from .workspace import Workspace

DESC = 'discovery of ROS architectures'

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def launch(fn):
    # type: (str) -> None:
    logger.info("simulating launch: %s", fn)


def main():
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)

    # simulates the architectural effects of a ROS launch
    parser = argparse.ArgumentParser(description=DESC)
    parser.add_argument('filename', type=str, help='a ROS launch file')
    args = parser.parse_args()
    launch(args.filename)
