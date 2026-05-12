from __future__ import annotations

import argparse

from robot.agent import BasicRobot
from robot.map_parser import LabMap


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MicroRato2026 Python robot", add_help=False)
    parser.add_argument("-h", "--host", default="localhost", dest="host", help="Simulator host")
    parser.add_argument("-p", "--pos", default=1, type=int, dest="pos", help="Robot id/position")
    parser.add_argument(
        "-r",
        "--robname",
        default="MicroRatoPython",
        dest="robname",
        help="Robot name shown in the simulator",
    )
    parser.add_argument("-m", "--map", dest="map_path", help="Optional lab XML file")
    parser.add_argument("--help", action="help", help="Show this help message and exit")
    return parser


def main():
    args = build_parser().parse_args()

    robot = BasicRobot(args.robname, args.pos, args.host)
    if args.map_path:
        lab_map = LabMap(args.map_path)
        robot.set_map(lab_map.lab_map)
        robot.print_map()

    robot.run()


if __name__ == "__main__":
    main()
