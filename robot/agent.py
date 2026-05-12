from __future__ import annotations

from robot.protocol import RobLinkAngs


DEFAULT_IR_ANGLES = [0.0, 60.0, -60.0, 180.0]
TURN_TOLERANCE_DEG = 8.0
TURN_SPEED = 0.12
FORWARD_SPEED = 0.08
FORWARD_CELL_TIME = 8
BLOCKED_THRESHOLD = 3
COLLISION_FRONT_THRESHOLD = 4
RECOVERY_TOLERANCE_STEPS = 20
CELL_CENTER_STEP = 2

CARDINALS = ("N", "E", "S", "W")
HEADING_TO_DEG = {"N": 90.0, "E": 0.0, "S": -90.0, "W": 180.0}
STEP_OFFSET = {
    "N": (0, CELL_CENTER_STEP),
    "E": (CELL_CENTER_STEP, 0),
    "S": (0, -CELL_CENTER_STEP),
    "W": (-CELL_CENTER_STEP, 0),
}


def normalize_angle(angle: float) -> float:
    while angle <= -180.0:
        angle += 360.0
    while angle > 180.0:
        angle -= 360.0
    return angle


def angle_error(target: float, current: float) -> float:
    return normalize_angle(target - current)


def turn_steps(from_heading: str, to_heading: str) -> int:
    from_idx = CARDINALS.index(from_heading)
    to_idx = CARDINALS.index(to_heading)
    diff = (to_idx - from_idx) % 4
    return diff


def rotate_heading(heading: str, steps: int) -> str:
    idx = CARDINALS.index(heading)
    return CARDINALS[(idx + steps) % 4]


class BasicRobot(RobLinkAngs):
    def __init__(self, rob_name: str, rob_id: int, host: str, angles: list[float] | None = None):
        super().__init__(rob_name, rob_id, angles or DEFAULT_IR_ANGLES, host)
        self.lab_map = None
        # Relative cell-center coordinates in simulator units. The start cell center is (0, 0).
        self.current_position = (0, 0)
        self.explored_positions = {self.current_position}
        self.position_connections: dict[tuple[int, int], set[tuple[int, int]]] = {self.current_position: set()}
        self.heading = "E"
        self.turn_target: str | None = None
        self.forward_target: tuple[int, int] | None = None
        self.forward_until_time: int | None = None
        self.recovery_tolerance_steps = 0

    def set_map(self, lab_map):
        self.lab_map = lab_map

    def print_map(self):
        if self.lab_map is None:
            return
        for row in reversed(self.lab_map):
            print("".join(row))

    def run(self):
        if self.status != 0:
            raise RuntimeError("Connection refused or simulator returned an error")

        state = "stop"
        stopped_state = "run"

        while True:
            self.readSensors()
            self.update_heading_from_compass()

            if self.measures.endLed:
                print(f"{self.robName} exiting")
                return

            if state == "stop" and self.measures.start:
                state = stopped_state

            if state != "stop" and self.measures.stop:
                stopped_state = state
                state = "stop"

            if state == "run":
                if self.measures.visitingLed:
                    state = "wait"
                if self.measures.ground == 0:
                    self.setVisitingLed(True)
                self.step()
            elif state == "wait":
                self.setReturningLed(True)
                if self.measures.visitingLed:
                    self.setVisitingLed(False)
                if self.measures.returningLed:
                    state = "return"
                self.driveMotors(0.0, 0.0)
            elif state == "return":
                if self.measures.visitingLed:
                    self.setVisitingLed(False)
                if self.measures.returningLed:
                    self.setReturningLed(False)
                self.step()

    def update_heading_from_compass(self):
        if not self.measures.compassReady:
            return

        compass = normalize_angle(self.measures.compass)
        candidates = {
            heading: abs(angle_error(target_deg, compass))
            for heading, target_deg in HEADING_TO_DEG.items()
        }
        self.heading = min(candidates, key=candidates.get)

    def is_open(self, sensor_value: float) -> bool:
        return sensor_value <= BLOCKED_THRESHOLD

    def position_in_direction(self, heading: str) -> tuple[int, int]:
        dx, dy = STEP_OFFSET[heading]
        return (self.current_position[0] + dx, self.current_position[1] + dy)

    def neighbor_status(self) -> list[tuple[str, tuple[int, int], bool]]:
        directions = [
            self.heading,
            rotate_heading(self.heading, -1),
            rotate_heading(self.heading, 1),
            rotate_heading(self.heading, 2),
        ]
        sensors = [
            self.measures.irSensor[0],
            self.measures.irSensor[1],
            self.measures.irSensor[2],
            self.measures.irSensor[3],
        ]

        result = []
        for heading, sensor_value in zip(directions, sensors):
            if not self.is_open(sensor_value):
                continue
            position = self.position_in_direction(heading)
            result.append((heading, position, position in self.explored_positions))
        return result

    def choose_next_heading(self) -> str | None:
        options = self.neighbor_status()
        if not options:
            return None

        if self.recovery_tolerance_steps > 0:
            self.recovery_tolerance_steps -= 1
            explored = [heading for heading, position, seen in options if seen]
            if explored:
                return explored[0]
            return options[0][0]

        unexplored = [heading for heading, position, seen in options if not seen]
        if unexplored:
            return unexplored[0]

        # Dead ends happen. Fall back to explored cells only when there is no new cell.
        return options[0][0]

    def choose_recovery_heading(self, avoid_current: bool = True) -> str:
        options = self.neighbor_status()
        if avoid_current:
            options = [option for option in options if option[0] != self.heading]

        explored_options = [heading for heading, position, seen in options if seen]
        if explored_options:
            return explored_options[0]
        if options:
            return options[0][0]

        left_heading = rotate_heading(self.heading, -1)
        right_heading = rotate_heading(self.heading, 1)
        back_heading = rotate_heading(self.heading, 2)

        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]

        side_options = [
            (left, left_heading),
            (right, right_heading),
            (back, back_heading),
        ]
        if avoid_current:
            side_options = [option for option in side_options if option[1] != self.heading]
        side_options.sort(key=lambda item: item[0])
        if side_options:
            return side_options[0][1]
        return self.heading

    def enter_recovery_mode(self):
        self.recovery_tolerance_steps = RECOVERY_TOLERANCE_STEPS

    def begin_turn(self, target_heading: str):
        self.turn_target = target_heading
        print(f"Turning from {self.heading} to {target_heading}")

    def run_turn(self) -> bool:
        if self.turn_target is None:
            return False

        if not self.measures.compassReady:
            steps = turn_steps(self.heading, self.turn_target)
            if steps == 1:
                self.driveMotors(TURN_SPEED, -TURN_SPEED)
            elif steps == 3:
                self.driveMotors(-TURN_SPEED, TURN_SPEED)
            else:
                self.driveMotors(TURN_SPEED, -TURN_SPEED)
            return True

        target_deg = HEADING_TO_DEG[self.turn_target]
        error = angle_error(target_deg, self.measures.compass)
        if abs(error) <= TURN_TOLERANCE_DEG:
            self.heading = self.turn_target
            self.turn_target = None
            self.driveMotors(0.0, 0.0)
            return False

        if error > 0:
            self.driveMotors(-TURN_SPEED, TURN_SPEED)
        else:
            self.driveMotors(TURN_SPEED, -TURN_SPEED)
        return True

    def begin_forward(self, target_position: tuple[int, int]):
        self.forward_target = target_position
        self.forward_until_time = self.measures.time + FORWARD_CELL_TIME
        print(f"Moving from {self.current_position} to {target_position}")

    def run_forward(self) -> bool:
        if self.forward_target is None or self.forward_until_time is None:
            return False

        if self.measures.collisionReady and self.measures.collision:
            print("Collision detected, backing out of forward motion")
            self.forward_target = None
            self.forward_until_time = None
            self.driveMotors(0.0, 0.0)
            self.enter_recovery_mode()
            self.begin_turn(self.choose_recovery_heading(avoid_current=True))
            return False

        if self.measures.irSensor[0] > COLLISION_FRONT_THRESHOLD:
            print("Front blocked, cancelling forward motion")
            self.forward_target = None
            self.forward_until_time = None
            self.driveMotors(0.0, 0.0)
            self.enter_recovery_mode()
            self.begin_turn(self.choose_recovery_heading(avoid_current=True))
            return False

        if self.measures.time >= self.forward_until_time:
            previous_position = self.current_position
            self.current_position = self.forward_target
            self.explored_positions.add(self.current_position)
            self.position_connections.setdefault(previous_position, set()).add(self.current_position)
            self.position_connections.setdefault(self.current_position, set()).add(previous_position)
            self.forward_target = None
            self.forward_until_time = None
            self.driveMotors(0.0, 0.0)
            print(f"Explored positions: {len(self.explored_positions)} current={self.current_position}")
            return False

        self.driveMotors(FORWARD_SPEED, FORWARD_SPEED)
        return True

    def step(self):
        if self.measures.collisionReady and self.measures.collision:
            print("Collision recovery")
            self.forward_target = None
            self.forward_until_time = None
            self.enter_recovery_mode()
            if self.turn_target is None:
                self.begin_turn(self.choose_recovery_heading(avoid_current=True))

        if self.run_turn():
            return

        if self.run_forward():
            return

        next_heading = self.choose_next_heading()
        if next_heading is None:
            recovery_heading = self.choose_recovery_heading(avoid_current=True)
            print(f"No open directions, recovering toward {recovery_heading}")
            self.enter_recovery_mode()
            self.begin_turn(recovery_heading)
            self.run_turn()
            return

        if next_heading != self.heading:
            self.begin_turn(next_heading)
            self.run_turn()
            return

        target_position = self.position_in_direction(next_heading)
        self.begin_forward(target_position)
        self.run_forward()
