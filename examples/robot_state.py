import time
import rbpodo as rb
import numpy as np


def _main():
    robot = rb.Cobot("10.0.2.7")
    rc = rb.ResponseCollector()

    try:
        robot.set_operation_mode(rc, rb.OperationMode.Simulation)
        rc = rc.error().throw_if_not_empty()

        joint = np.array([0, 0, 0, 0, 0, 0])
        robot.move_j(rc, joint, 60, 80)
        rc = rc.error().throw_if_not_empty()

        time.sleep(0.25)
        while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
            pass
    finally:
        print('Exit')


if __name__ == '__main__':
    _main()
