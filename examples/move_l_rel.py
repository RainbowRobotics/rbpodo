import rbpodo as rb
import numpy as np


def _main():
    robot = rb.Cobot("10.0.2.7")
    rc = rb.ResponseCollector()

    try:
        robot.set_operation_mode(rc, rb.OperationMode.Simulation)
        rc = rc.error().throw_if_not_empty()

        target_point = np.array([0, 100, -200, 0, 0, 0])
        robot.move_l_rel(rc, target_point, 300, 400, rb.ReferenceFrame.Base)
        rc = rc.error().throw_if_not_empty()

        if robot.wait_for_move_started(rc, 0.5).is_success():
            robot.wait_for_move_finished(rc)
        rc = rc.error().throw_if_not_empty()
    finally:
        print('Exit')


if __name__ == '__main__':
    _main()
