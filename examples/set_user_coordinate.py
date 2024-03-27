import rbpodo as rb
import numpy as np

ROBOT_IP = "10.0.2.7"


def _main():
    try:
        robot = rb.Cobot(ROBOT_IP)
        rc = rb.ResponseCollector()

        res = robot.set_tcp_info(rc, np.array([0, 0, 0, 0, 0, 0]))
        if not res.is_success():
            print("failed to set tcp")
            exit(-1)
        rc = rc.error().throw_if_not_empty()

        res = robot.set_user_coordinate(rc, 0, np.array([0, 0, 0, 0, 90, 0]))
        if not res.is_success():
            print("failed to set user coordinate")
            exit(-1)
        rc = rc.error().throw_if_not_empty()

        robot.flush(rc)

        robot.move_l_rel(rc, np.array([100, 0, 0, 0, 0, 0]), 100, 200, rb.ReferenceFrame.User0)
        if robot.wait_for_move_started(rc, 0.5).type() == rb.ReturnType.Success:
            robot.wait_for_move_finished(rc)
        rc = rc.error().throw_if_not_empty()
    except Exception as e:
        print(e)


if __name__ == "__main__":
    _main()
