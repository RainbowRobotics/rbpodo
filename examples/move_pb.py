import rbpodo as rb
import numpy as np

ROBOT_IP = "10.0.2.7"


def _main():
    try:
        robot = rb.Cobot(ROBOT_IP)
        rc = rb.ResponseCollector()

        robot.set_operation_mode(rc, rb.OperationMode.Simulation)
        robot.set_speed_bar(rc, 0.5)

        robot.move_pb_clear(rc)
        robot.move_pb_add(rc, np.array([100, 200, 200, 90, 0, 0]), 200.0, rb.BlendingOption.Ratio, 0.5)
        robot.move_pb_add(rc, np.array([300, 300, 400, 0, 0, 0]), 400.0, rb.BlendingOption.Ratio, 0.5)
        robot.move_pb_add(rc, np.array([0, 200, 400, 90, 0, 0]), 200.0, rb.BlendingOption.Ratio, 0.5)

        # **Important**
        # Before you start move, flush buffer to response collector to avoid unexpected behavior in 'wait' function
        robot.flush(rc)
        rc = rc.error().throw_if_not_empty()
        rc.clear()

        robot.move_pb_run(rc, 800, rb.MovePBOption.Intended)
        if robot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
            robot.wait_for_move_finished(rc)
        rc = rc.error().throw_if_not_empty()
    except Exception as e:
        print(e)


if __name__ == "__main__":
    _main()
