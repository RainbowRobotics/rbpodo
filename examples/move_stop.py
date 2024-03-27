import time
import numpy as np
import rbpodo as rb

ROBOT_IP = "10.0.2.7"


def _main():
    try:
        robot = rb.Cobot(ROBOT_IP)
        rc = rb.ResponseCollector()

        robot.set_operation_mode(rc, rb.OperationMode.Simulation)
        robot.set_speed_bar(rc, 0.5)

        robot.move_jb2_clear(rc)
        robot.move_jb2_add(rc, np.array([90, 0, 0, 0, 0, 0]), 100, 100, 1.0)
        robot.move_jb2_add(rc, np.array([-90, 0, 0, 0, 0, 0]), 100, 100, 1.0)
        robot.move_jb2_add(rc, np.array([90, 0, 0, 0, 0, 0]), 100, 100, 1.0)
        robot.move_jb2_run(rc)

        if robot.wait_for_move_started(rc, 0.5).type() == rb.ReturnType.Success:
            time.sleep(1.0)

            robot.task_stop(rc)
            while robot.get_robot_state(rc)[1] == rb.RobotState.Moving:
                time.sleep(1.0e-3)
            # Or you can use following condition
            # ``robot.wait_for_task_finished(rc)``
        robot.flush(rc)
        rc = rc.error().throw_if_not_empty()
    except Exception as e:
        print(e)
    finally:
        print('Exit')


if __name__ == "__main__":
    _main()
