import rbpodo as rb
import numpy as np


def _main():
    robot = rb.Cobot("10.0.2.7")
    rc = rb.ResponseCollector()

    robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    robot.set_speed_bar(rc, 0.5)

    joint = np.array([180 * 10, 0, 0, 0, 0, 0])  # Raise an error!
    robot.move_j(rc, joint, 60, 80)

    if robot.wait_for_move_started(rc, 0.5).type() == rb.ReturnType.Success:
        print("Start moving ...")
        robot.wait_for_move_finished(rc)
    for res in rc.error():
        if res.category() == "code":
            print(f"Error Code: {res.msg()}, Message: {rb.ErrorCodeMessage[int(res.msg())].en}")


if __name__ == '__main__':
    _main()
