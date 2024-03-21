import rbpodo as rb
import numpy as np


def main():
    robot = rb.Cobot("10.0.2.7")
    rc = rb.ResponseCollector()

    def callback(response: rb.Response):
        print('Message:', response)
        if response.type() == rb.Response.Type.Error:
            print("An error has occurred in the robot. The program will terminate.")
            exit(-1)
    rc.set_callback(callback)

    try:
        robot.set_operation_mode(rc, rb.OperationMode.Simulation)
        joint = np.array([0, 0, 0, 0, 0, 0])
        robot.move_j(rc, joint, 60, 80)
        if robot.wait_for_move_started(rc, 0.5).is_success():
            robot.wait_for_move_finished(rc)
    finally:
        print('Exit')


if __name__ == '__main__':
    main()
