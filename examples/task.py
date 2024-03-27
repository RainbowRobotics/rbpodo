import rbpodo as rb

ROBOT_IP = "10.0.2.7"
TASK_NAME = "aaadfasdfasdfsadf"


def _main():
    try:
        robot = rb.Cobot(ROBOT_IP)
        rc = rb.ResponseCollector()

        robot.set_operation_mode(rc, rb.OperationMode.Simulation)
        robot.set_speed_bar(rc, 0.5)

        robot.task_load(rc, TASK_NAME)
        if robot.wait_for_task_loaded(rc, 1.).type() != rb.ReturnType.Success:
            print("Failed to load task !")
            print("Error:")
            for res in rc.error():
                print(f"  - {res}")
            return
        print("Task loaded successfully")

        robot.flush(rc)

        robot.task_play(rc)
        if robot.wait_for_task_started(rc, 0.1).type() == rb.ReturnType.Success:
            print("Task started ...")
            robot.wait_for_task_finished(rc)
        rc = rc.error().throw_if_not_empty()
    except Exception as e:
        print(e)
    finally:
        print('Exit')


if __name__ == "__main__":
    _main()
