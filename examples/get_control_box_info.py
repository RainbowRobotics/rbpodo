import rbpodo as rb
import numpy as np


def _main():
    robot = rb.Cobot("10.0.2.7")
    rc = rb.ResponseCollector()

    res, cb_info = robot.get_control_box_info(rc)
    if res.is_success():
        print(f"Control Box Info: {cb_info}")


if __name__ == "__main__":
    _main()
