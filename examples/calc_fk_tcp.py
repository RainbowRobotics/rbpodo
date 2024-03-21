import rbpodo as rb
import numpy as np


def _main():
    robot = rb.Cobot("10.0.2.7")
    rc = rb.ResponseCollector()

    pnt = np.zeros((6,))
    robot.calc_fk_tcp(rc, pnt, 0, 0, 0, 0, 0, 0)
    print(pnt)

    [res, pnt] = robot.calc_fk_tcp(rc, np.zeros((6,)))
    print(pnt)


if __name__ == "__main__":
    _main()
