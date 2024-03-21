import time
import rbpodo as rb

ROBOT_IP = "10.0.2.7"


def _main():
    robot = rb.Cobot(ROBOT_IP)
    rc = rb.ResponseCollector()

    robot.set_dout_bit_combination(rc, 0, 15, 5, rb.Endian.LittleEndian)
    time.sleep(1)

    robot.set_dout_bit_combination(rc, 0, 15, 10, rb.Endian.LittleEndian)
    time.sleep(1)

    robot.set_dout_bit_combination(rc, 0, 15, 0, rb.Endian.LittleEndian)
    time.sleep(1)


if __name__ == "__main__":
    _main()
