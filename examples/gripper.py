import time
import rbpodo as rb

robot = rb.Cobot("10.0.2.7")
rc = rb.ResponseCollector()

robot.gripper_rts_rhp12rn_select_mode(rc, rb.GripperConnectionPoint.ToolFlange_Advanced, True)
time.sleep(0.5)

robot.gripper_rts_rhp12rn_force_control(rc, rb.GripperConnectionPoint.ToolFlange_Advanced, 100)
time.sleep(2)

robot.gripper_rts_rhp12rn_force_control(rc, rb.GripperConnectionPoint.ToolFlange_Advanced, -100)
time.sleep(2)

robot.gripper_rts_rhp12rn_force_control(rc, rb.GripperConnectionPoint.ToolFlange_Advanced, 0)
rc = rc.error().throw_if_not_empty()
