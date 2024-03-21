import rbpodo as rb

robot = rb.Cobot("10.0.2.7")
rc = rb.ResponseCollector()

res = robot.set_operation_mode(rc, rb.OperationMode.Real)
rc = rc.error().throw_if_not_empty()

print(res.type())
