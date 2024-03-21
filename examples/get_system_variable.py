import rbpodo as rb

robot = rb.Cobot("10.0.2.7")
rc = rb.ResponseCollector()

[_, out] = robot.get_system_variable(rc, rb.SystemVariable.SD_J1_REF)
rc = rc.error().throw_if_not_empty()

print(out)
