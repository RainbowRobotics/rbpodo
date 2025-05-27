## v0.10.4 (2025-05-27)

### 🐛 Fixes

- **python_binding**: with pybind11 v3.0.0, this repository is now compatible with Numpy 2.x.x

## v0.10.3 (2025-02-17)

### 🐛 Fixes

- change return mechanism on activate and shutdown function
- **python_binding**: set the size of digital in/out to 16 not 6

## v0.10.2 (2024-03-27)

### 🐛 Fixes

- **flush**: flush buffer before using wait functions
- **task_resume**: rename 'reactive_collision' to 'collision'

## v0.10.1 (2024-03-26)

### 🐛 Fixes

- **wait_for_task_loaded**: modify return condition - even if 'return_on_error' is true, when receiving the error message (load nofile), the function returns error

## v0.10.0 (2024-03-25)

### ✨ Features

- add set_user_coordinate()
- add set_payload_info() and set_tcp_info()
- add get_tcp_info() and get_tfc_info()

### 🐛 Fixes

- improve explanation for better clarification

## v0.9.1 (2024-03-21)

### 🐛 Fixes

- fix ci
- fix typo on README.md

## v0.9.0 (2024-03-21)

## v0.1.0 (2024-03-21)

### ✨ Features

- initial commit
