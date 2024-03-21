# CobotData Class

## ``CobotData`` constructor

```c++
explicit CobotData(const std::string& address, int port = kDataPort);
```

## request_data

```c++
std::optional<SystemState> request_data(double timeout = -1.) const;
```

Return type is [SystemState](./other_types.md#system-state).