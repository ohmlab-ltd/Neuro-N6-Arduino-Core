# LSM6DSL

Driver for the ST LSM6DSL IMU on the sensor bus.

```cpp
#include <LSM6DSL_Arduino.h>
LSM6DSL imu;
void setup() { imu.begin(); }
void loop()  { imuVec3 a = imu.readAccel(); }
```

| Call | Result |
|---|---|
| `begin()` | Finds the sensor at 0x6A or 0x6B and sets 104 Hz |
| `readAccel()` | g |
| `readGyro()` | Degrees per second |
| `readTemp()` | Degrees Celsius |
| `sendNS()` | Streams the readings to Neuro Studio as `META_IMU_FRAME` |

`extras/st-driver` holds ST's component driver. It is not compiled.

Example: `ReadAcceleration`.

Licence: MIT. ST's driver in `extras/st-driver` is BSD-3-Clause, see its
`LICENSE.md`.
