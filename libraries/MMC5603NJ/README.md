# MMC5603NJ

Driver for the MEMSIC MMC5603NJ magnetometer on the sensor bus.

```cpp
#include <MMC5603NJ.h>
MMC5603NJ mag;
void setup() { mag.begin(); }
void loop()  { magSensorFloatXYZ m = mag.getMag(); }
```

| Call | Result |
|---|---|
| `getMag()` | One measurement in microtesla, with set and reset |
| `getTemp()` | Degrees Celsius |
| `setContinuousMode(rate)` | Continuous sampling at `rate` Hz |
| `sendNS()` | Streams field and temperature to Neuro Studio |

Example: `ReadMagneticField`.

Licence: MIT.
