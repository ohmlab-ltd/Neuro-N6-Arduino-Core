# VL53L1CB

Driver for the ST VL53L1CB time-of-flight sensor, read as a 4x4 grid of
distances.

```cpp
#include <VL53L1CB_Arduino.h>
#include <PostProcess.h>

VL53L1CB tof;

void setup() {
  tof.begin();
}

void loop() {
  tofFrame f = tof.read();   // f.mm[4][4], millimetres, 0 = no target
  tof.sendNS();
}
```

- `whoami()` returns `0xEACC`.
- Long distance mode, 100 ms timing budget. A zone reads 0 when its return is
  too weak (under 0.125 MCPS, or sigma over 15 mm).
- `read()` and `sendNS()` step the sweep without blocking. A zone takes 50 to
  100 ms, a full grid up to 1.6 s.
- `begin()` soft-resets the sensor first.
- `sendNS()` streams the grid to Neuro Studio and needs `<PostProcess.h>`.

Examples: `ReadDistance`, `GlobalShutterVision`.

Licence: MIT, except `src/vl53l1cb_st_tables.h`, register values from ST's
VL53L1X ultra lite driver under BSD-3-Clause; the notice is in the file.
