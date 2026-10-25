# A7683EVideo

Gives a sketch the JPEG frames the camera pipeline encodes.

```cpp
#include <VD66GY_Arduino.h>
#include <A7683EVideo.h>

VisionConfig cfg;
cfg.camera    = CAMERA_VGA;
cfg.transport = TRANSPORT_ETHERNET;
Vision.begin(cfg);

const uint8_t *jpeg; uint32_t len;
if (A7683EVideo.grab(&jpeg, &len, 50)) {
  upload(jpeg, len);
  A7683EVideo.release();
}
```

| Call | Result |
|---|---|
| `begin()`, `active()` | Whether the core selected this transport |
| `grab(&jpeg, &len, waitMs)` | The next frame. `false` on timeout, or while a frame is held |
| `release()` | Returns the buffer |
| `holding()` | Whether a frame is held |
| `framesSeen()`, `framesSkipped()` | Counters |

- The library registers as the `TRANSPORT_ETHERNET` sink. It cannot be linked
  with `<EthernetN6.h>`.
- Call `release()` after every `grab()` that returned `true`. Capture stops
  after two frames held.

Example: `LteThermalUpload`.
