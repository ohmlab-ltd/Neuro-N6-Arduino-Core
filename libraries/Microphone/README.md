# Microphone

Captures the on-board PDM microphone and streams 16-bit mono PCM to Neuro
Studio.

```cpp
#include <Microphone.h>

Microphone mic;

void setup() {
  mic.begin();
}

void loop() { }
```

| Call | Result |
|---|---|
| `begin()` | Starts capture and streaming |
| `sampleRate()` | 32552 Hz |
| `peak()` | Largest sample magnitude since the last call |
| `streamToNS(on)`, `streaming()` | Streaming to Neuro Studio |
| `available()`, `read(dst, maxSamples)` | Samples, with streaming off |

| Signal | Pin |
|---|---|
| PDM data | PB2 |
| PDM clock | PE2 |

Audio is sent as `META_AUDIO_PCM` records, with or without a camera.

Example: `MicrophoneLevel`.

Licence: MIT, except `src/stm32n6xx_hal_mdf.c`, ST's HAL driver, and the
GPDMA set-up in `src/neuron6_audio.c`, from ST's BSP, which are BSD-3-Clause
(`LICENSE.txt`).
