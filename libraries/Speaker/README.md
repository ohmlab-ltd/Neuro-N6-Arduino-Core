# Speaker

Audio output through the on-board MAX98357A I2S amplifier, on SAI1.

```cpp
#include <Speaker.h>
Speaker speaker;

void setup() {
  speaker.begin();
  speaker.setVolume(0.5f);
  speaker.tone(440, 250);     // 440 Hz for 250 ms
}

void loop() { }
```

| Call | Purpose |
|---|---|
| `begin(sampleRate = 48000)` | Starts the output. `sampleRate()` returns the rate in use, about 48828 Hz |
| `tone(freq)`, `tone(freq, ms)`, `noTone()` | Sine tone |
| `write(pcm, n)`, `availableForWrite()` | 16-bit mono PCM. `write()` returns the samples accepted |
| `setVolume(0.0 to 1.0)` | Digital volume |
| `playing()`, `end()` | State, and stop |
| `debugDump()` | Prints the driver state |

| Signal | Pin |
|---|---|
| BCLK | PG1 |
| LRCLK | PG2 |
| DIN | PE3 |
| SD_MODE | PB6 |

- `tone()` takes priority over `write()`. With neither, the output is silent.
- Output is mono: each sample goes to both I2S slots.
- The SAI clock is PLL4 / 16, 25 MHz, so 44.1 kHz and 48 kHz are not exact.
- A DMA error stops the output until `begin()` is called again.

Examples: `SpeakerTone`, `SpeakerClip`.

Licence: MIT, except `src/stm32n6xx_hal_sai.c` and `src/stm32n6xx_hal_sai_ex.c`,
ST's HAL driver, and the GPDMA set-up in `src/neuron6_speaker.c`, from ST's
BSP, which are BSD-3-Clause (`LICENSE.txt`).
