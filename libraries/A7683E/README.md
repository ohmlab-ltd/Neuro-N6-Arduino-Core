# A7683E

Driver for a SIMCom A7683E LTE Cat-1 bis modem: power control, AT commands,
data context and HTTP upload.

```cpp
#include <A7683E.h>

#define PWRKEY_PIN    MISO
#define RESET_PIN     D9
#define NETLIGHT_PIN  D10

A7683E.begin(PWRKEY_PIN, RESET_PIN, NETLIGHT_PIN);
A7683E.powerOn();
A7683E.waitForReady();
A7683E.waitForNetwork();
A7683E.attachData("uk.lebara.mobi", "wap", "wap");   // your SIM's APN, user, password
A7683E.httpBegin("http://host/ingest", "image/jpeg");
A7683E.httpPost(jpeg, jpegLen);
```

Pass `A7683E_NO_PIN` for a control pin that is not wired.

## Wiring

| Module | Neuro N6 | Notes |
|---|---|---|
| `VCC` | External 5 V | 2 A peak |
| `GND` | Supply ground and N6 ground | |
| `TXD` | `D0`, PD0 | UART4 RX |
| `RXD` | `D1`, PD1 | UART4 TX |
| `PWRKEY` | Any GPIO | Open drain. 500 ms low switches on, 2.5 s low switches off |
| `RESET` | Any GPIO | Open drain. A hard reset |
| `NETLIGHT` | Any GPIO | Input |

The module's UART is 3.3 V.

## API

| Group | Calls |
|---|---|
| Lifecycle | `begin(pwrkey, reset, netlight, baud)`, `beginOn(Stream&, ...)`, `setPins()`, `end()` |
| Power | `powerOn()`, `powerOff()`, `reset()`, `pulsePowerKey(ms)`, `pulseReset(ms)`, `isAlive()` |
| NETLIGHT | `netlightState(windowMs)`, `netlightPeriodMs()`, `netlightName()` |
| Baud rate | `findBaud()`, `syncBaud(baud)`, `baud()` |
| AT | `sendAT()`, `sendATExpect()`, `readLine()`, `lastLine()`, `writeRaw()`, `writeBytes()`, `flushInput()` |
| Status | `waitForReady()`, `simReady()`, `signalQuality()`, `signalDbm()`, `registrationStatus()`, `isRegistered()`, `waitForNetwork()`, `getIMEI()`, `getModel()`, `getOperator()` |
| Data | `setApn()`, `setApnAuth()`, `attachData(apn[, user, pass])`, `getLocalIP()` |
| HTTP | `httpBegin(url, contentType)`, `httpPost(body, len)`, `httpStatus()`, `httpEnd()` |

`netlightState()` returns `NETLIGHT_DARK`, `NETLIGHT_DATA`,
`NETLIGHT_SEARCHING`, `NETLIGHT_REGISTERED`, `NETLIGHT_SOLID` or
`NETLIGHT_UNWIRED`, from the LED's blink period.

## Notes

- `httpPost()` sends a body larger than 153600 bytes in chunks.
- UART4's kernel clock is shared with the BLE HCI link. In a sketch that uses
  both, call `A7683E.begin()` first.
- UART4 has no clock in the `PS_CLOCK_ECO` power profile.
