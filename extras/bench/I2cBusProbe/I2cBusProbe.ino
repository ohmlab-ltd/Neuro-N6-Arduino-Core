/*
  I2C bus probe

  Probes every address on the camera I2C bus once every two seconds and prints
  whatever answers.

  Hardware: Neuro N6 with a camera, or any other I2C device, on the camera
  I2C bus: SDA on PE6, SCL on PH9.

  This example code is in the public domain.
*/

#include <NeuroN6_app.h>

static const char *i2c_known(uint8_t addr7) {
  switch (addr7) {
    case 0x10: return "  <- VD66GY (expected 8-bit 0x20)";
    case 0x3C: return "  <- OV5640 (8-bit 0x78)";
    default:   return "";
  }
}

void setup() {
  VisionConfig cfg;
  cfg.transport = TRANSPORT_USB;
  Vision.begin(cfg);

  delay(1500);

  NeuroN6_DbgPrint(
    "\r\n[I2CSCAN] bus = hi2c1 (I2C1: SDA=PE6 SCL=PH9). Scanning 7-bit 0x08..0x77 every 2 s.\r\n"
    "[I2CSCAN] VD66GY = 0x10 (8-bit 0x20), OV5640 = 0x3C (8-bit 0x78).\r\n");
}

void loop() {
  char b[112];
  NeuroN6_DbgPrint("[I2CSCAN] ---- scan ----\r\n");

  int found = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    // Two tries of 5 ms each. The lock is per probe: the camera driver shares
    // the bus.
    n6_i2c_lock();
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 5);
    n6_i2c_unlock();
    if (st == HAL_OK) {
      snprintf(b, sizeof(b), "[I2CSCAN]   ACK  7-bit 0x%02X  /  8-bit 0x%02X%s\r\n",
               (unsigned)a, (unsigned)(a << 1), i2c_known(a));
      NeuroN6_DbgPrint(b);
      found++;
    }
  }

  if (found == 0) {
    NeuroN6_DbgPrint(
      "[I2CSCAN]   (nothing acked -- bus empty: sensor unpowered / held in reset / not wired)\r\n");
  }
  snprintf(b, sizeof(b), "[I2CSCAN] ---- %d device(s) ----\r\n", found);
  NeuroN6_DbgPrint(b);

  delay(2000);
}
