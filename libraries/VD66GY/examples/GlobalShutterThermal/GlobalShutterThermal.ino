/*
  LICENSING NOTICE

  This example sketch embeds the YOLOv8 model file
  yolov8n_256_quant_pc_ii_seg_coco-st.tflite,
  which is licensed under the GNU Affero General Public License, version 3.0
  (AGPL-3.0), and not under the MIT licence that governs the remainder of
  this repository. Accordingly, this sketch is licensed under AGPL-3.0.

  A firmware image built from this sketch is a work based on that model.
  Any party who distributes such an image, or a product containing it,
  must comply with AGPL-3.0 in full, including the provision of
  Corresponding Source, or must hold an Ultralytics Enterprise License.
  Building and flashing the sketch for personal or internal use does not
  by itself trigger those obligations.

  The STMicroelectronics components that execute the model are supplied
  under ST licence SLA0044, whose terms in most cases forbid the
  distribution of firmware that combines them with AGPL-licensed
  software.

  The full notice and the licence text are in models/README.md and
  models/LICENSE-AGPL-3.0.txt.
*/

/*
  Global shutter thermal

  Streams the global-shutter camera and the thermal camera to Neuro Studio,
  with YOLOv8 instance segmentation on the video and the board's motion and
  magnetic field readings.

  Hardware: Neuro N6 with a VD66GY camera and a FLIR Lepton thermal camera on
  the Thermal carrier.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg
#include <Wire.h>
#include <LSM6DSL_Arduino.h>
#include <MMC5603NJ.h>
#include <NeuroVisionThermal.h>
#include "FreeRTOS.h"
#include "task.h"

VD66GY camera(VD66GY_CARRIER_THERMAL);

NEURON6_DECLARE_MODEL(yolov8_iseg);

LSM6DSL          imu;
MMC5603NJ        mag;
NeuroVisionThermal thermal;

bool imuOK     = false;
bool magOK     = false;
bool thermalOK = false;      // the thermal camera's video link is up
static uint32_t s_thermal_ok_count = 0u;
static uint32_t s_thermal_fail_count = 0u;
static uint32_t s_thermal_consec_fail = 0u;
// True while the thermal camera restarts. It has to boot on a quiet I2C bus,
// so loop() stops reading the sensors.
static volatile bool s_thermal_recovering = false;

// One 160x120 frame.
static uint16_t tframe[NVT_VOSPI_PIXELS];

static void thermalLog(const char *text) {
  Serial.print(text);
  vnd_meta_publish_debug_text(text);
}

// Uses no I2C, so it can run while the main camera streams.
static bool startThermalVideo(const char *when) {
  const bool ok = thermal.videoBegin();
  if (!ok) {
    char line[80];
    snprintf(line, sizeof(line), "[thermal] %s: video link failed, err=%d\r\n",
             when, thermal.videoLastError());
    thermalLog(line);
  }
  return ok;
}

// Collects the thermal frames. It never uses I2C.
static void thermalTask(void *arg) {
  (void)arg;
  uint32_t lastAnnounceTick = 0u;
  for (;;) {
    if (!thermalOK) {
      thermalOK = startThermalVideo("retry");
      if (!thermalOK) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
      thermal.streamAnnounce(true);
      lastAnnounceTick = xTaskGetTickCount();
    }

    // Blocks until the next frame, about nine a second.
    const bool gotFrame = thermal.readFrame(tframe, 1000);

    if (gotFrame) {
      s_thermal_recovering = false;
      thermal.streamFrame(tframe, true);   // true: the pixels are temperatures
      s_thermal_ok_count++;
      s_thermal_consec_fail = 0u;
      if (s_thermal_ok_count == 1u || (s_thermal_ok_count % 256u) == 0u) {
        char tb[64];
        snprintf(tb, sizeof(tb), "[thermal] frame ok #%lu\r\n", (unsigned long)s_thermal_ok_count);
        thermalLog(tb);
      }
    } else {
      s_thermal_fail_count++;
      s_thermal_consec_fail++;
      if (s_thermal_fail_count == 1u || (s_thermal_fail_count % 20u) == 0u) {
        char tb[64];
        snprintf(tb, sizeof(tb), "[thermal] readFrame timeout #%lu err=%d\r\n",
                 (unsigned long)s_thermal_fail_count, thermal.videoLastError());
        thermalLog(tb);
      }
      // The video link restarts after about 20 seconds of silence, and the
      // camera is power-cycled after about a minute.
      if (s_thermal_consec_fail >= 20u) {
        s_thermal_consec_fail = 0u;
        static uint32_t s_recover_cycles = 0u;
        s_recover_cycles++;
        if ((s_recover_cycles % 3u) == 0u) {
          thermalLog("[thermal] no frames, power-cycling the thermal camera\r\n");
          s_thermal_recovering = true;
          thermal.powerOn();
          vTaskDelay(pdMS_TO_TICKS(1500));   // boot time
        }
        thermalOK = false;
        continue;
      }
      if (s_thermal_ok_count == 0u &&
          (xTaskGetTickCount() - lastAnnounceTick) >= pdMS_TO_TICKS(2000)) {
        thermal.streamAnnounce(true);
        lastAnnounceTick = xTaskGetTickCount();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  // The order matters. The thermal camera must boot on a quiet I2C bus, so
  // the sensors start first, then the thermal camera, then the main camera.
  imuOK = imu.begin();
  magOK = mag.begin();

  thermal.powerOn();

  // The thermal camera is mounted upside down on the Thermal carrier.
  thermal.setFlip180(true);

  thermalOK = startThermalVideo("setup");
  if (thermalOK) {
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 5000u) {
      if (thermal.readFrame(tframe, 900)) {
        thermal.streamFrame(tframe, true);
        s_thermal_ok_count = 1u;
        thermal.streamAnnounce(true);
        break;
      }
    }
  }

  char line[80];
  snprintf(line, sizeof(line), "[thermal] imu=%u mag=%u thermal camera=%u\r\n",
           imuOK ? 1u : 0u, magOK ? 1u : 0u, s_thermal_ok_count ? 1u : 0u);
  thermalLog(line);

  VisionConfig cfg;
  cfg.camera      = CAMERA_VD66_FULL;
  cfg.fps         = FPS_45;
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  // The model's input, a square from the middle of the frame.
  cfg.nn.width  = 256;
  cfg.nn.height = 256;
  cfg.nn.aspect = ASPECT_CROP_CENTER;

  cfg.transport   = TRANSPORT_USB;         // the Thermal carrier has no display
  cfg.af          = AF_DISABLED;           // this camera has no autofocus
  cfg.strobe      = STROBE_DISABLED;
  cfg.isp_enabled = VD66_isp_tuning_auto;  // automatic white balance

  Vision.begin(&NN_Instance_yolov8_iseg, PostProcess_YOLOv8_ISEG(0.4f, 0.5f), cfg);
  xTaskCreate(thermalTask, "thermal", 2048, NULL, tskIDLE_PRIORITY + 3u, NULL);
}

void loop() {
  // The model runs about 22 times a second. Running it on every camera frame
  // leaves the thermal camera short of time.
  static uint32_t s_lastNN = 0u;
  if ((millis() - s_lastNN) >= 45u) {
    s_lastNN = millis();
    Vision.run();
  } else {
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  // The sensors wait 4 seconds while the camera sets itself up over I2C.
  static uint32_t s_loop0 = 0u;
  if (s_loop0 == 0u) s_loop0 = millis();
  if (!s_thermal_recovering && (millis() - s_loop0) > 4000u) {
    if (imuOK) imu.sendNS();
    if (magOK) mag.sendNS();
  }
}
