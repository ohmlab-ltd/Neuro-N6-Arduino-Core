# Third-party notices

The MIT licence in `LICENSE` covers the code Ohm Lab wrote. An example sketch
whose header says so is in the public domain. Third-party components, and the
parts of Ohm Lab files that come from third-party code, keep their own
licences, listed here. Anything not listed is Ohm Lab code.

## Models

- `models/nia_1_int8.onnx`, Ohm Lab's Nia network, is under the Ohm Lab Model
  Licence (`models/LICENSE-OhmLab-Model.txt`), not MIT. It may be used in any
  form, including the code and weights generated from it, only on Ohm Lab
  hardware.
- `models/yolov8n_256_quant_pc_ii_seg_coco-st.tflite` and
  `models/yolov8n_256_quant_pc_uf_pose_coco-st.tflite` derive from Ultralytics
  YOLOv8 and are under the GNU Affero General Public License, version 3.0
  (AGPL-3.0), text in `models/LICENSE-AGPL-3.0.txt`. The example sketches that
  embed them are AGPL-3.0 too, and each says so in its header. No other part of
  this repository is AGPL-3.0. `models/README.md` sets out the terms, including
  their interaction with ST's SLA0044 licence for the software that runs them.
- `models/centerface-hwc.tflite`, Star-Clouds' CenterFace, is MIT
  (`models/LICENSE-MIT-CenterFace.txt`).
- `models/face4-int8.onnx`, foamliu's MobileFaceNet, Copyright 2019 foamliu,
  is Apache-2.0 (`models/face4-int8.onnx.license`), exported to ONNX and
  quantised by Ohm Lab. It was trained on InsightFace's MS1M data, which
  InsightFace provides for non-commercial research purposes only.
- The other bundled models and their licences are in `models/README.md`. Each
  model file has a `.license` file beside it naming its copyright holders and
  licence.

## Licences

| Licence | Components |
|---|---|
| MIT | Ohm Lab code, TinyUSB, FreeRTOS, six files in the NXP IW610 driver, the CenterFace model |
| BSD-3-Clause | STM32 HAL, ST project files from STM32CubeN6, ST sensor drivers and fonts, ST encoder wrapper layer, Hantro video encoder, lwIP, pyserial, NXP IW610 driver, ST VL53L1X register values, ST audio pre-processing, the GPDMA set-up of the Microphone and Speaker drivers |
| BSD-2-Clause | AprilTag tag36h11 codes |
| Apache-2.0 | Arm CMSIS, ST CMSIS device headers and the startup and system files made from their templates, `partition_stm32n657xx.h`, two files in the NXP IW610 driver, five models, the ImageNet class names, ZXing's `patternMatchVariance` in Barcode1D |
| ISC | Quirc |
| LGPL-2.1 | ArduinoCore-API, ArduinoBLE |
| AGPL-3.0 | Two YOLOv8 model files and the twelve sketches that embed them: eleven library examples and one bench sketch in `extras/bench` |
| Ohm Lab Model Licence | The Nia model and every form generated from it |
| ST SLA0044 | ST Edge AI runtime and its NPU cache files, vision post-processing, two files from ST's post-processing wrapper, the ST-derived parts of `NeuroN6.c` and `imageEncode.c`, the camera middleware, the VD66GY ISP middleware, generated network code, the audio event model |
| NXP proprietary | IW610 controller firmware |
| None stated | OmniVision OV5640 autofocus firmware |
| CC-BY-4.0 | `CODE_OF_CONDUCT.md`, from the Contributor Covenant |

`cores/NeuroN6/api` and `libraries/ArduinoBLE` are LGPL-2.1 and are linked
statically into a firmware image. A recipient of the image must be able to
relink it with a modified version of that code: offer the object files or the
source.

## Core

A code change by Ohm Lab is marked "Ohm Lab" in the file. The exceptions are
changed throughout: `FreeRTOSConfig.h`, `tusb_config.h`, the ST project files,
`ov5640.c` and `ov5640.h`. Dashes and curly quotes in third-party comments and
documents are replaced with ASCII characters.

| Path | Component | Version | Licence | Files Ohm Lab changed |
|---|---|---|---|---|
| `cores/NeuroN6/STM32N6xx_HAL_Driver/` | STM32N6 HAL and LL drivers | 1.2.0. The RTC and LL USB files are 1.3.0 | BSD-3-Clause | `Src/stm32n6xx_hal_sdio.c`, `stm32n6xx_hal_dcmipp.c`, `stm32n6xx_hal_cortex.c`, `stm32n6xx_ll_venc.c` |
| `cores/NeuroN6/CMSIS/` | Arm CMSIS | 5.9.0 | Apache-2.0 | None |
| `cores/NeuroN6/CMSIS/Device/ST/STM32N6xx/` | ST device headers | 1.2.0 | Apache-2.0 | `Include/stm32n657xx.h` |
| `cores/NeuroN6/FreeRTOS/`, `FreeRTOSConfig.h` | FreeRTOS kernel | V10.6.2 | MIT | `portable/GCC/ARM_CM55_NTZ/non_secure/port.c`, `FreeRTOSConfig.h` |
| `cores/NeuroN6/class/`, `device/`, `host/`, `portable/`, `common/`, `typec/`, `osal/`, `tusb.c`, `tusb.h`, `tusb_option.h`, `tusb_config.h`, `CMakeLists.txt`, `tinyusb.mk`, `usb_descriptors.c` | TinyUSB | 0.20.0 | MIT | `portable/synopsys/dwc2/dcd_dwc2.c`, `portable/synopsys/dwc2/dwc2_stm32.h`, `osal/osal_freertos.h`, `tusb_config.h`. `usb_descriptors.c` is adapted from TinyUSB's device examples |
| `cores/NeuroN6/api/` | ArduinoCore-API | 1.5.2 | LGPL-2.1 | None. `Common.h` and `itoa.c` are Ohm Lab files |
| `cores/NeuroN6/AI_Runtime/` | ST Edge AI runtime | 4.0 in `Npu/ll_aton/`, 2.2.0 in `Inc/` and `Lib/` | SLA0044 | In `Npu/ll_aton/`: `ll_aton_runtime.c`, `ll_aton_NN_interface.h`, `ll_aton_osal_freertos.c`, `ll_aton_osal_threadx.c`, `ll_aton_osal_zephyr.c`. `ll_aton_osal_rtos_template.c` is renamed `ll_aton_osal_rtos_template.h` |
| `cores/NeuroN6/lib_vision_models_pp/` | ST vision model post-processing | | SLA0044 | `lib_vision_models_pp/Inc/mpe_pp_output_if.h` |
| `cores/NeuroN6/Third_Party/VideoEncoder/` | Hantro VC8000NanoE encoder | 9.22.3.7, with ST's changes to 2024-12-17 | BSD-3-Clause, Verisilicon and Google | `inc/basetype.h`, `inc/encswhwregisters.h`, `source/common/encInputLineBuffer.c`, `source/h264/H264CabacContext.h`, `source/jpeg/EncJpegInstance.h`. `source/jpeg/` holds copies of eleven `inc/` headers |
| `cores/NeuroN6/ST/VideoEncoder_EWL/` | ST encoder wrapper layer | 1.2.0 | BSD-3-Clause | `ewl_impl.c`. `ewl_conf.h` is ST's template, filled in |
| `cores/NeuroN6/imageEncode.c` | Ohm Lab file made from `FSBL/Src/imageEncode.c` of ST's `VENC_JPEG_Encoding` application in STM32CubeN6 | | SLA0044 for ST's part, see `AI_Runtime/LICENSE.txt` | Changed throughout |
| `cores/NeuroN6/NeuroN6.c` | Ohm Lab file with parts derived from ST's example and application code, as its header states | | SLA0044 for those parts, see `AI_Runtime/LICENSE.txt` | Changed throughout |
| `cores/NeuroN6/partition_stm32n657xx.h` | CMSIS partition header, STM32CubeMX output | | Apache-2.0 | Comment markers removed |
| `cores/NeuroN6/stm32_assert.h` | ST assert header | | BSD-3-Clause | None |
| `cores/NeuroN6/startup_stm32n657i0hxq.S`, `system_stm32n6xx_s.c` | ST CMSIS device templates, through STM32CubeMX | | Apache-2.0, see `CMSIS/Device/ST/STM32N6xx/LICENSE.txt` | Both |
| `cores/NeuroN6/stm32n6xx_hal_conf.h`, `stm32n6xx_hal_msp.c`, `stm32n6xx_hal_timebase_tim.c`, `stm32n6xx_it.c`, `stm32n6xx_it.h`, `secure_nsc.c`, `secure_nsc.h`, `syscalls.c`, `sysmem.c`, `lcd.h`, `system/linker/STM32N657I0HXQ_LRUN.ld` | STM32CubeMX and STM32CubeIDE output from STM32CubeN6's HAL templates, project templates and examples; `lcd.h` is the BSP common component | | BSD-3-Clause, see `STM32N6xx_HAL_Driver/LICENSE.txt` | All but `lcd.h` |
| `cores/NeuroN6/mcu_cache.c`, `npu_cache.c` | ST Edge AI NPU device files | | SLA0044, see `AI_Runtime/LICENSE.txt` | Both |

## Libraries

| Path | Component | Version | Licence | Files Ohm Lab changed |
|---|---|---|---|---|
| `libraries/ArduinoBLE/` | ArduinoBLE | 2.1.0 | LGPL-2.1 | `src/utility/HCI.cpp`, `HCI.h`, `ATT.cpp`, `HCITransport.h`, `src/local/BLELocalDevice.cpp`, `BLELocalCharacteristic.cpp`, `library.properties`. `src/utility/HCIMayaW476Transport.cpp` is an Ohm Lab file |
| `libraries/Quirc/` | quirc, Daniel Beer | 1.0 | ISC | `src/quirc.c`, `src/quirc_internal.h`, `src/decode.c`. `examples/` holds Ohm Lab files |
| `libraries/VL53L1CB/src/vl53l1cb_st_tables.h` | The default configuration from ST's VL53L1X ultra lite driver (STSW-IMG009) | | BSD-3-Clause, the notice is in the file | None. The rest of `VL53L1CB` is Ohm Lab code |
| `libraries/N6lwIP/src/lwip/` | lwIP, with NXP's port in `port/` | 2.2.1 | BSD-3-Clause | `port/lwipopts.h`, `port/sys_arch/dynamic/sys_arch.c`. The 6LoWPAN sources, `netif/lowpan6*.c` and `netif/zepif.c`, and their headers are removed, with their entries in `Filelists.cmake`, `Filelists.mk` and `netif/FILES` |
| `libraries/MayaW476/src/wifi_nxp/`, `src/osa/`, `src/maya_w4_board/wifi_bt_module_config.h`, `src/maya_w4_board/wlan_txpwrlimit_cfg_murata_NH.h` | NXP IW610 WiFi and BLE driver | v1.3.r49.p2 | BSD-3-Clause per each file's SPDX line, with the exceptions below | `wlcmgr/wlan.c`, `wlcmgr/wlan_txpwrlimit_cfg.c`, `wifidriver/wifi.c`, `wifidriver/mlan_api.c`, `wifidriver/mlan_cfp.c`, `wifidriver/mlan_glue.c`, `wifidriver/incl/mlan_wmm.h`, `incl/wm_utils.h`, `maya_w4_board/wlan_txpwrlimit_cfg_murata_NH.h`, `wifi_bt_firmware/iw610x/iw610_cpu12_se.c`, `firmware_dnld/firmware_dnld.c`, `dhcpd/dhcp-server.c`, `incl/dhcp-server.h`, `port/net/net.c`. `incl/wifidriver/wifi.h` is renamed `wifi_driver.h`, and its include is changed in the 16 driver files that use it and in `maya_w4_board/wlan_txpwrlimit_cfg_murata_NH.h`. The six files in `src/osa/stubs/` are Ohm Lab files |
| `libraries/MayaW476/firmware/sduartspi_iw610.bin.se` | NXP IW610 firmware, from NXP `wifi_nb_fw` commit `7d80b36c846d7948acc19fff17d9bfcc3e8acd53` | 18.99.8.p10 | NXP proprietary, see `firmware/LA_OPT_NXP_Software_License.txt`. NXP's Software Content Register for that commit grants the Section 2.3 distribution licence for this version, see `firmware/sduartspi_iw610.bin.se.license` | None |
| `libraries/VD66GY/extras/st-driver/` | ST camera middleware, VD66GY sensor driver and ISP middleware | stm32-mw-camera v1.5.1, ISP library 2.0.1 | SLA0044. BSD-3-Clause for `sensors/vd6g/` | `isp/flat/vd66gy_mai1_isp_param_conf.h`, `sensors/vd6g/vd6g.h`. `isp/flat/isp_conf.h` is an Ohm Lab file |
| `libraries/VD66GY/reference/` | ST IQTune configuration headers | | SLA0044 | None |
| `libraries/LSM6DSL/extras/st-driver/` | ST LSM6DSL component, not compiled | | BSD-3-Clause | None |
| `libraries/AprilTag/src/n6_tag36h11_codes.h` | AprilTag tag36h11 code table | | BSD-2-Clause | None |
| `libraries/OV5640/src/ov5640.c`, `ov5640.h`, `ov5640_reg.c`, `ov5640_reg.h` | ST OV5640 component | | BSD-3-Clause, see `libraries/OV5640/LICENSE.txt` | `ov5640.c`, `ov5640.h` |
| `libraries/OV5640/src/ov5640_af_firmware.h` | OmniVision OV5640 autofocus firmware, copied from 0015/ESP32-OV5640-AF. ArduCAM's library holds the same bytes | | Copyright OmniVision Technologies. OmniVision states no licence, and the MIT licence does not cover it | None |
| `libraries/Microphone/src/stm32n6xx_hal_mdf.c` | STM32 HAL MDF driver | 1.2.0 | BSD-3-Clause, see `libraries/Microphone/LICENSE.txt` | None |
| `libraries/Speaker/src/stm32n6xx_hal_sai.c`, `stm32n6xx_hal_sai_ex.c` | STM32 HAL SAI driver | 1.2.0 | BSD-3-Clause, see `libraries/Speaker/LICENSE.txt` | None |
| `libraries/Microphone/src/neuron6_audio.c`, `libraries/Speaker/src/neuron6_speaker.c`: the GPDMA node set-up | ST's STM32N6570-DK BSP audio driver, `stm32n6570_discovery_audio.c` | | BSD-3-Clause, see each library's `LICENSE.txt` | Adapted. The rest of both files is Ohm Lab code |
| `libraries/EthernetN6/src/stm32n6xx_hal_eth.c`, `stm32n6xx_hal_eth_ex.c` and their headers | STM32 HAL Ethernet driver | 1.2.0 | BSD-3-Clause, see `libraries/EthernetN6/LICENSE.txt` | `stm32n6xx_hal_eth.c`, `stm32n6xx_hal_eth_ex.c` |
| `libraries/Barcode1D/src/barcode1d.c`: `pattern_variance()` and its two limits | ZXing's `patternMatchVariance`, the ZXing authors | | Apache-2.0, see `models/LICENSE-Apache-2.0.txt` | Ported to C. The rest of Barcode1D is Ohm Lab code |
| `libraries/PostProcess/src/app_postprocess_iseg_yolo_v8_ui.c`, `app_postprocess_mpe_yolo_v8_ui.c` | ST ai-postprocessing-wrapper, from the STM32N6 getting-started packages. ST ships it with an SLA0044 `LICENSE.md` | | SLA0044, see `libraries/PostProcess/LICENSE.md` | Both, rewritten. The rest of `PostProcess` is Ohm Lab code |
| `libraries/PostProcess/src/Fonts/` | ST utility fonts | | BSD-3-Clause, see `Fonts/LICENSE.txt` | None |
| `libraries/N6AudioAI/src/audio_din.*`, `audio_prePost_process.*`, `common_tables.*`, `dct.*`, `feature_extraction.*`, `mel_filterbank.*`, `window.*` | ST audio pre-processing and post-processing | | BSD-3-Clause, see `LICENSE.st-audio-preproc` | Comments only, in `audio_prePost_process.c`, `mel_filterbank.c`, `window.c` and `feature_extraction.h` |
| `libraries/NeuroN6Examples/examples/03.NeuralNetwork/ImageClassification/imagenet_labels.h`, `libraries/VD66GY/examples/GlobalShutterClassification/imagenet_labels.h` | ImageNet-1000 class names from Keras's `imagenet_class_index.json` | | Apache-2.0 | Generated into a C header |

Exceptions under `libraries/MayaW476/src/wifi_nxp/`:

| Files | Licence |
|---|---|
| `port/osa/osa_freertos.c`, `incl/port/osa/slist.h`, `incl/port/osa/stack_simple.h` | MIT |
| `port/osa/slist.c`, `port/osa/stack_simple.c` | MIT, from Michael Becker's freertos-addons, and BSD-3-Clause for NXP's part |
| `../maya_w4_board/wlan_txpwrlimit_cfg_murata_NH.h` | MIT |
| `nw_utils/wifi_ping.c`, Intel's part of `incl/port/net/wm_net.h` | Apache-2.0 |
| `port/net/wifi_netif.c` | lwIP's BSD-3-Clause |
| `wmcrypto/aescrypto/` | BSD-3-Clause, Jouni Malinen's hostap |

`libraries/VD66GY/extras/st-driver/isp/` holds the ISP middleware twice.
`flat/` is the copy the build compiles. `Inc/` and `Src/` are ST's layout.

## Tools

| Path | Component | Version | Licence | Files Ohm Lab changed |
|---|---|---|---|---|
| `tools/pyserial/serial/` | pyserial, Chris Liechti | 3.5 | BSD-3-Clause | None |

## Documents

| Path | Component | Version | Licence | Files Ohm Lab changed |
|---|---|---|---|---|
| `CODE_OF_CONDUCT.md` | Contributor Covenant, contributor-covenant.org | 2.1 | CC-BY-4.0 | The enforcement contact |
