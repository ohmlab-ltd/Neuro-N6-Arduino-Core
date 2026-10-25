/*
 * ModelBench
 *
 * Breaks NPU inference timing into its two parts, with no camera and no video
 * stream. The output line carries three figures in milliseconds per inference:
 *
 *   A) per-frame   Init, epochs, Reset and DeInit on every inference
 *   B) init-once   Init once, then epochs and Reset per inference
 *   overhead       A minus B
 *
 * Hardware: Neuro N6.
 *
 * The Nia model this bench embeds is licensed under the Ohm Lab Model Licence
 * (models/LICENSE-OhmLab-Model.txt): a firmware image built from it may be run
 * only on Ohm Lab hardware.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include <NeuroN6_app.h>
#pragma neuron6 model="nia_1_int8.onnx" name=nia

NEURON6_DECLARE_MODEL(nia);

/* Blocks on the NPU IRQ between epochs. */
static inline void run_epochs(NN_Instance_TypeDef *inst) {
  LL_ATON_RT_RetValues_t r;
  do {
    r = LL_ATON_RT_RunEpochBlock(inst);
    if (r == LL_ATON_RT_WFE) {
      LL_ATON_OSAL_WFE();
    }
  } while (r != LL_ATON_RT_DONE);
}

void setup() {
  VisionConfig cfg;
  cfg.transport = TRANSPORT_USB;
  Vision.begin(cfg);

  delay(1000);
  NeuroN6_DbgPrint("[NIA-BENCH] inference timing breakdown (no camera/video)\r\n");

  /* Warm-up: one inference primes the runtime and the weight cache. */
  LL_ATON_RT_Init_Network(&NN_Instance_nia);
  run_epochs(&NN_Instance_nia);
  LL_ATON_RT_Reset_Network(&NN_Instance_nia);
  LL_ATON_RT_DeInit_Network(&NN_Instance_nia);
}

void loop() {
  const int N = 50;
  NN_Instance_TypeDef *inst = &NN_Instance_nia;

  /* A) per-frame */
  uint32_t tA = millis();
  for (int i = 0; i < N; i++) {
    LL_ATON_RT_Init_Network(inst);
    run_epochs(inst);
    LL_ATON_RT_Reset_Network(inst);
    LL_ATON_RT_DeInit_Network(inst);
  }
  uint32_t A = millis() - tA;

  /* B) init-once */
  LL_ATON_RT_Init_Network(inst);
  uint32_t tB = millis();
  for (int i = 0; i < N; i++) {
    run_epochs(inst);
    LL_ATON_RT_Reset_Network(inst);
  }
  uint32_t B = millis() - tB;
  LL_ATON_RT_DeInit_Network(inst);

  uint32_t a_cms = (A * 100u) / (uint32_t)N;   /* centi-ms per inference */
  uint32_t b_cms = (B * 100u) / (uint32_t)N;
  int32_t  o_cms = (int32_t)a_cms - (int32_t)b_cms;
  if (o_cms < 0) o_cms = 0;

  char m[208];
  snprintf(m, sizeof(m),
    "[NIA-BENCH] per-frame=%lu.%02lu ms/inf | init-once(run-only)=%lu.%02lu ms/inf | "
    "Init/DeInit overhead=%ld.%02ld ms/inf\r\n",
    (unsigned long)(a_cms / 100u), (unsigned long)(a_cms % 100u),
    (unsigned long)(b_cms / 100u), (unsigned long)(b_cms % 100u),
    (long)(o_cms / 100), (long)(o_cms % 100));
  NeuroN6_DbgPrint(m);

  delay(1000);
}
