/**
 ******************************************************************************
 * @file    PostProcessor_Classifier.c
 * @brief   Image classifier post-processor.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "PostProcessor_Zoo.h"
#include "zoo_pp.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Inference timestamps: ts[0] before the NPU pass, ts[1] after. */
extern uint32_t ts[];

#define ZOO_CLS_MAX_TOP   8
#define ZOO_CLS_NAME_MAX  48
#define ZOO_CLS_LCD_LINES 3

typedef struct {
  zoo_common_t c;
  int32_t      top_k;
  int32_t      k;                        /* entries that reached conf */
  int32_t      idx[ZOO_CLS_MAX_TOP];
  float32_t    score[ZOO_CLS_MAX_TOP];
  char         line[ZOO_CLS_LCD_LINES][64];
} zoo_cls_state_t;

static zoo_cls_state_t g_cls = {
  .c = { .magic = ZOO_PP_MAGIC, .name = "Classifier", .conf = 0.2f,
         .slot = VND_META_SLOT_MODEL_0, .status = ZOO_PP_OK },
  .top_k = 5,
};

static int32_t cls_init(void *ctx, NN_Instance_TypeDef *model)
{
  ((zoo_cls_state_t *)ctx)->k = 0;
  return zoo_det_init(ctx, model);
}

static int32_t cls_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_cls_state_t *s = (zoo_cls_state_t *)ctx;
  zoo_common_t *c = &s->c;
  s->k = 0;

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *o = NULL;
  for (int32_t i = 0; i < nt; i++) {
    if (o == NULL || t[i].count > o->count) o = &t[i];
  }
  if (o == NULL || o->count < 2u) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected one output of class scores");
  if (o->count > 65535u) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "more than 65535 classes");
  const uint32_t n = o->count;

  /* A range outside 0..1 means raw scores, which go through a softmax. */
  float32_t lo, hi;
  if (o->type == DataType_FLOAT) {
    lo = hi = zoo_value(o, 0);
    for (uint32_t i = 1; i < n; i++) {
      const float32_t v = zoo_value(o, i);
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
  } else {
    const int32_t qmin = (o->type == DataType_INT8) ? -128 : 0;
    lo = o->scale * (float32_t)(qmin - o->zp);
    hi = o->scale * (float32_t)(qmin + 255 - o->zp);
  }
  const bool probs = (lo >= -0.01f && hi <= 1.01f);

  const int32_t kmax = (s->top_k < 1) ? 1 : ((s->top_k > ZOO_CLS_MAX_TOP) ? ZOO_CLS_MAX_TOP : s->top_k);
  int32_t idx[ZOO_CLS_MAX_TOP];
  float32_t val[ZOO_CLS_MAX_TOP];
  int32_t m = 0;
  for (uint32_t i = 0; i < n; i++) {
    const float32_t v = zoo_value(o, i);
    if (m == kmax && v <= val[m - 1]) continue;
    int32_t j = (m < kmax) ? m++ : (kmax - 1);
    while (j > 0 && val[j - 1] < v) {
      val[j] = val[j - 1];
      idx[j] = idx[j - 1];
      j--;
    }
    val[j] = v;
    idx[j] = (int32_t)i;
  }

  float32_t sum = 1.0f;
  if (!probs) {
    sum = 0.0f;
    for (uint32_t i = 0; i < n; i++) sum += expf(zoo_value(o, i) - val[0]);
  }
  for (int32_t j = 0; j < m; j++) {
    const float32_t p = probs ? val[j] : expf(val[j] - val[0]) / sum;
    if (p < c->conf) break;
    s->idx[s->k]   = idx[j];
    s->score[s->k] = p;
    s->k++;
  }

  c->nb_classes = (int32_t)n;
  return zoo_ok(c);
}

static const char *cls_name(const zoo_cls_state_t *s, int32_t i)
{
  return zoo_label(s->c.labels, s->c.nlabels, s->c.nb_classes, s->idx[i]);
}

static void cls_draw(void *ctx, OVL_Mode_t mode)
{
  zoo_cls_state_t *s = (zoo_cls_state_t *)ctx;
  const char *lines[ZOO_CLS_LCD_LINES];
  int32_t nl = 0;
  if (s->k == 0) {
    snprintf(s->line[0], sizeof(s->line[0]), "nothing above %d%%",
             (int)(s->c.conf * 100.0f + 0.5f));
    lines[nl++] = s->line[0];
  }
  for (int32_t i = 0; i < s->k && nl < ZOO_CLS_LCD_LINES; i++) {
    snprintf(s->line[nl], sizeof(s->line[nl]), "%.40s %d%%", cls_name(s, i),
             (int)(s->score[i] * 100.0f + 0.5f));
    lines[nl] = s->line[nl];
    nl++;
  }
  zoo_lcd_text_lines(lines, nl, mode);
}

static void cls_emit(void *ctx)
{
  const zoo_cls_state_t *s = (const zoo_cls_state_t *)ctx;
  uint8_t v[6 + ZOO_CLS_MAX_TOP * (5 + ZOO_CLS_NAME_MAX)];
  const int32_t nclasses = (s->c.nb_classes > 65535) ? 65535 : s->c.nb_classes;
  float32_t thr = s->c.conf;
  if (thr < 0.f) thr = 0.f;
  if (thr > 1.f) thr = 1.f;

  uint32_t len = 6u;
  v[0] = (uint8_t)s->k;
  v[1] = 0u;
  vnd_meta_put_u16_le(&v[2], (uint16_t)nclasses);
  vnd_meta_put_u16_le(&v[4], (uint16_t)(thr * 1000.0f + 0.5f));
  for (int32_t i = 0; i < s->k; i++) {
    const char *name = cls_name(s, i);
    uint32_t nl = (uint32_t)strlen(name);
    if (nl > ZOO_CLS_NAME_MAX) nl = ZOO_CLS_NAME_MAX;
    float32_t p = s->score[i];
    if (p > 1.f) p = 1.f;
    vnd_meta_put_u16_le(&v[len], (uint16_t)s->idx[i]);
    vnd_meta_put_u16_le(&v[len + 2u], (uint16_t)(p * 1000.0f + 0.5f));
    v[len + 4u] = (uint8_t)nl;
    memcpy(&v[len + 5u], name, nl);
    len += 5u + nl;
  }

  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, s->c.slot);
  if (s->c.slot == VND_META_SLOT_MODEL_0) {
    const uint32_t ms = ts[1] - ts[0];
    const uint16_t ms16 = (ms > 65535u) ? 65535u : (uint16_t)ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &ms16, sizeof(ms16));
  }
  (void)vnd_meta_builder_put_tlv(&meta, META_IMAGE_CLASS, v, (uint16_t)len);
  vnd_meta_builder_finish(&meta);
}

static IPostProcessor g_cls_pp = {
  .init = cls_init, .run = cls_run, .draw_lcd = cls_draw,
  .emit_metadata = cls_emit, .ctx = &g_cls,
};

IPostProcessor *PostProcess_Classifier(float conf, const char *const *labels,
                                       int nlabels)
{
  return PostProcess_Classifier_Ex(conf, labels, nlabels, 5, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_Classifier_Ex(float conf, const char *const *labels,
                                          int nlabels, int top_k,
                                          vnd_meta_slot_t slot)
{
  g_cls.c.conf    = conf;
  g_cls.c.labels  = labels;
  g_cls.c.nlabels = (labels != NULL) ? nlabels : 0;
  g_cls.c.slot    = slot;
  g_cls.top_k     = (top_k < 1) ? 1 : ((top_k > ZOO_CLS_MAX_TOP) ? ZOO_CLS_MAX_TOP : top_k);
  return &g_cls_pp;
}

int PostProcess_ClassifierTop(const IPostProcessor *pp, float *score)
{
  if (pp != &g_cls_pp || g_cls.k == 0) {
    if (score != NULL) *score = 0.0f;
    return -1;
  }
  if (score != NULL) *score = g_cls.score[0];
  return (int)g_cls.idx[0];
}
