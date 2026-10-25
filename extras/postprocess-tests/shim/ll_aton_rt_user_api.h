/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Host stand-in for the ATON runtime's buffer descriptors. */
#ifndef LL_ATON_RT_USER_API_H
#define LL_ATON_RT_USER_API_H

#include <stdint.h>

typedef enum {
  DataType_UNDEFINED = 0,
  DataType_FLOAT = 1,
  DataType_UINT8 = 6,
  DataType_INT8 = 7,
  DataType_UINT16 = 8,
  DataType_INT16 = 9,
  DataType_INT32 = 10,
} Buffer_DataType_TypeDef;

typedef enum {
  CHPos_UNDEFINED = 0,
  CHPos_First = 1,
  CHPos_Last = 2,
  CHPos_Mixed = 3,
} Buffer_CHPos_TypeDef;

typedef struct {
  const char *name;
  unsigned char *addr_base;
  uint32_t offset_start;
  uint32_t offset_end;
  uint32_t offset_limit;
  uint8_t is_user_allocated;
  uint8_t is_param;
  uint16_t epoch;
  uint32_t batch;
  const uint32_t *mem_shape;
  uint16_t mem_ndims;
  Buffer_CHPos_TypeDef chpos;
  Buffer_DataType_TypeDef type;
  int8_t Qm;
  int8_t Qn;
  uint8_t Qunsigned;
  uint8_t ndims;
  uint8_t nbits;
  uint8_t per_channel;
  const uint32_t *shape;
  const float *scale;
  const int16_t *offset;
} LL_Buffer_InfoTypeDef;

typedef struct NN_Instance_TypeDef NN_Instance_TypeDef;

static inline unsigned char *LL_Buffer_addr_start(const LL_Buffer_InfoTypeDef *b)
{
  return b->addr_base + b->offset_start;
}
static inline unsigned char *LL_Buffer_addr_end(const LL_Buffer_InfoTypeDef *b)
{
  return b->addr_base + b->offset_end;
}
static inline uint32_t LL_Buffer_len(const LL_Buffer_InfoTypeDef *b)
{
  return b->offset_end - b->offset_start;
}

/* Defined by the test driver. */
const LL_Buffer_InfoTypeDef *LL_ATON_Output_Buffers_Info(const NN_Instance_TypeDef *n);
const LL_Buffer_InfoTypeDef *LL_ATON_Input_Buffers_Info(const NN_Instance_TypeDef *n);

#endif /* LL_ATON_RT_USER_API_H */
