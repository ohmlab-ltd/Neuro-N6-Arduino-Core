/**
  ******************************************************************************
  * @file    stm32n6_sdio_host.c
  * @brief   STM32N6 SDIO host port for the wifi_nxp driver and the IW610.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#include "stm32n6_sdio_host.h"
#include "maya_w4_pinmap.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "stm32n6xx_ll_sdmmc.h"

#include <string.h>

/* CMD5 OCR bits */
#define IW610_OCR_VDD_32_33   (1U << 20U)   /* 3.2 to 3.3 V */
#define IW610_OCR_READY_C     (1U << 31U)   /* C: card ready */

/* The wifi_nxp sdio_drv_* contract, as in mlan_sdio_api.h. */
#define WM_SUCCESS  0
#define WM_FAIL     1

extern int  sdio_drv_init(void (*cd_int)(int));
extern void sdio_drv_deinit(void);
extern int  sdio_drv_creg_read (int addr, int fn, uint32_t *resp);
extern int  sdio_drv_creg_write(int addr, int fn, uint8_t data, uint32_t *resp);
extern int  sdio_drv_read (uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp);
extern int  sdio_drv_write(uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp);
extern bool sdio_get_enume_status(void);

extern void sdio_enable_interrupt(void);
extern void sdio_disable_interrupt(void);

/* wifi_nxp's card-interrupt entry point: posts WIFI_EVENT_SDIO to wake
 * wifi_core_task. */
extern void handle_cdint(int error);

static SDIO_HandleTypeDef *s_hsdio           = NULL;
static void              (*s_cd_int_cb)(int)  = NULL;
static volatile bool      s_enumerated       = false;
static SemaphoreHandle_t   s_sdio_mutex       = NULL;

/* DMA completion, given from the SDMMC1 interrupt and taken under
 * s_sdio_mutex. The status is 0, or negative on a data error. */
static SemaphoreHandle_t   s_sdio_dma_done    = NULL;
static volatile int32_t    s_sdio_dma_status  = 0;

/* The IDMA bypasses the D-cache. The invalidate after a receive works on
 * whole 32-byte lines, so a receive into a buffer that does not start and end
 * on a line boundary goes through s_sdio_bounce. */
#define SDIO_DCACHE_LINE   32U
#define SDIO_BOUNCE_SIZE   512U
static uint8_t s_sdio_bounce[SDIO_BOUNCE_SIZE] __attribute__((aligned(SDIO_DCACHE_LINE)));

/* Returns the HAL state to READY after a DMA error, which the HAL does only
 * on DATAEND. The caller holds s_sdio_mutex. */
static inline void STM32N6_SDIO_ResetAfterDmaError(void)
{
    if (s_hsdio == NULL) {
        return;
    }
    __HAL_SDIO_DISABLE_IT(s_hsdio, SDMMC_IT_DATAEND  | SDMMC_IT_DCRCFAIL |
                                   SDMMC_IT_DTIMEOUT | SDMMC_IT_TXUNDERR |
                                   SDMMC_IT_RXOVERR);
    __HAL_SDIO_CLEAR_FLAG(s_hsdio, SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
                                   SDMMC_FLAG_TXUNDERR | SDMMC_FLAG_RXOVERR |
                                   SDMMC_FLAG_DATAEND);
    s_hsdio->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
    if ((s_hsdio->Instance->DCTRL & SDMMC_DCTRL_SDIOEN) != 0U) {
        s_hsdio->Instance->DCTRL = SDMMC_DCTRL_SDIOEN;
    } else {
        s_hsdio->Instance->DCTRL = 0U;
    }
    __SDMMC_CMDTRANS_DISABLE(s_hsdio->Instance);
    /* Written directly: the HAL has no abort API. */
    s_hsdio->State = HAL_SDIO_STATE_READY;
    s_hsdio->Context = 0U;  /* SDIO_CONTEXT_NONE */
}

/**
 * IW610 function interrupt, in interrupt context. The chip holds DAT[1] until
 * a task reads its status registers, so SDIOIT is masked here and re-armed at
 * the end of the next bus transfer.
 */
static void STM32N6_SDIO_FunctionCallback(SDIO_HandleTypeDef *hsdio, uint32_t func)
{
    if (func != 1U) {
        return;
    }

    __HAL_SDIO_DISABLE_IT(hsdio, SDMMC_IT_SDIOIT);

    /* STA.SDIOIT is write-one-to-clear and the HAL leaves it set. */
    __HAL_SDIO_CLEAR_FLAG(hsdio, SDMMC_FLAG_SDIOIT);

    handle_cdint(0);

    /* wifi_nxp passes cd_int as NULL. */
    if (s_cd_int_cb != NULL) {
        s_cd_int_cb(0);
    }
}

/* DMA completion callbacks. They run in the SDMMC1 interrupt. */

void HAL_SDIO_TxCpltCallback(SDIO_HandleTypeDef *hsdio)
{
    (void)hsdio;
    BaseType_t hpw = pdFALSE;
    s_sdio_dma_status = 0;
    if (s_sdio_dma_done != NULL) {
        (void)xSemaphoreGiveFromISR(s_sdio_dma_done, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void HAL_SDIO_RxCpltCallback(SDIO_HandleTypeDef *hsdio)
{
    (void)hsdio;
    BaseType_t hpw = pdFALSE;
    s_sdio_dma_status = 0;
    if (s_sdio_dma_done != NULL) {
        (void)xSemaphoreGiveFromISR(s_sdio_dma_done, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void HAL_SDIO_ErrorCallback(SDIO_HandleTypeDef *hsdio)
{
    BaseType_t hpw = pdFALSE;
    /* The HAL leaves the data-error flags set. */
    __HAL_SDIO_CLEAR_FLAG(hsdio,
        SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
        SDMMC_FLAG_RXOVERR  | SDMMC_FLAG_TXUNDERR);
    s_sdio_dma_status = -1;
    if (s_sdio_dma_done != NULL) {
        (void)xSemaphoreGiveFromISR(s_sdio_dma_done, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

/* Called at the end of every task-context bus transfer. */
static inline void STM32N6_SDIO_RearmIoIt(void)
{
    if (s_hsdio != NULL) {
        /* The SDMMC1 handler writes MASK too. Its priority, 5, is masked by
         * the critical section. */
        taskENTER_CRITICAL();
        __HAL_SDIO_ENABLE_IT(s_hsdio, SDMMC_IT_SDIOIT);
        taskEXIT_CRITICAL();
    }
}

void STM32N6_SDIO_BindHandle(SDIO_HandleTypeDef *hsdio)
{
    s_hsdio = hsdio;
}

void STM32N6_SDIO_IRQHandler(void)
{
    if (s_hsdio != NULL) {
        HAL_SDIO_IRQHandler(s_hsdio);
    }
}

/* wifi_nxp sdio_drv_* API */

/**
 * Returns 1 on a response, or on CMDSENT when none is expected, 0 on timeout
 * and -1 on CCRCFAIL, which is expected for R4.
 */
static int sdio_wait_cmd_response(bool expect_response)
{
    uint32_t tick_start = HAL_GetTick();
    uint32_t sta;
    do {
        sta = SDMMC1->STA;
        if (expect_response) {
            if (sta & SDMMC_STA_CMDREND)  return 1;
            if (sta & SDMMC_STA_CCRCFAIL) return -1;
            if (sta & SDMMC_STA_CTIMEOUT) return 0;
        } else {
            if (sta & SDMMC_STA_CMDSENT) return 1;
        }
    } while ((HAL_GetTick() - tick_start) < 1000U);
    return 0;
}

/** CMD5, IO_SEND_OP_COND. Returns 1 on success, 0 on timeout. */
static int sdio_send_cmd5(uint32_t ocr_arg, uint32_t *r4_out)
{
    SDMMC1->ICR = 0xFFFFFFFFU;
    SDMMC1->ARG = ocr_arg;
    SDMMC1->CMD = (5U << SDMMC_CMD_CMDINDEX_Pos)
                | SDMMC_CMD_WAITRESP_0      /* short response */
                | SDMMC_CMD_CPSMEN;

    int rc = sdio_wait_cmd_response(true);
    if (rc == 0) return 0;

    *r4_out = SDMMC1->RESP1;
    SDMMC1->ICR = 0xFFFFFFFFU;
    return 1;
}

/**
 * Enumerates the IW610. HAL_SDIO_Init is not used: the IW610 does not accept
 * its CMD11 voltage switch.
 */
int sdio_drv_init(void (*cd_int)(int))
{
    if (s_hsdio == NULL) {
        return -WM_FAIL;
    }

    if (s_enumerated) {
        s_cd_int_cb = cd_int;
        return WM_SUCCESS;
    }

    if (s_sdio_mutex == NULL) {
        s_sdio_mutex = xSemaphoreCreateMutex();
        if (s_sdio_mutex == NULL) {
            return -WM_FAIL;
        }
    }

    if (s_sdio_dma_done == NULL) {
        s_sdio_dma_done = xSemaphoreCreateBinary();
        if (s_sdio_dma_done == NULL) {
            vSemaphoreDelete(s_sdio_mutex);
            s_sdio_mutex = NULL;
            return -WM_FAIL;
        }
    }

    s_hsdio->Instance = SDMMC1;
    HAL_SDIO_MspInit(s_hsdio);

    /* Identification clock, 1-bit bus. SDMMC_CK = kernel / (2 x CLKDIV):
     * 114.29 MHz / 286 = 399.6 kHz, inside the 400 kHz limit. */
    #define MAYA_W4_SDIO_INIT_CLKDIV   (0x8FU)
    SDMMC1->CLKCR = (MAYA_W4_SDIO_INIT_CLKDIV << SDMMC_CLKCR_CLKDIV_Pos);
    SDMMC1->POWER = SDMMC_POWER_PWRCTRL;
    HAL_Delay(20U);

    /* CMD0, go idle. It has no response. */
    SDMMC1->ICR = 0xFFFFFFFFU;
    SDMMC1->ARG = 0U;
    SDMMC1->CMD = (0U << SDMMC_CMD_CMDINDEX_Pos) | SDMMC_CMD_CPSMEN;
    (void)sdio_wait_cmd_response(false);
    SDMMC1->ICR = 0xFFFFFFFFU;

    /* CMD5 with argument 0: probe. */
    uint32_t r4 = 0U;
    if (sdio_send_cmd5(0U, &r4) != 1) {
        return -WM_FAIL;
    }
    uint8_t num_io_funcs = (uint8_t)((r4 >> 28) & 0x07U);
    if (num_io_funcs == 0U) {
        return -WM_FAIL;
    }

    /* CMD5 OCR vote: poll the C bit. */
    uint32_t poll = 1000U;
    while (poll-- > 0U) {
        if (sdio_send_cmd5(IW610_OCR_VDD_32_33, &r4) != 1) {
            return -WM_FAIL;
        }
        if ((r4 & IW610_OCR_READY_C) != 0U) {
            break;
        }
        HAL_Delay(1U);
    }
    if ((r4 & IW610_OCR_READY_C) == 0U) {
        return -WM_FAIL;
    }

    /* CMD3, get the relative card address. The SDIO specification allows
     * ILLEGAL_CMD for a short while after CMD0, so this retries. */
    uint16_t rca = 1U;
    uint32_t err;
    poll = 1000U;
    do {
        err = SDMMC_CmdSetRelAdd(SDMMC1, &rca);
        if (err == HAL_SDIO_ERROR_NONE) break;
        if (err != SDMMC_ERROR_ILLEGAL_CMD) {
            return -WM_FAIL;
        }
        HAL_Delay(1U);
    } while (poll-- > 0U);
    if (err != HAL_SDIO_ERROR_NONE) {
        return -WM_FAIL;
    }

    /* CMD7, select the card. CMD52 and CMD53 work from here. */
    if (SDMMC_CmdSelDesel(SDMMC1, ((uint32_t)rca) << 16U) != HAL_SDIO_ERROR_NONE) {
        return -WM_FAIL;
    }

    /* Set directly, so that the HAL_SDIO API works. */
    s_hsdio->State     = HAL_SDIO_STATE_READY;
    s_hsdio->ErrorCode = HAL_SDIO_ERROR_NONE;
    s_hsdio->Context   = SDIO_CONTEXT_NONE;

    if (HAL_SDIO_SetDataBusWidth(s_hsdio, HAL_SDIO_4_WIRES_MODE) != HAL_OK) {
        return -WM_FAIL;
    }

    /* Transfer clock. Do not change it without retesting emissions. */
    #define MAYA_W4_SDIO_XFER_CLKDIV   (0x07U)   /* kernel/14 = 8.163 MHz */
    {
        uint32_t clkcr = SDMMC1->CLKCR;
        clkcr &= ~SDMMC_CLKCR_CLKDIV_Msk;
        clkcr |= (MAYA_W4_SDIO_XFER_CLKDIV << SDMMC_CLKCR_CLKDIV_Pos);

        /* Required for writes: a FIFO underrun would put undefined bytes on
         * the bus under a valid CRC. */
        clkcr |= SDMMC_CLKCR_HWFC_EN;

        /* SDMMC_CLKCR_PWRSAV stays clear: the host samples the IW610's DAT1
         * interrupt only while SDMMC_CK runs. */

        SDMMC1->CLKCR = clkcr;
        HAL_Delay(5U);   /* let new clock stabilise */
    }

    s_cd_int_cb = cd_int;
    /* The HAL counts enabled card interrupts on the handle and HAL_SDIO_DeInit
     * keeps the count. At 2 its handler issues a CMD52 and calls a callback
     * slot that is empty, so the card's interrupt is never cleared. */
    s_hsdio->IOInterruptNbr = 0U;
    s_hsdio->IOFunctionMask = 0U;
    if (HAL_SDIO_RegisterIOFunctionCallback(s_hsdio,
                                            HAL_SDIO_FUNCTION_1,
                                            STM32N6_SDIO_FunctionCallback) != HAL_OK) {
        return -WM_FAIL;
    }
    if (HAL_SDIO_EnableIOFunction(s_hsdio, HAL_SDIO_FUNCTION_1) != HAL_OK) {
        return -WM_FAIL;
    }
    if (HAL_SDIO_EnableIOFunctionInterrupt(s_hsdio, HAL_SDIO_FUNCTION_1) != HAL_OK) {
        return -WM_FAIL;
    }

    /* Priority 5, not above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, as
     * the FromISR calls in the callbacks require. */
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    s_enumerated = true;
    return WM_SUCCESS;
}

void sdio_drv_deinit(void)
{
    if (s_hsdio != NULL && s_enumerated) {
        (void)HAL_SDIO_DeInit(s_hsdio);
    }
    s_enumerated = false;
    s_cd_int_cb  = NULL;
    if (s_sdio_mutex != NULL) {
        vSemaphoreDelete(s_sdio_mutex);
        s_sdio_mutex = NULL;
    }
    if (s_sdio_dma_done != NULL) {
        vSemaphoreDelete(s_sdio_dma_done);
        s_sdio_dma_done = NULL;
    }
}

/** CMD52 register read. Returns 1 on success and 0 otherwise. */
int sdio_drv_creg_read(int addr, int fn, uint32_t *resp)
{
    if (s_hsdio == NULL || resp == NULL) {
        return 0;
    }

    HAL_SDIO_DirectCmd_TypeDef cmd = {
        .Reg_Addr       = (uint32_t)addr,
        .ReadAfterWrite = HAL_SDIO_WRITE_ONLY,        /* unused for read */
        .IOFunctionNbr  = (uint8_t)fn,
    };

    uint8_t value = 0U;

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    HAL_StatusTypeDef hs = HAL_SDIO_ReadDirect(s_hsdio, &cmd, &value);

    (void)xSemaphoreGive(s_sdio_mutex);

    STM32N6_SDIO_RearmIoIt();

    if (hs != HAL_OK) {
        return 0;
    }
    *resp = (uint32_t)value;
    return 1;
}

/** CMD52 register write, with read-after-write. */
int sdio_drv_creg_write(int addr, int fn, uint8_t data, uint32_t *resp)
{
    if (s_hsdio == NULL || resp == NULL) {
        return 0;
    }

    HAL_SDIO_DirectCmd_TypeDef cmd = {
        .Reg_Addr       = (uint32_t)addr,
        .ReadAfterWrite = HAL_SDIO_READ_AFTER_WRITE,
        .IOFunctionNbr  = (uint8_t)fn,
    };

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    HAL_StatusTypeDef hs = HAL_SDIO_WriteDirect(s_hsdio, &cmd, data);

    (void)xSemaphoreGive(s_sdio_mutex);

    STM32N6_SDIO_RearmIoIt();

    if (hs != HAL_OK) {
        return 0;
    }
    *resp = (uint32_t)data;     /* the HAL does not return the echo */
    return 1;
}

/**
 * CMD53 read over DMA. buf must be DMA-reachable and 4-byte aligned. A buffer
 * larger than s_sdio_bounce must be whole cache lines.
 */
int sdio_drv_read(uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp)
{
    if (s_hsdio == NULL || buf == NULL || resp == NULL) {
        return 0;
    }

    HAL_SDIO_ExtendedCmd_TypeDef cmd = {
        .Reg_Addr      = addr,
        /* No address increment: the IW610 MEM_PORT is a FIFO. */
        .OpCode        = HAL_SDIO_OP_CODE_NO_INC,
        .Block_Mode    = (bcnt > 1U) ? HAL_SDIO_MODE_BLOCK : HAL_SDIO_MODE_BYTE,
        .IOFunctionNbr = fn,
    };

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    /* The HAL derives the block count from hsdio->block_size. It is set under
     * the mutex because the handle is shared. */
    s_hsdio->block_size = (uint16_t)bsize;

    const uint32_t len = bcnt * bsize;
    uint8_t *dst = buf;
    if (((((uint32_t)buf) | len) & (SDIO_DCACHE_LINE - 1U)) != 0U && len <= SDIO_BOUNCE_SIZE) {
        dst = s_sdio_bounce;
    }

    /* Drains a completion left by a transfer that timed out. */
    (void)xSemaphoreTake(s_sdio_dma_done, 0);
    s_sdio_dma_status = 0;

    HAL_StatusTypeDef hs = HAL_SDIO_ReadExtended_DMA(s_hsdio, &cmd, dst, len);

    if (hs == HAL_OK) {
        if (xSemaphoreTake(s_sdio_dma_done, pdMS_TO_TICKS(200)) != pdTRUE) {
            hs = HAL_TIMEOUT;
        } else if (s_sdio_dma_status != 0) {
            hs = HAL_ERROR;
        }
    }

    if (hs == HAL_OK) {
        SCB_InvalidateDCache_by_Addr((volatile void *)dst, (int32_t)len);
        if (dst != buf) {
            (void)memcpy(buf, dst, len);
        }
    } else {
        STM32N6_SDIO_ResetAfterDmaError();
    }

    (void)xSemaphoreGive(s_sdio_mutex);

    STM32N6_SDIO_RearmIoIt();

    *resp = 0U;
    return (hs == HAL_OK) ? 1 : 0;
}

/* CMD53 write over DMA. buf must be DMA-reachable and 4-byte aligned. */
int sdio_drv_write(uint32_t addr, uint32_t fn, uint32_t bcnt, uint32_t bsize, uint8_t *buf, uint32_t *resp)
{
    if (s_hsdio == NULL || buf == NULL || resp == NULL) {
        return 0;
    }

    HAL_SDIO_ExtendedCmd_TypeDef cmd = {
        .Reg_Addr      = addr,
        /* No address increment: MEM_PORT is a FIFO. */
        .OpCode        = HAL_SDIO_OP_CODE_NO_INC,
        .Block_Mode    = (bcnt > 1U) ? HAL_SDIO_MODE_BLOCK : HAL_SDIO_MODE_BYTE,
        .IOFunctionNbr = fn,
    };

    if (xSemaphoreTake(s_sdio_mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    s_hsdio->block_size = (uint16_t)bsize;

    (void)xSemaphoreTake(s_sdio_dma_done, 0);
    s_sdio_dma_status = 0;

    /* The IDMA bypasses the D-cache. */
    SCB_CleanDCache_by_Addr((uint32_t *)(void *)buf,
                            (int32_t)(bcnt * bsize));

    HAL_StatusTypeDef hs = HAL_SDIO_WriteExtended_DMA(s_hsdio, &cmd, buf,
                                                      bcnt * bsize);

    if (hs == HAL_OK) {
        if (xSemaphoreTake(s_sdio_dma_done, pdMS_TO_TICKS(200)) != pdTRUE) {
            hs = HAL_TIMEOUT;
        } else if (s_sdio_dma_status != 0) {
            hs = HAL_ERROR;
        }
    }

    if (hs != HAL_OK) {
        STM32N6_SDIO_ResetAfterDmaError();
    }

    (void)xSemaphoreGive(s_sdio_mutex);

    STM32N6_SDIO_RearmIoIt();

    *resp = 0U;
    return (hs == HAL_OK) ? 1 : 0;
}

bool sdio_get_enume_status(void)
{
    return s_enumerated;
}

/** Function 1 card interrupt, called by wifi_nxp. */
void sdio_enable_interrupt(void)
{
    if (s_hsdio != NULL && s_enumerated) {
        /* MASKR.SDIOIT is set directly: HAL_SDIO_EnableIOFunctionInterrupt
         * does not re-arm it once CCCR4 shows the function enabled, and it
         * issues a CMD52 on every call. */
        taskENTER_CRITICAL();
        __HAL_SDIO_ENABLE_IT(s_hsdio, SDMMC_IT_SDIOIT);
        taskEXIT_CRITICAL();
    }
}

void sdio_disable_interrupt(void)
{
    if (s_hsdio != NULL && s_enumerated) {
        /* The host mask only: the caller is in a critical section, where a
         * CMD52 cannot take s_sdio_mutex. */
        taskENTER_CRITICAL();
        __HAL_SDIO_DISABLE_IT(s_hsdio, SDMMC_IT_SDIOIT);
        taskEXIT_CRITICAL();
    }
}

/** Weak stand-in for FreeRTOS v11's vTaskListTasks, which the driver's info
 *  dump refers to. */
__attribute__((weak)) char *vTaskListTasks(char *pcWriteBuffer, size_t uxBufferLength)
{
    (void)uxBufferLength;
    if (pcWriteBuffer != NULL) { pcWriteBuffer[0] = '\0'; }
    return pcWriteBuffer;
}

