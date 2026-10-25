/**
  ******************************************************************************
  * @file    Secure/Inc/secure_nsc.h
  * @author  MCD Application Team
  * @brief   Header for secure non-secure callable APIs list
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SECURE_NSC_H
#define SECURE_NSC_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>



/* CMSE Non-Secure Entry macro -----------------------------------------------*/
#ifndef CMSE_NS_ENTRY
#  if defined(__ICCARM__)
#    define CMSE_NS_ENTRY __cmse_nonsecure_entry
#  else
#    define CMSE_NS_ENTRY __attribute__((cmse_nonsecure_entry))
#  endif
#endif

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Secure callback ID enumeration definition
  */
typedef enum
{
  SECURE_FAULT_CB_ID = 0x00U,  /*!< Secure fault callback ID */
  GTZC_ERROR_CB_ID   = 0x01U   /*!< GTZC error callback ID   */
} SECURE_CallbackIDTypeDef;



/* Exported constants --------------------------------------------------------*/


/* Exported macros -----------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void SECURE_RegisterCallback(SECURE_CallbackIDTypeDef CallbackId, void *func);



#endif /* SECURE_NSC_H */
