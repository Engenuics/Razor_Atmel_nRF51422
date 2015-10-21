 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>
#include <stdint.h>
#include "ant_parameters.h"
#include "appconfig.h"

/*
 * UICR reserved for Customer Block
 */
#define SYSTEM_UICR_CUST_WORDS_SIZE          32
#define SYSTEM_UICR_CUST_MFG_ESN_OFFSET      1
#define SYSTEM_UICR_CUST_BIST_OFFSET         0

typedef struct
{
   uint32_t USER_CFG[SYSTEM_UICR_CUST_WORDS_SIZE];
} NRF_UICR_Custom_Struct;
#define SYSTEM_UICR_CUST_STRUCT            ((NRF_UICR_Custom_Struct*) (NRF_UICR_BASE + 0x80)) //offset 0x080 nrF51 Reserved for Customer


/**
 * @brief Application system level initialization
 */
void System_Init(void);

/**
 * @brief Application system reset message
 */
void System_ResetMesg(ANT_MESSAGE *pstTxMessage);

/**
 * @brief Application system reset handler
 */
uint8_t System_Reset(uint8_t ucResetCmd);

/**
 * @brief Application deep sleep configuration handler
 */
uint8_t System_SetDeepSleep(ANT_MESSAGE *pstRxMessage); //counter 30.517 uS Resolution.

/**
 * @brief Used for asynchronous serial suspend function
 */
void System_SetSuspendSleep(void);

/**
 * @brief Application deep sleep handler
 */
void System_DeepSleep(void);


#if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
/**
 * @brief Constructs serial number message and calls function to retrieve serial number.
 */
void System_GetSerialNumberMesg(ANT_MESSAGE *pstTxMessage);
/**
 * @brief Retrieves lower 32-bits of 64-bit Nordic device ID stored in FICR. Returns ESN.
 */
void System_GetSerialNumber(uint8_t *pucESN);
#endif // !SERIAL_NUMBER_NOT_AVAILABLE

#endif // SYSTEM_H
