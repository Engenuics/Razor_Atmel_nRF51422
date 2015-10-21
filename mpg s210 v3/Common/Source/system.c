 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#include "system.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_gpio.h"

#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_error.h"
#include "appconfig.h"
#include "boardconfig.h"
#include "dsi_utility.h"
#include "serial.h"

/**
 * @brief Application system level initialization
 */
void System_Init(void)
{
#if defined (APP_RTC_TIMER)
   uint32_t ulErrorCode;

   ulErrorCode = sd_nvic_DisableIRQ(APP_NRF_RTC_IRQN);
   if (ulErrorCode != NRF_SUCCESS)
   {
      ASSERT(false);
   }

   // Enable RTC event interrupt
   ulErrorCode = sd_nvic_ClearPendingIRQ(APP_NRF_RTC_IRQN);
   if (ulErrorCode != NRF_SUCCESS)
   {
      ASSERT(false);
   }
   ulErrorCode = sd_nvic_SetPriority(APP_NRF_RTC_IRQN, NRF_APP_PRIORITY_LOW);
   if (ulErrorCode != NRF_SUCCESS)
   {
      ASSERT(false);
   }
   ulErrorCode = sd_nvic_EnableIRQ(APP_NRF_RTC_IRQN);
   if (ulErrorCode != NRF_SUCCESS)
   {
      ASSERT(false);
   }

   APP_NRF_RTC->PRESCALER = APP_RTC_PRESCALER;

   // TODO: we should only start this when needed!
   APP_NRF_RTC->TASKS_CLEAR = 1;
   APP_NRF_RTC->TASKS_START = 1; // start RTC
#endif // APP_RTC_TIMER

   //Default pin settings.
   NRF_GPIO->OUT = PIN_OUT_INIT;
   NRF_GPIO->DIR = PIN_DIR_INIT;

   #ifdef MPG
   /* Pin initializations for status LEDs */
   NRF_GPIO->PIN_CNF[LED_RED_BIT] = P0_29_LED_RED_CNF;
   NRF_GPIO->PIN_CNF[LED_YLW_BIT] = P0_28_LED_YLW_CNF;
   NRF_GPIO->PIN_CNF[LED_GRN_BIT] = P0_27_LED_GRN_CNF;
   NRF_GPIO->PIN_CNF[LED_BLU_BIT] = P0_26_LED_BLU_CNF;
   
   /* Brute-force routine to show the LEDs before turning over to the application */
   NRF_GPIO->OUTSET = (P0_29_LED_RED | P0_28_LED_YLW | P0_27_LED_GRN | P0_26_LED_BLU);
   for(uint32_t i = 0; i < 1000000; i++);
   /* Leave the yellow LED on to indicate that ANT is not yet set up */
   NRF_GPIO->OUTCLR = (P0_29_LED_RED | P0_28_LED_YLW | P0_27_LED_GRN | P0_26_LED_BLU);

#endif /* MPG */

   SCB->SCR |= SCB_SCR_SEVONPEND_Msk; // allow wakeup from enabled events and all interrupts (including disabled)
}

/**
 * @brief Application system reset message
 */
void System_ResetMesg(ANT_MESSAGE *pstTxMessage)
{
   uint32_t ulErrorCode;
   uint32_t ulReason;
   uint8_t ucResetMesg = 0;  // Default power on reset

   ulErrorCode = sd_power_reset_reason_get((uint32_t*)&ulReason);
   if (ulErrorCode != NRF_SUCCESS)
   {
      ASSERT(false);
   }

   if ( ulReason & POWER_RESETREAS_LOCKUP_Msk) // it looks like nrf_nvic_SystemReset generates this instead of SREQ
      ucResetMesg |= RESET_CMD;
   if ( ulReason & POWER_RESETREAS_SREQ_Msk) // from System reset req
      ucResetMesg |= RESET_CMD;
   if ( ulReason & POWER_RESETREAS_DOG_Msk) // from watchdog
      ucResetMesg |= RESET_WDT;
   if ( ulReason & POWER_RESETREAS_RESETPIN_Msk) // from the reset pin
      ucResetMesg |= RESET_RST;

   pstTxMessage->ANT_MESSAGE_aucMesgData[0] = ucResetMesg;
   pstTxMessage->ANT_MESSAGE_ucSize         = MESG_STARTUP_MESG_SIZE;
   pstTxMessage->ANT_MESSAGE_ucMesgID       = MESG_STARTUP_MESG_ID;
}

/**
 * @brief Application system reset handler
 */
uint8_t System_Reset(uint8_t ucResetCmd)
{
   uint32_t ulErrorCode;

#if defined (COMPLETE_CHIP_SYSTEM_RESET)

   ulErrorCode = sd_power_reset_reason_clr( POWER_RESETREAS_OFF_Msk | POWER_RESETREAS_LOCKUP_Msk | POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_RESETPIN_Msk);
   if (ulErrorCode != NRF_SUCCESS)
   {
      ASSERT(false);
   }

   ulErrorCode  = sd_softdevice_disable();
   if (ulErrorCode != NRF_SUCCESS)
   {
      ASSERT(false);
   }

   NVIC_SystemReset(); // allowed after disabling softdevice

#else
   #error //unsupported reset scheme

#endif
   return NO_RESPONSE_MESSAGE;
}

/**
 * @brief Application deep sleep configuration handler
 */
static uint8_t ucDeepSleepFlag = 0;
uint8_t System_SetDeepSleep(ANT_MESSAGE *pstRxMessage)
{
   uint32_t ulSleep30u517;

   if(pstRxMessage->ANT_MESSAGE_ucSize == 0x05 || pstRxMessage->ANT_MESSAGE_ucSize == 0x06)
   {
      ulSleep30u517  = pstRxMessage->ANT_MESSAGE_aucPayload[0];
      ulSleep30u517 |= ((uint32_t)pstRxMessage->ANT_MESSAGE_aucPayload[1]) << 8;
      ulSleep30u517 |= ((uint32_t)pstRxMessage->ANT_MESSAGE_aucPayload[2]) << 16;
      ulSleep30u517 |= ((uint32_t)pstRxMessage->ANT_MESSAGE_aucPayload[3]) << 24;

      if (pstRxMessage->ANT_MESSAGE_ucSize == 0x05)
      {
      #if defined (SERIAL_SLEEP_POLLING_MODE) && defined (SERIAL_SLEEP_WAKEUP_RTC)
         Serial_Snooze_Interval_Set(ulSleep30u517,0);
      #endif // SERIAL_SLEEP_POLLING_MODE
      }
      if (pstRxMessage->ANT_MESSAGE_aucPayload[4] <= 1)
      {
      #if defined (SERIAL_SLEEP_POLLING_MODE)  && defined (SERIAL_SLEEP_WAKEUP_RTC)
         Serial_Snooze_Interval_Set(ulSleep30u517, pstRxMessage->ANT_MESSAGE_aucPayload[4]);
      #endif // SERIAL_SLEEP_POLLING_MODE
      }
      else
      {
         return INVALID_MESSAGE;
      }
   }
   else if (pstRxMessage->ANT_MESSAGE_ucSize == 0x01)
   {
      ucDeepSleepFlag = 1;
   }
   else
   {
      return INVALID_MESSAGE;
   }

   return RESPONSE_NO_ERROR;
}

/**
 * @brief Used for asynchronous serial suspend function
 */
void System_SetSuspendSleep(void)
{
   ucDeepSleepFlag = 1;
}

/**
 * @brief Application deep sleep handler
 */
void System_DeepSleep(void)
{
   uint32_t ulErrorCode;

   if (ucDeepSleepFlag)
   {
   //OPTION 1
   #if 1
      ulErrorCode = sd_power_system_off(); // this should not return;
      if (ulErrorCode != NRF_SUCCESS)
      {
         ASSERT(false);
      }
      while(1); //reset is our only hope.
   #endif

   //OPTION 2
   #if 0
      ulErrorCode = sd_softdevice_disable();
      if (ulErrorCode != NRF_SUCCESS)
      {
         ASSERT(false);
      }
      NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
      while(1);
   #endif
   }
}


/**
 * @brief Retrieves 64-bit Nordic device ID stored in FICR.
 */
uint8_t System_FICRDeviceID(ANT_MESSAGE *pstTxMessage)
{
   // read lower 32-bits of permanent random device ID from FICR NVM
   uint32_t dev_id = NRF_FICR->DEVICEID[0];

   /* Allows us to read the permanent random device ID from FICR */
   //construct ANT Tx message
   pstTxMessage->ANT_MESSAGE_ucSize = 8;
   pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_DEBUG_ID; // called from debug message, therefore returns as debug message

   //construct device ID to little endian lower 32-bit number
   pstTxMessage->ANT_MESSAGE_aucMesgData[0] = dev_id >>  0 & 0x000000FF;
   pstTxMessage->ANT_MESSAGE_aucMesgData[1] = dev_id >>  8 & 0x000000FF;
   pstTxMessage->ANT_MESSAGE_aucMesgData[2] = dev_id >> 16 & 0x000000FF;
   pstTxMessage->ANT_MESSAGE_aucMesgData[3] = dev_id >> 24 & 0x000000FF;

   //retrieve upper 32-bits of permanent random device ID from FICR NVM
   dev_id = NRF_FICR->DEVICEID[1];

   //construct device ID to little endian upper 32-bits
   pstTxMessage->ANT_MESSAGE_aucMesgData[4] = dev_id >>  0 & 0x000000FF;
   pstTxMessage->ANT_MESSAGE_aucMesgData[5] = dev_id >>  8 & 0x000000FF;
   pstTxMessage->ANT_MESSAGE_aucMesgData[6] = dev_id >> 16 & 0x000000FF;
   pstTxMessage->ANT_MESSAGE_aucMesgData[7] = dev_id >> 24 & 0x000000FF;

   return RESPONSE_NO_ERROR;
}

#if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
/**
 * @brief Constructs serial number message and calls function to retrieve serial number.
 */
void System_GetSerialNumberMesg(ANT_MESSAGE *pstTxMessage)
{
   pstTxMessage->ANT_MESSAGE_ucSize   = MESG_GET_SERIAL_NUM_SIZE;
   pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_GET_SERIAL_NUM_ID;
   System_GetSerialNumber(pstTxMessage->ANT_MESSAGE_aucMesgData);
}

/**
 * @brief Retrieves lower 32-bits of 64-bit Nordic device ID stored in FICR. Returns ESN.
 */
void System_GetSerialNumber(uint8_t *pucESN)
{
   // read out permanent random device ID from FICR NVM
   uint32_t dev_id = NRF_FICR->DEVICEID[0];

   /* Allows us to read the permanent random device ID from FICR */
   //construct ESN to little endian 32-bit number
   pucESN[0] = dev_id >>  0 & 0x000000FF;
   pucESN[1] = dev_id >>  8 & 0x000000FF;
   pucESN[2] = dev_id >> 16 & 0x000000FF;
   pucESN[3] = dev_id >> 24 & 0x000000FF;
}
#endif // !SERIAL_NUMBER_NOT_AVAILABLE
