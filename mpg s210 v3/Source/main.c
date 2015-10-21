 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"

#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_error.h"
#include "app_error.h"
#include "appconfig.h"
#include "boardconfig.h"
#include "command.h"
#include "global.h"
#include "serial.h"
#include "system.h"


uint32_t ulHardfaultProgramCounter;
uint32_t ulHardfaultLinkRegister;
uint32_t ulErrorCode;
uint8_t ucEventChannel;
uint8_t ucEventType;
bool bAllowSleep = 0;
bool bAllowSerialSleep = 0;
ANT_MESSAGE *pstRxMessage;
ANT_MESSAGE *pstTxMessage;

/**
 * @brief Handler for application asserts
 */
void assert_nrf_callback(uint16_t usLineNum, const uint8_t *pucFileName)
{
   // Copying parameters to static variables because parameters are not accessible in debugger
   static volatile uint8_t  ucDebugFileName[32];
   static volatile uint16_t usDebugLineNum;

   strcpy((char *)ucDebugFileName, (const char *)pucFileName);
   usDebugLineNum = usLineNum;
   UNUSED_VARIABLE(ucDebugFileName);
   UNUSED_VARIABLE(usDebugLineNum);

#if defined (RESET_ON_ASSERT_AND_FAULTS)
   NVIC_SystemReset();
#else
    while(1); // loop for debugging
#endif // RESET_ON_ASSERT_AND_FAULTS
}

/**
 * @brief Handler for softdevice asserts
 */
void softdevice_assert_callback(uint32_t ulPC, uint16_t usLineNum, const uint8_t *pucFileName)
{
   UNUSED_PARAMETER(ulPC);
   assert_nrf_callback(usLineNum, pucFileName);
}

/**
 * @brief Application error handler function
 */
void app_error_handler(uint32_t ulErrorCode, uint32_t ulLineNum, const uint8_t * pucFileName)
{
   // Copying parameters to static variables because parameters are not accessible in debugger.
   static volatile uint8_t  ucDebugFileName[32];
   static volatile uint16_t usDebugLineNum;
   static volatile uint32_t ulDebugErrorCode;

   strcpy((char *)ucDebugFileName, (const char *)pucFileName);
   usDebugLineNum   = ulLineNum;
   ulDebugErrorCode = ulErrorCode;
   UNUSED_VARIABLE(ucDebugFileName);
   UNUSED_VARIABLE(usDebugLineNum);
   UNUSED_VARIABLE(ulDebugErrorCode);

#if defined (RESET_ON_ASSERT_AND_FAULTS)
   NVIC_SystemReset();
#else
    while(1); // loop for debugging
#endif // RESET_ON_ASSERT_AND_FAULTS
}

/**
 * @brief Handler for hard faults
 */
void HardFault_Handler(uint32_t ulProgramCounter, uint32_t ulLinkRegister)
{
   (void)ulProgramCounter;
   (void)ulLinkRegister;

#if defined (RESET_ON_ASSERT_AND_FAULTS)
   NVIC_SystemReset();
#else
    while(1); // loop for debugging
#endif // RESET_ON_ASSERT_AND_FAULTS
}

/**
 * @brief Handler for SWI0 interrupts
 */
void SWI0_IRQHandler(void)
{
   // unused
}

/**
 * @brief Handler for radio notification interrupts (SWI1)
 */
void RADIO_NOTIFICATION_IRQHandler(void)
{
   // unused
}

/**
 * @brief Handler for protocol events & SOC event interrupts (SWI2)
 */
void SD_EVT_IRQHandler(void)
{
   uint32_t ulEvent;

   while (sd_evt_get(&ulEvent) != NRF_ERROR_NOT_FOUND) // read out SOC events
   {
     /* Don't understand why nothing is done with the event code */
   }

   bEventANTProcessStart = 1; // start ANT event handler to check if there are any ANT events
}

/**
 * @brief Handler for UART0 interrupts
 */
void UART0_IRQHandler(void)
{
   Serial_UART0_IRQHandler();
}

/**
 * @brief Handler for GPIOTE interrupts
 */
void GPIOTE_IRQHandler(void)
{
   Serial_GPIOTE_IRQHandler();
}

/**
 * @brief Handler for RTC1 interrupts
 */
void RTC1_IRQHandler(void)
{
   uint32_t ulCurrentTime;

#if defined (APP_RTC_TIMER)
   APP_NRF_RTC->TASKS_STOP = 1;
   ulCurrentTime = APP_NRF_RTC->COUNTER;
   APP_NRF_RTC->TASKS_CLEAR = 1; // reset counter back to 0
   nrf_delay_us(47); // delay for RTC timer task execution

   // update RTC handlers
   Serial_RTC_IRQHandler(ulCurrentTime); // serial interface RTC usage

   APP_NRF_RTC->TASKS_START = 1;
#endif // APP_RTC_TIMER
}

/**
 * @brief Set application queued burst mode
 */
void Main_SetQueuedBurst(void)
{
   bEventBurstMessageProcess = 1; // ANT burst message to process
}

/**
 * @brief Main
 */


int main()
{

   /*** scatter file loading done by sd_softdevice_enable must be done first before any RAM access ***/
#if defined (TEST_32K_RC)
   ulErrorCode = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, softdevice_assert_callback);
   APP_ERROR_CHECK(ulErrorCode);
#elif defined (TEST_32K_SYNTH)
   ulErrorCode = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, softdevice_assert_callback);
   APP_ERROR_CHECK(ulErrorCode);
#else
   ulErrorCode = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_50_PPM, softdevice_assert_callback);
   APP_ERROR_CHECK(ulErrorCode);
#endif

   bEventRXSerialMessageProcess = 0;
   bEventANTProcessStart = 0;
   bEventANTProcess = 0;
   bEventBurstMessageProcess = 0;
   bEventStartupMessage = 0;
   bQueuedTxBurst = 0;
   ucBurstSequence = 0;

   System_Init();

   Serial_Init();
   pstRxMessage = Serial_GetRxMesgPtr();
   pstTxMessage = Serial_GetTxMesgPtr();
#if defined (SERIAL_REPORT_RESET_MESSAGE)
   bEventStartupMessage = 1;
   System_ResetMesg((ANT_MESSAGE *)pstTxMessage); // send reset message upon system startup
#endif // !SERIAL_REPORT_RESET_MESSAGE

   ulErrorCode = sd_power_reset_reason_clr(POWER_RESETREAS_OFF_Msk | POWER_RESETREAS_LOCKUP_Msk | POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_RESETPIN_Msk); // clear reset reasons
   APP_ERROR_CHECK(ulErrorCode);

   // note: IRQ priorities must be set correctly before enabling them!
   ulErrorCode = sd_nvic_SetPriority(SWI0_IRQn, NRF_APP_PRIORITY_LOW);
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_SetPriority(RADIO_NOTIFICATION_IRQn, NRF_APP_PRIORITY_HIGH); // SW1_IRQn
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_SetPriority(SD_EVT_IRQn, NRF_APP_PRIORITY_LOW); // SW2_IRQn
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_EnableIRQ(SWI0_IRQn);
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_EnableIRQ(RADIO_NOTIFICATION_IRQn); // SW1_IRQn
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_EnableIRQ(SD_EVT_IRQn); // SW2_IRQn
   APP_ERROR_CHECK(ulErrorCode);

   // loop forever

   while (1)
   {
      bAllowSleep = 1;

      bAllowSerialSleep = Serial_RxMessage(); // poll for receive, check if serial interface can sleep

      if (bEventStartupMessage) // send out startup serial message
      {
         bEventStartupMessage = 0;

         bAllowSleep = 0;
      }
      else if (bEventRXSerialMessageProcess) // rx serial message to handle
      {
         bEventRXSerialMessageProcess = 0; // clear the RX event flag
         Command_SerialMessageProcess((ANT_MESSAGE *)pstRxMessage, (ANT_MESSAGE *)pstTxMessage); // send to command handler

         bAllowSleep = 0;
      }
      else if (bEventANTProcessStart || bEventANTProcess) // protocol event message to handle
      {
         if (!bEventANTProcess)
         {
            bEventANTProcessStart = 0;
            bEventANTProcess = 1;
         }

         ulErrorCode = sd_ant_event_get(&ucEventChannel, &ucEventType, (uint8_t*)pstTxMessage); // read event from ANT stack

         if ((ulErrorCode == NRF_SUCCESS) && ucEventType) // received event, send to event handlers
         {
            Serial_ANTEventHandler(ucEventType, pstTxMessage);
         }
         else // no event
         {
            bEventANTProcess = 0;
         }

         bAllowSleep = 0;
      }
      else if (bEventBurstMessageProcess) // we have a burst message to process
      {
         bEventBurstMessageProcess = 0; // clear the burst event flag
         if (Command_BurstMessageProcess((ANT_MESSAGE *)pstRxMessage, (ANT_MESSAGE *)pstTxMessage)) // try to process the burst transfer message
         {
            bQueuedTxBurst = 1; // indicate queued burst transfer process
         }

         bAllowSleep = 0;
      }

      Serial_TxMessage(); // transmit any pending tx messages, goes to sleep if it can

      if (bAllowSleep) // if sleep is allowed
      {
         if (bAllowSerialSleep)
            Serial_Sleep(); // serial interface sleep

         System_DeepSleep(); // goto deep sleep/system off if required

         (void)sd_app_evt_wait(); // wait for event low power mode

         Serial_CheckAsyncSuspend(); // determine if we woke up due to the suspend line being asserted, and if so go to deep sleep
      }
   }
}
