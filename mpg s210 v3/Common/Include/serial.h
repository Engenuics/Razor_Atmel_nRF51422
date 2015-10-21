 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stdint.h>
#include "ant_parameters.h"
#include "appconfig.h"

#define SERIAL_SLEEP_POLLING_MODE // enable serial sleeping mechanism
#if defined (SERIAL_SLEEP_POLLING_MODE)
 /*#define SERIAL_SLEEP_WAKEUP_RTC*/ // comment out if NOT using RTC to wakeup, ELSE use SENSE-DETEC SIGNAL.
#endif // SERIAL_SLEEP_POLLING_MODE

#define SERIAL_RX_BUFFER_SIZE        (MESG_MAX_DATA_SIZE + MESG_ID_SIZE)

/**
 * @brief Serial interface initialization
 */
void Serial_Init(void);

/**
 * @brief Get input message buffer
 */
ANT_MESSAGE *Serial_GetRxMesgPtr(void);

/**
 * @brief Get output message buffer
 */
ANT_MESSAGE *Serial_GetTxMesgPtr(void);

/**
 * @brief Hold incoming serial communication
 */
void Serial_HoldRx(void);

/**
 * @brief Allow incoming serial communication
 */
void Serial_ReleaseRx(void);

/**
 * @brief Send serial message
 */
void Serial_TxMessage(void);

/**
 * @brief Receive serial message
 */
bool Serial_RxMessage(void);

/**
 * @brief Serial interface sleep handler
 */
void Serial_Sleep(void);

/**
 * @brief Determine if the async suspend line is being asserted, and if so go to deep sleep
 */
void Serial_CheckAsyncSuspend(void);

#if defined (SERIAL_SLEEP_POLLING_MODE) && defined (SERIAL_SLEEP_WAKEUP_RTC)
/**
 * @brief Configure serial interface sleep wakeup interval
 */
void Serial_Snooze_Interval_Set(uint32_t ulSleep30u517, uint8_t ucOnceOnly);
#endif // SERIAL_SLEEP_POLLING_MODE && SERIAL_SLEEP_WAKEUP_RTC

/**
 * @brief Interrupt handler for synchronous serial SMSGRDY and SRDY interrupt. Uses GPIOTE 0 and 1
 */
#define SERIAL_SYNC_GPIOTE_EVENT_SMSGRDY  0  // assigned GPIOTE 0 for SMSGRDY
#define SERIAL_SYNC_GPIOTE_EVENT_SRDY     1  // assigned GPIOTE 1 for SRDY
void Serial_GPIOTE_IRQHandler(void);

/**
 * @brief Interrupt handler for asynchronous serial interface
 */
void Serial_UART0_IRQHandler(void);

/**
 * @brief Interrupt handler for RTC usage by serial interface
 */
void Serial_RTC_IRQHandler(uint32_t ulCurrentTime);

/**
 * @brief ANT event handler used by serial interface
 */
void Serial_ANTEventHandler(uint8_t ucEventType, ANT_MESSAGE *pstANTMessage);

#endif /* SERIAL_H_ */
