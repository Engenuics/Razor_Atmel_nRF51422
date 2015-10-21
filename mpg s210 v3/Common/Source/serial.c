 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#include "serial.h"

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"

#include "ant_interface.h"
#include "ant_parameters.h"
#include "appconfig.h"
#include "boardconfig.h"
#include "global.h"
#include "serial.h"
#include "system.h"


/*NOTES:
 * This serial.c is specifically designed for nrf51-ANT Serial Async/Sync Implementation.
 * please refer to Interfacing with ANT General Purpose Chipsets and Modules Rev 2.1.pdf
 *
 * Few differences to the specification above.
 * 1. Synchronous bit flow controlled is not supported
 * 2. Asynchronous RTS is hardware flow controlled. RTS flow control is byte level instead of message level
 *
 * NOTE: PERIPHERALS USED
 * 1. SPI0  (for Synchronous Serial)
 * 2. UART0 (for Asynchronous Serial)
 * 3. RTC1  (for periodic low power wake-up poll)
 */

/*****************************************
 * SPI WORKAROUND
 * This temporary workaround was implemented
 * because the SPI mode 3 (Clock Idle Hi, Active low) was not available
 * on parts before nRF51822-QFAA build CA
 *****************************************/
  //#define SPI_IDLE_LOW_WORKAROUND         // uncomment SPI_IDLE_LOW_WORKAROUND to implement the workaround

/***************************************************************************
 * GENERIC REGISTERS
 ***************************************************************************/

#define NUMBER_OF_NRF_GPIO_PINS           (32)

/***************************************************************************
 * NRF SYNCHRONOUS SPI PERIPHERAL ACCESS DEFINITIONS
 ***************************************************************************/
#define SERIAL_SYNC                       NRF_SPI0 //using NRF SPI Master 0

#define SERIAL_SYNC_FREQUENCY             SPI_FREQUENCY_FREQUENCY_M4
#define SERIAL_SYNC_FREQUENCY_SLOW        SPI_FREQUENCY_FREQUENCY_K500
#define SERIAL_SYNC_SERIAL_ENABLE()       (SERIAL_SYNC->ENABLE |= (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos)) // SPI Peripheral Enable\Disable
#define SERIAL_SYNC_SERIAL_DISABLE()      (SERIAL_SYNC->ENABLE &= ~(SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos))
#define SYNC_SEN_ASSERT()                 NRF_GPIO->OUT &= ~(1UL << SERIAL_SYNC_PIN_SEN)
#define SYNC_SEN_DEASSERT()               NRF_GPIO->OUT |= (1UL << SERIAL_SYNC_PIN_SEN)
#define IS_MRDY_ASSERTED()                (!((NRF_GPIO->IN >> SERIAL_SYNC_PIN_SMSGRDY) & 1UL)) // true when asserted, is active low
#define IS_SRDY_ASSERTED()                (!((NRF_GPIO->IN >> SERIAL_SYNC_PIN_SRDY) & 1UL)) // true when asserted, is active low
#define IS_BIT_SYNC()                     0UL // temporarily defaulted to Byte Sync
#define SERIAL_PORT_SEL                   (NRF_GPIO->IN & (0x1UL << SERIAL_PIN_PORTSEL)) // reading serial PORTSEL PIN

#if defined (SYNCHRONOUS_DISABLE)
   #define IS_SYNC_MODE()                 false
#elif defined (ASYNCHRONOUS_DISABLE)
   #define IS_SYNC_MODE()                 true
#else
   #define IS_SYNC_MODE()                 (SERIAL_PORT_SEL) // sync is PORTSEL high
#endif //SYNC ASYNC

#define EVENT_PIN_SMSGRDY                 SERIAL_SYNC_GPIOTE_EVENT_SMSGRDY // GPIO Task and Event number for SMSGRDY Pin
#define EVENT_PIN_SRDY                    SERIAL_SYNC_GPIOTE_EVENT_SRDY // GPIO Task and Event number for SRDY Pin
#define IS_INT_PIN_SMSGRDY_ASSERTED()     (NRF_GPIOTE->EVENTS_IN[EVENT_PIN_SMSGRDY] && (NRF_GPIOTE->INTENSET & (1UL << EVENT_PIN_SMSGRDY)))
#define IS_INT_PIN_SRDY_ASSERTED()        (NRF_GPIOTE->EVENTS_IN[EVENT_PIN_SRDY] && (NRF_GPIOTE->INTENSET & (1UL << EVENT_PIN_SRDY)))

#define INT_PIN_SMSGRDY_ENABLE()          NRF_GPIOTE->INTENSET = 1UL << EVENT_PIN_SMSGRDY;
#define INT_PIN_SMSGRDY_DISABLE()         NRF_GPIOTE->INTENCLR = 1UL << EVENT_PIN_SMSGRDY;
#define INT_PIN_SMSGRDY_FLAG_CLEAR()      NRF_GPIOTE->EVENTS_IN[EVENT_PIN_SMSGRDY] = 0
#define INT_PIN_SRDY_ENABLE()             NRF_GPIOTE->INTENSET = 1UL << EVENT_PIN_SRDY;
#define INT_PIN_SRDY_DISABLE()            NRF_GPIOTE->INTENCLR = 1UL << EVENT_PIN_SRDY;
#define INT_PIN_SRDY_FLAG_CLEAR()         NRF_GPIOTE->EVENTS_IN[EVENT_PIN_SRDY] = 0

#define SYNC_SRDY_SLEEP_DELAY             20  // wait for a bit, srdy might be coming quick. per tick is about 1.37uS. Average interbyte space is ~25uS

/***************************************************************************
 * NRF ASYNCHRONOUS UART PERIPHERAL ACCESS DEFINITIONS
 ***************************************************************************/
#define SERIAL_ASYNC  NRF_UART0   //using NRF UART 0
/*** Interrupt Control ***/
#define SERIAL_ASYNC_INT_TXRDY_ENABLE()   SERIAL_ASYNC->INTENSET = (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) // enable Tx Ready Interrupt
#define SERIAL_ASYNC_INT_TXRDY_DISABLE()  SERIAL_ASYNC->INTENCLR = (UART_INTENCLR_TXDRDY_Clear << UART_INTENCLR_TXDRDY_Pos)// disable Rx Ready Interrupt
#define SERIAL_ASYNC_INT_RXRDY_ENABLE()   SERIAL_ASYNC->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) // enable Rx Ready Interrupt
#define SERIAL_ASYNC_INT_RXRDY_DISABLE()  SERIAL_ASYNC->INTENCLR = (UART_INTENCLR_RXDRDY_Clear << UART_INTENCLR_RXDRDY_Pos)// disable Rx Ready Interrupt
/*** Start and Stop Tx Task ***/
#define SERIAL_ASYNC_START_TX()           {SERIAL_ASYNC->TASKS_STARTTX = 1; bTransmitting = true;}
#define SERIAL_ASYNC_STOP_TX()            {SERIAL_ASYNC->TASKS_STOPTX = 1; bTransmitting = false;}
/*** RTS Line Control ***/
#define SERIAL_ASYNC_RTS_ENABLE()         SERIAL_ASYNC_INT_RXRDY_ENABLE() // RTS line is tied to hardware flow control,
#define SERIAL_ASYNC_RTS_DISABLE()        SERIAL_ASYNC_INT_RXRDY_DISABLE()
/*** UART Peripheral Enable\Disable ***/
#define SERIAL_ASYNC_SERIAL_ENABLE()      SERIAL_ASYNC->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos
#define SERIAL_ASYNC_SERIAL_DISABLE()     SERIAL_ASYNC->ENABLE = UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos
/*** Fixed Baudrate value ***/
#define SERIAL_ASYNC_BAUDRATE_DEFAULT_VALUE    UART_BAUDRATE_BAUDRATE_Baud57600

#define IS_PIN_SLEEP_ASSERTED()           (NRF_GPIO->IN & (0x1UL << SERIAL_ASYNC_PIN_SLEEP))
#define IS_PIN_SUSPEND_ASSERTED()         (!((NRF_GPIO->IN & (0x1UL << SERIAL_ASYNC_PIN_SUSPEND)))) // true when asserted, is active low

#if defined (SERIAL_SLEEP_POLLING_MODE) && defined (SERIAL_SLEEP_WAKEUP_RTC)
   #define SERIAL_SLEEP_POLLTIME_DOZE                    5 // number of doze wakeups.
   #define SERIAL_SLEEP_POLLTIME_DOZE_INTERVAL           32 // interval of doze wakeups (1ms @ 30.517us rtc counter resolution)
   #define SERIAL_SLEEP_POLLTIME_SNOOZE_INTERVAL_SHORT   98 // interval of short snooze wakeups (3ms @ 30.517us rtc counter resolution)
   #define SERIAL_SLEEP_POLLTIME_SNOOZE_INTERVAL_LONG    1639 // interval of long snooze wakeups (50ms @ 30.517us rtc counter resolution)
#endif // SERIAL_SLEEP_POLLING_MODE

/***************************************************************************
 * Local Variables
 ***************************************************************************/
/*
 * Serial control flags: Must be accessed/changed atomically as they can be accessed by multiple contexts
 * If bitfields are used, the entire bitfield operation must be atomic!!
 */
static volatile bool bStartMessage;
static volatile bool bEndMessage;
static volatile bool bTransmitting;
static volatile bool bHold;
static volatile bool bSyncMode;
static volatile bool bSleep;
static volatile bool bSRdy;

/**/
#define PIN_SENSE_DISABLED    0
#define PIN_SENSE_LOW         1
#define PIN_SENSE_HIGH        2
static uint8_t ucPinSenseSRDY = PIN_SENSE_DISABLED;
static uint8_t ucPinSenseMRDY = PIN_SENSE_DISABLED;

#if defined (SERIAL_SLEEP_POLLING_MODE) && defined (SERIAL_SLEEP_WAKEUP_RTC)
   static volatile uint32_t ulSnoozeIntervalLong = SERIAL_SLEEP_POLLTIME_SNOOZE_INTERVAL_LONG;  // default
   static volatile uint8_t ucDozeIntervalCount;
   static volatile bool bSnoozeIntervalBurstMode;
   static volatile bool bSnoozeIntervalLongVariableOnceOnly = 0; // the variable option will run only once and then will revert back to default (SERIAL_SLEEP_POLLTIME_SNOOZE_INTERVAL_LONG)
#endif // SERIAL_SLEEP_POLLING_MODE

/*
 * This table is a lookup for the baud rate control registers,
 * based on the pre-established baud rates we support.
 */

#if !defined (ASYNCHRONOUS_DISABLE)
#define BAUDRATE_DEFAULT_NDX        7
uint8_t ucBaudrateNdx = BAUDRATE_DEFAULT_NDX; // default Asynch baudrate 57600
static const uint32_t asBaudControl[8] =
{
   UART_BAUDRATE_BAUDRATE_Baud4800, // 4800  baud.
   UART_BAUDRATE_BAUDRATE_Baud38400, // 38400 baud
   UART_BAUDRATE_BAUDRATE_Baud19200, // 19200 baud.
   UART_BAUDRATE_BAUDRATE_Baud57600, // 57600 baud.  Spare value, may be updated later.
   UART_BAUDRATE_BAUDRATE_Baud1200, // 1200  baud.
   UART_BAUDRATE_BAUDRATE_Baud9600, // 9600  baud.
   UART_BAUDRATE_BAUDRATE_Baud2400, // 2400  baud.
   UART_BAUDRATE_BAUDRATE_Baud57600 // 57600 baud.
};
#endif

static volatile ANT_MESSAGE stTxMessage; // volatile required due to polling on size during transmit
static volatile ANT_MESSAGE stRxMessage;

static uint8_t ucRxPtr;
static uint8_t ucTxPtr;

#if defined (SERIAL_ASYNC_PIN_SLEEP)
static uint32_t ulPinSenseCfg;
static uint32_t ulPinSenseEnabled;
#endif //SERIAL_ASYNC_PIN_SLEEP

/***************************************************************************
 * Private Functions Protoype
 ***************************************************************************/

static void AsyncProc_TxMessage(void);
static void SyncProc_TxMessage(void);
static void Serial_Wakeup(void);

#if !defined (SYNCHRONOUS_DISABLE)
static void Gpiote_FallingEdge_Enable (void);
static void Gpiote_FallingEdge_Disable (void);
static bool Sync_IsMSGRDYAsserted(void);
static void SyncProc_RxMessage(void);
#endif //!SYNCHRONOUS_DISABLE

/***************************************************************************
 * Public Functions
 ***************************************************************************/
/**
 * @brief Serial interface initialization
 */
void Serial_Init (void)
{
#if defined (SERIAL_PIN_PORTSEL)
   // Set SERIAL_PIN_PORTSEL as input, This determines what type of Serial to use Async or Sync
   NRF_GPIO->PIN_CNF[SERIAL_PIN_PORTSEL] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                    (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                    (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                    (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
#endif

   bStartMessage = 0;
   bEndMessage = 0;
   bTransmitting = 0;
   bHold = 0;
   bSyncMode = 0;
   bSRdy = 0;
   bSleep = 0;


#if defined (SERIAL_SLEEP_POLLING_MODE) && defined (SERIAL_SLEEP_WAKEUP_RTC)
   ucDozeIntervalCount = SERIAL_SLEEP_POLLTIME_DOZE;
   bSnoozeIntervalBurstMode = false;
#endif // SERIAL_SLEEP_POLLING_MODE

#if !defined (SYNCHRONOUS_DISABLE)
   if (IS_SYNC_MODE()) // check which mode is desired
   {
      bSyncMode = true;

      /* Make sure peripheral is on reset, disable first*/
      /*****************************************************************************************************
       * Initializing SPI related registers
       */

      SERIAL_SYNC_SERIAL_DISABLE();
      sd_nvic_DisableIRQ(SPI0_TWI0_IRQn);
      sd_nvic_ClearPendingIRQ(SPI0_TWI0_IRQn);

      // SPI0_TWI0 PERPOWER enabled in system_nrf51 init

      /*** Configuring Support PINS ***/
      /*SMSGRDY Serial Message Ready Pin*/
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                   (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                   (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                   (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                   (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      /*SRDY Serial Ready Pin*/
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

      /*SEN Serial Enable Pin*/
      SYNC_SEN_DEASSERT(); // Make sure it is Deasserted not to confuse the Host.
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SEN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                               (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

      /*SFLOW Serial Flow Control, Byte or Bit Flow Control*/
      #if defined (SERIAL_SYNC_PIN_SFLOW)
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SFLOW] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                 (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                 (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                 (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                 (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      #endif //SERIAL_SYNC_PIN_SFLOW
 
      /*Configuring Serial SPI Pins*/
      #if !defined (SPI_IDLE_LOW_WORKAROUND)
      NRF_GPIO->OUTSET = (1UL << SERIAL_SYNC_PIN_SCLK);
      #endif
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SCLK] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                                (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SOUT] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                                (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                               (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      SERIAL_SYNC->PSELSCK = SERIAL_SYNC_PIN_SCLK;    // Assign port pin for Serial Clock
      SERIAL_SYNC->PSELMOSI = SERIAL_SYNC_PIN_SOUT;   // Assign port pin for Serial Out
      SERIAL_SYNC->PSELMISO = SERIAL_SYNC_PIN_SIN;    // Assign port pin for serial In


      #if defined (SPI_IDLE_LOW_WORKAROUND)
      // If NRF51 Chip Build is BA and below.
      SERIAL_SYNC->CONFIG = (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos) |
                            (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) |
                            (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
      #else
      // If NRF51 Chip build is CA and above.
      SERIAL_SYNC->CONFIG = (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos) |
                            (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) |
                            (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
      #endif

      #if defined (SERIAL_SYNC_USER_FREQUENCY)
      SERIAL_SYNC->FREQUENCY = (SERIAL_SYNC_USER_FREQUENCY << SPI_FREQUENCY_FREQUENCY_Pos);

      #else //!SERIAL_SYNC_USER_FREQUENCY
      #if defined (SERIAL_SYNC_PIN_BR3)
      NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_BR3] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                               (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);


      if (NRF_GPIO->IN & (1UL << SERIAL_SYNC_PIN_BR3))
      {
         SERIAL_SYNC->FREQUENCY = (SERIAL_SYNC_FREQUENCY_SLOW << SPI_FREQUENCY_FREQUENCY_Pos);
      }
      else
      #endif //SERIAL_SYNC_PIN_BR3
      {
         SERIAL_SYNC->FREQUENCY = (SERIAL_SYNC_FREQUENCY << SPI_FREQUENCY_FREQUENCY_Pos); /*default*/
      }

      #endif //SERIAL_SYNC_USER_FREQUENCY

      SERIAL_SYNC_SERIAL_ENABLE(); // enable and acquire associated serial pins

      while (IS_SRDY_ASSERTED()) // reset might have come from SRDY->MSGRDY assertion sequence. Host is waiting for us to assert SEN
      {
         /****
          * Make sure we are ready to catch SRDY falling edge when we do Assert SEN.
          **/

         /* Setup HiToLo Edge Interrupt ready*/
         sd_nvic_DisableIRQ(GPIOTE_IRQn);
         sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
         NRF_GPIOTE->CONFIG[EVENT_PIN_SRDY] = (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) |
                                                 (SERIAL_SYNC_PIN_SRDY << GPIOTE_CONFIG_PSEL_Pos) |
                                                 (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
         INT_PIN_SRDY_ENABLE();              // Get SRDY interrupt ready as we may about to assert SEN the HOST may respond quickly
         INT_PIN_SRDY_FLAG_CLEAR();
         sd_nvic_SetPriority(GPIOTE_IRQn, NRF_APP_PRIORITY_LOW);
         sd_nvic_EnableIRQ(GPIOTE_IRQn); // Enable GPIO Active low interrupt (Hi to Low)

         /* Setup SRDY De-Assert level sensing to wakeup when are about to sleep */
         NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                      (0UL << GPIO_PIN_CNF_SENSE_Pos);
         NRF_GPIOTE->EVENTS_PORT = 0;
         NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;
         NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);

         /* We're ready. Tell host we are and wait for SRDY to De-Assert first before we start transactions */
         SYNC_SEN_ASSERT();

         (void)sd_app_evt_wait();
         NRF_GPIOTE->EVENTS_PORT = 0;

         /* Run HFCLK to latch in interrupt and auto clear flags.*/
         sd_clock_hfclk_request();

      }

      INT_PIN_SRDY_DISABLE()

      /*force wakeup*/
      bSleep = 1;
      Serial_Wakeup();
   }
   else
#endif //SYNCHRONOUS_DISABLE
   {
   #if !defined (ASYNCHRONOUS_DISABLE)
      bSyncMode = false;

      /* Make sure peripheral is reseted, disable first*/
      SERIAL_ASYNC_SERIAL_DISABLE(); // disable peripheral to release associated peripherals
      sd_nvic_DisableIRQ(UART0_IRQn); // disable UART interrupts
      sd_nvic_ClearPendingIRQ(UART0_IRQn); // clear any pending interrupts
      SERIAL_ASYNC->INTENCLR = (UART_INTENCLR_TXDRDY_Clear << UART_INTENCLR_TXDRDY_Pos) | (UART_INTENCLR_RXDRDY_Clear << UART_INTENCLR_RXDRDY_Pos); // disable both ready interrupt
      SERIAL_ASYNC->TASKS_STOPTX = 1; // stop Transmit Task
      SERIAL_ASYNC->TASKS_STOPRX = 1; // stop Reception Task

      /*Starting up the peripheral*/
      // UART0 PERPOWER enabled in system_nrf51 init

      ucBaudrateNdx = BAUDRATE_DEFAULT_NDX;   // Default
#if defined (SERIAL_ASYNC_BR1) && (SERIAL_ASYNC_BR2) && (SERIAL_ASYNC_BR3)
      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_BR1] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                            (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                            (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_BR2] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                            (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                            (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_BR3] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                            (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                            (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      ucBaudrateNdx  = (NRF_GPIO->IN & (1UL << SERIAL_ASYNC_BR1)) ? 0x01 : 0x00;
      ucBaudrateNdx |= (NRF_GPIO->IN & (1UL << SERIAL_ASYNC_BR2)) ? 0x02 : 0x00;
      ucBaudrateNdx |= (NRF_GPIO->IN & (1UL << SERIAL_ASYNC_BR3)) ? 0x04 : 0x00;
#endif // SERIAL_ASYNC_BR1, SERIAL_ASYNC_BR2, SERIAL_ASYNC_BR3

#if defined (SERIAL_ASYNC_PIN_SLEEP)
      /* configure but don't arm yet the sleep pin */
      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SLEEP] =   (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                    (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                    (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                    (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
   #if defined (SERIAL_ASYNC_PIN_SUSPEND) // suspend has a dependency on sleep
      /* configure but don't arm yet the suspend pin */
      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SUSPEND] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                    (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                    (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                    (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
   #endif // SERIAL_ASYNC_PIN_SUSPEND
#endif // SERIAL_ASYNC_PIN_SLEEP

      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_RXD] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_TXD] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                                (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_RTS] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                                (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
      SERIAL_ASYNC->PSELRXD = SERIAL_ASYNC_PIN_RXD; // assign port pin for RXD
      SERIAL_ASYNC->PSELTXD = SERIAL_ASYNC_PIN_TXD; // assign port pin for TXD
      SERIAL_ASYNC->PSELRTS = SERIAL_ASYNC_PIN_RTS; // assign port pin for RTS

      SERIAL_ASYNC->CONFIG = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos) |
                            (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos); // flow control and parity
      SERIAL_ASYNC->BAUDRATE = asBaudControl[ucBaudrateNdx] << UART_BAUDRATE_BAUDRATE_Pos; // baudrate

      SERIAL_ASYNC_SERIAL_ENABLE(); // enable uart and acquire pins
      SERIAL_ASYNC_INT_TXRDY_ENABLE(); // enable tx ready interrupt to chain transmission
      SERIAL_ASYNC_INT_RXRDY_ENABLE(); // enable tx ready interrupt to chain transmission

   #if defined (TEST_UART_LOWER_PRIORITY)
      sd_nvic_SetPriority(UART0_IRQn, NRF_APP_PRIORITY_LOW); // set it up on application low priority
   #else
      sd_nvic_SetPriority(UART0_IRQn, NRF_APP_PRIORITY_HIGH); // set it up on application high priority
   #endif // TEST_UART_LOWER_PRIORITY
      sd_nvic_EnableIRQ(UART0_IRQn); // enable UART interrupt

      /* Though GPIOTE isn't being used, DETECT can trigger on any SENSE configured pin causing GPIOTE flag to be set.
       * Setting GPIOTE interrupt allows us to enter the ISR and clear the pending flag. */
      sd_nvic_SetPriority(GPIOTE_IRQn, NRF_APP_PRIORITY_LOW);
      sd_nvic_EnableIRQ(GPIOTE_IRQn); // enable GPIOTE interrupt

      //set to default rx
      Serial_ReleaseRx(); // release RTS to start reception
      SERIAL_ASYNC->TASKS_STARTRX = 1; // start Reception as early as now.
#endif // !ASYNCHRONOUS_DISABLE

   }

   stRxMessage.ANT_MESSAGE_ucSize = 0;
   stTxMessage.ANT_MESSAGE_ucSize = 0;
}

#if defined (SERIAL_SLEEP_POLLING_MODE) && defined (SERIAL_SLEEP_WAKEUP_RTC)
/**
 * @brief Configure serial interface sleep wakeup interval
 */
void Serial_Snooze_Interval_Set(uint32_t ulSleep30u517, uint8_t ucOnceOnly)
{
   if(ulSleep30u517 > 0x00FFFFFF) // must not be greater than the CC counter.
   {
      return;
   }
   ulSnoozeIntervalLong = ulSleep30u517;

   if (ucOnceOnly)
   {
      bSnoozeIntervalLongVariableOnceOnly = 1;
   }
}
#endif // SERIAL_SLEEP_POLLING_MODE

/**
 * @brief Serial interface sleep handler
 */
void Serial_Sleep(void)
{
#if defined (SERIAL_SLEEP_POLLING_MODE)
   /* NOTE: bSleep disabled to make sure Serial Wakeup is configured
    * everytime system is intending to go to sleep.*/
   //if (!bSleep) // if not in serial sleep mode
   {
      bSleep = 1; // indicate that serial is in sleep mode

      if (bSyncMode)
      {
   #if !defined (SYNCHRONOUS_DISABLE)
         sd_clock_hfclk_release();
         Gpiote_FallingEdge_Disable();

      #if defined (SERIAL_SLEEP_WAKEUP_RTC) && defined (APP_RTC_TIMER)
         sd_nvic_DisableIRQ(APP_NRF_RTC_IRQN);
         ucDozeIntervalCount = SERIAL_SLEEP_POLLTIME_DOZE; // reset doze wakeup count
         sd_nvic_EnableIRQ(APP_NRF_RTC_IRQN);

         SERIAL_POLL_NRF_RTC->CC[SERIAL_POLL_NRF_RTC_CC_INDEX] = 0; // set to 0 to reset wakeup time
         SERIAL_POLL_NRF_RTC->INTENSET = SERIAL_POLL_NRF_RTC_INTENSET_COMPARE_SET;
         SERIAL_POLL_NRF_RTC->EVTENSET = SERIAL_POLL_NRF_RTC_EVTENSET_COMPARE_SET;
         sd_nvic_SetPendingIRQ(APP_NRF_RTC_IRQN); // trigger rtc timer interrupt to set next wakeup time
      #else
         /* if serial is on hold (bHold) there is no use on waking up Message Ready request
          * since we should ignoring all incoming messages anyway */
         if (!bHold)  //
         {
            /*Use SENSE->DETECT on pin SMSGRDY at low*/
            /*SMSGRDY Serial Message Ready Pin*/
            /*from PAN 28 v1.2: 7. GPIO: SENSE mechanism fires under some circumstances when it should not.*/
            NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                         (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                         (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                         (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                         (0UL << GPIO_PIN_CNF_SENSE_Pos);
            /*ARM IT!*/
            NRF_GPIOTE->EVENTS_PORT = 0;
            NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

            NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY]|= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);


          }
      #endif //SERIAL_SLEEP_WAKEUP_RTC
   #endif //!SYNCHRONOUS_DISABLE
      }
      else
      {
   #if !defined (ASYNCHRONOUS_DISABLE)
      #if defined (SERIAL_ASYNC_PIN_SLEEP)
         uint8_t i;

         SERIAL_ASYNC->TASKS_STOPRX = 1;  // stop reception task
         SERIAL_ASYNC_RTS_DISABLE();      // disable RXRDY interrupt and relinquish control of RTS line
         sd_clock_hfclk_release();        // change power states by disabling hi freq clock

         /* go through every GPIO pin and save sense configuration while disabling sense during sleep */
         ulPinSenseEnabled = 0;
         ulPinSenseCfg = 0;
         for (i = 0; i < NUMBER_OF_NRF_GPIO_PINS; i++)
         {
            if (NRF_GPIO->PIN_CNF[i] & GPIO_PIN_CNF_SENSE_Msk)
            {
               ulPinSenseEnabled |= 0x01 << i;        // log pin enabled status
               if (((NRF_GPIO->PIN_CNF[i] & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos) == GPIO_PIN_CNF_SENSE_High)
                  ulPinSenseCfg |= 0x01 << i;         // log sense level status
            }
            // disable sense on this pin
            NRF_GPIO->PIN_CNF[i] &= ~((~GPIO_PIN_CNF_SENSE_Disabled) << GPIO_PIN_CNF_SENSE_Pos);
         }

         #if defined (SERIAL_ASYNC_PIN_SUSPEND)
         /* set up sense trigger on suspend signal low */
         /*from PAN 28 v1.2: 7. GPIO: SENSE mechanism fires under some circumstances when it should not.*/
         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SUSPEND] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                       (0UL << GPIO_PIN_CNF_SENSE_Pos);
         #endif // SERIAL_ASYNC_PIN_SUSPEND

         /* set up sense wakeup on sleep signal low */
         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SLEEP] =   (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                       (0UL << GPIO_PIN_CNF_SENSE_Pos);
         /*ARM IT!*/
         NRF_GPIOTE->EVENTS_PORT = 0;
         NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SLEEP] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
      #if defined (SERIAL_ASYNC_PIN_SUSPEND)
         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SUSPEND] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
      #endif // SERIAL_ASYNC_PIN_SUSPEND


      #endif // SERIAL_ASYNC_PIN_SLEEP
   #endif //!ASYNCHRONOUS_DSIABLE
       }
   }
#endif // SERIAL_SLEEP_POLLING_MODE && APP_RTC_TIMER
}

/**
 * @brief Serial interface wakeup handler
 */
static void Serial_Wakeup(void)
{
#if defined (SERIAL_SLEEP_POLLING_MODE)
   if(bSleep) // if in serial sleep mode
   {
      bSleep = 0; // indicate that serial is not in sleep mode

      if (bSyncMode)
      {
   #if !defined (SYNCHRONOUS_DISABLE)
      #if !defined (SERIAL_SLEEP_WAKEUP_RTC)
         /*check if we are waking up from an asserted smsgrdy with srdy asserted.*/
         if(NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] & (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos))
         {
            if (IS_SRDY_ASSERTED())
            {
               SYNC_SEN_DEASSERT(); // set the SEN to be high
               NVIC_SystemReset(); // Initiate System reset now.
            }
         }

         NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;
         /*Made sure SMSGRDY not sensitive*/
         NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
         NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
         NRF_GPIOTE->EVENTS_PORT = 0;
         ucPinSenseSRDY = PIN_SENSE_DISABLED;
         ucPinSenseMRDY = PIN_SENSE_DISABLED;

      #endif //SERIAL_SLEEP_WAKEUP_RTC

         sd_clock_hfclk_request();
         Gpiote_FallingEdge_Enable(); // re-enable GPIOTE sense
   #endif //!SYNCHRONOUS_DISABLE
      }
      else
      {
   #if !defined (ASYNCHRONOUS_DISABLE)
      #if defined (SERIAL_ASYNC_PIN_SLEEP)
         uint8_t i;

         NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

         /* restore pin sense configuration for wakeup */
         for (i = 0; i < NUMBER_OF_NRF_GPIO_PINS; i++)
         {
            if ((ulPinSenseEnabled >> i) & 0x01)
            {
               // pins were all disabled, so we can just OR the values in
               if ((ulPinSenseCfg >> i) & 0x01)
                  NRF_GPIO->PIN_CNF[i] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
               else
                  NRF_GPIO->PIN_CNF[i] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
            }
            else // disable sense on this pin
               NRF_GPIO->PIN_CNF[i] &= ~((~GPIO_PIN_CNF_SENSE_Disabled) << GPIO_PIN_CNF_SENSE_Pos);
         }

         #if defined (SERIAL_ASYNC_PIN_SUSPEND)
         /*Made sure SUSPEND not sensitive*/
         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SUSPEND] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                       (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
         #endif // SERIAL_ASYNC_PIN_SUSPEND
         /* Make sure SLEEP not sensitive */
         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SLEEP] =   (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                       (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
         NRF_GPIOTE->EVENTS_PORT = 0;
         NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SLEEP] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);


         stRxMessage.ANT_MESSAGE_ucSize = 0; // reset message counter
         sd_clock_hfclk_request();           // change power states by re-enabling hi freq clock
         if (!bHold)
            SERIAL_ASYNC_RTS_ENABLE();       // re-enable UART control of RTS line if it wasn't held to begin with
         SERIAL_ASYNC->TASKS_STARTRX = 1;    // start Reception as early as now.
      #endif // SERIAL_ASYNC_PIN_SLEEP
   #endif //!ASYNCHRONOUS_DISABLE
      }
   }
#endif // SERIAL_SLEEP_POLLING_MODE
}

/**
 * @brief Determine if the async suspend line is being asserted, and if so go to deep sleep
 */
void Serial_CheckAsyncSuspend(void)
{
#if !defined (ASYNCHRONOUS_DISABLE)
   if (!bSyncMode)
   {
   #if defined (SERIAL_ASYNC_PIN_SUSPEND)
      if (IS_PIN_SUSPEND_ASSERTED())      // check to make sure GPIO SUSPEND is still being asserted
      {
         if (!IS_PIN_SLEEP_ASSERTED())    // check to see if both pins are being asserted
         {
            NRF_GPIOTE->EVENTS_PORT = 0;  // clear interrupt flag
            return;                       // if both pins aren't being asserted, don't suspend
         }
         /*Made sure SUSPEND not sensitive*/
         NRF_GPIO->PIN_CNF[SERIAL_ASYNC_PIN_SUSPEND] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                       (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

         System_SetSuspendSleep();        // set deep sleep flag
         System_DeepSleep();              // immediately drop to deep sleep; reset upon wakeup
      }
      /* simply clear interrupt if GPIOTE is enabled and sleep fires, but not suspend */
      else
   #endif //SERIAL_ASYNC_PIN_SUSPEND
      if (NRF_GPIOTE->EVENTS_PORT)
         NRF_GPIOTE->EVENTS_PORT = 0;     // clear interrupt flag
   }
#endif //!ASYNCHRONOUS_DISABLE
}

/**
 * @brief Get input message buffer
 */
ANT_MESSAGE *Serial_GetRxMesgPtr(void)
{
   return ((ANT_MESSAGE *)&stRxMessage);
}

/**
 * @brief Get output message buffer
 */
ANT_MESSAGE *Serial_GetTxMesgPtr(void)
{
   return ((ANT_MESSAGE *)&stTxMessage);
}

/**
 * @brief Allow incoming serial communication
 */
void Serial_ReleaseRx(void)
{
   bHold = false; // release it

#if !defined (ASYNCHRONOUS_DISABLE)
   if(!bSyncMode)
   {
      stRxMessage.ANT_MESSAGE_ucSize = 0; // reset the serial receive state machine
      SERIAL_ASYNC_RTS_ENABLE();
   }
#endif // !ASYNCHRONOUS_DISABLE
}

/**
 * @brief Hold incoming serial communication
 */
void Serial_HoldRx(void)
{
   bHold = true; // hold it

#if !defined (ASYNCHRONOUS_DISABLE)
   if(!bSyncMode)
   {
      SERIAL_ASYNC_RTS_DISABLE();
   }
#endif // !ASYNCHRONOUS_DISABLE
}

/**
 * @brief Receive serial message
 */
bool Serial_RxMessage(void)
{
   if(bSyncMode)
   {
   #if !defined (SYNCHRONOUS_DISABLE)
      if (Sync_IsMSGRDYAsserted())
      {
         Serial_Wakeup();

         SyncProc_RxMessage();
         return true; // allow serial sleep
      }
   #endif // !SYNCHRONOUS_DISABLE
   }
   else
   {
   #if !defined (ASYNCHRONOUS_DISABLE)
      #if defined (SERIAL_ASYNC_PIN_SLEEP)
      if (!IS_PIN_SLEEP_ASSERTED())
      {
         Serial_Wakeup();
         return false; // disallow serial sleep
      }
      #endif //SERIAL_ASYNC_PIN_SLEEP
   #endif // !ASYNCHRONOUS_DISABLE
   }
   return true; // allow serial sleep
}

/**
 * @brief Send serial message
 */
void Serial_TxMessage(void)
{
   if(stTxMessage.ANT_MESSAGE_ucSize) // if message size is not empty, there should be something to transmit.
   {
      Serial_Wakeup();
   }
   #if !defined (SYNCHRONOUS_DISABLE)
   else
   {
      if(bSyncMode){
         SYNC_SEN_DEASSERT(); // make sure SEN is deasserted if we have nothing to transmit.
      }
      return; //buffer is empty, get out of here.
   }
   #endif

   if (!bTransmitting)
   {
      bStartMessage = true;
      bEndMessage = false;
      bTransmitting = true;

      stTxMessage.ANT_MESSAGE_ucCheckSum = MESG_TX_SYNC; // include MESG_TX_SYNC to checksum calculation

      for (ucTxPtr=0; ucTxPtr<=(stTxMessage.ANT_MESSAGE_ucSize + 1); ucTxPtr++) // have to go two more than size so we include the size and the ID
         stTxMessage.ANT_MESSAGE_ucCheckSum ^= stTxMessage.aucMessage[ucTxPtr]; // calculate the checksum

      stTxMessage.ANT_MESSAGE_aucMesgData[stTxMessage.ANT_MESSAGE_ucSize] = stTxMessage.ANT_MESSAGE_ucCheckSum; // move the calculated checksum to the correct location in the message

      if(bSyncMode)
      {
         SyncProc_TxMessage();
      }
      else
      {
         AsyncProc_TxMessage(); // kick the first transmission.
         do
         {
            (void)sd_app_evt_wait();
         }
         while (stTxMessage.ANT_MESSAGE_ucSize); // wait for the whole message to be transmitted
      }
   }
}

#if !defined (SYNCHRONOUS_DISABLE)
/**
 * @brief Enable active low detection for SMSGRDY and SRDY
 */
static void Gpiote_FallingEdge_Enable (void)
{
   sd_nvic_DisableIRQ(GPIOTE_IRQn);
   sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);

   if (bSyncMode)
   {
      /*SMSGRDY Hi to Lo interrupt*/
      NRF_GPIOTE->CONFIG[EVENT_PIN_SMSGRDY] = (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                                              (SERIAL_SYNC_PIN_SMSGRDY << GPIOTE_CONFIG_PSEL_Pos) |
                                              (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
      /*SRDY Hi to Lo interrupt*/
      NRF_GPIOTE->CONFIG[EVENT_PIN_SRDY] = (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) |
                                           (SERIAL_SYNC_PIN_SRDY << GPIOTE_CONFIG_PSEL_Pos) |
                                           (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);

      INT_PIN_SMSGRDY_ENABLE();
   }

   sd_nvic_SetPriority(GPIOTE_IRQn, NRF_APP_PRIORITY_LOW);
   sd_nvic_EnableIRQ(GPIOTE_IRQn); // enable GPIO Active low interrupt (Hi to Low)
}
#endif // !SYNCHRONOUS_DISABLE

#if !defined (SYNCHRONOUS_DISABLE)
/**
 * @brief Disable active low detection for SMSGRDY and SRDY
 */
static void Gpiote_FallingEdge_Disable (void)
{
   sd_nvic_DisableIRQ(GPIOTE_IRQn);
   sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);

   if (bSyncMode)
   {
      NRF_GPIOTE->CONFIG[EVENT_PIN_SMSGRDY] = 0UL;
      NRF_GPIOTE->CONFIG[EVENT_PIN_SRDY] = 0UL;
   }
}
#endif // !SYNCHRONOUS_DISABLE

#if !defined (SYNCHRONOUS_DISABLE)
/**
 * @brief Handle SRDY Sleep
 */
static void SyncSRDYSleep(void)
{
   uint8_t ucDelay;

   if (bStartMessage)
   {
      bStartMessage = false; // clear the start flag
      INT_PIN_SRDY_FLAG_CLEAR(); // clear the interrupt status bit
      INT_PIN_SRDY_ENABLE(); // get SRDY interrupt ready as we may about to assert SEN the HOST may respond quickly
      SYNC_SEN_ASSERT(); // set the serial enable low (active)
   }

   ucDelay = SYNC_SRDY_SLEEP_DELAY;
   if(!bSRdy)
   {
      do
      {
         if (ucDelay)
            ucDelay--;

         if(!ucDelay)
         {
            //enable sensing first before disarming SRDY interrupt.
            NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                         (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                         (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                         (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                         (0UL << GPIO_PIN_CNF_SENSE_Pos);
            NRF_GPIOTE->EVENTS_PORT = 0;
            NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;
            NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY]|= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

            sd_clock_hfclk_release();
            Gpiote_FallingEdge_Disable();

            while (!bSRdy)
            {
               (void)sd_app_evt_wait();

               if(NRF_GPIOTE->EVENTS_PORT || (ucPinSenseSRDY == PIN_SENSE_LOW))// check if SRDY woke us.
               {
                  NRF_GPIOTE->EVENTS_PORT = 0;
                  ucPinSenseSRDY = PIN_SENSE_DISABLED;

                  NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                               (0UL << GPIO_PIN_CNF_SENSE_Pos);

                  NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                                               (0UL << GPIO_PIN_CNF_SENSE_Pos);
                  NRF_GPIOTE->EVENTS_PORT = 0;
                  sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
                  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

                  NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);

                  if(!IS_MRDY_ASSERTED())
                  {
                     NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
                  }

                  while (IS_SRDY_ASSERTED())
                  {
                     (void)sd_app_evt_wait();

                     if(NRF_GPIOTE->EVENTS_PORT || (ucPinSenseSRDY == PIN_SENSE_HIGH) || (ucPinSenseMRDY == PIN_SENSE_LOW))
                     {
                        NRF_GPIOTE->EVENTS_PORT = 0;
                        if(IS_MRDY_ASSERTED() && (((NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos ) == GPIO_PIN_CNF_SENSE_Low))
                        {
                           SYNC_SEN_DEASSERT(); // set the SEN to be high
                           NVIC_SystemReset(); // initiate System reset now.
                        }
                     }
                  }

                  bSRdy = 1; // yes, SRDY did.
               }
            }
         }
      }while (!bSRdy || IS_SRDY_ASSERTED());

      if (!ucDelay)
      {
         bSleep = 1;
         Serial_Wakeup();
      }
   }
   bSRdy = false; // take out the semaphore

   if (bEndMessage) // handle the end of the message
   {
      bEndMessage = false; // clear the end flag
      INT_PIN_SRDY_DISABLE(); // disable SRDY interrupt since we don't need it.
      SYNC_SEN_DEASSERT(); // set the serial enable high (inactive)
   }
}
#endif // !SYNCHRONOUS_DISABLE

#if !defined (SYNCHRONOUS_DISABLE)
/**
 * @brief Handle synchronous byte transaction
 */
static uint8_t SyncReadWriteByte(uint8_t ucByte)
{
   uint8_t ucReadByte;

   ucReadByte = 0; // initialize the receive byte

   SyncSRDYSleep(); // wait for an SRDY before beginning

   SERIAL_SYNC->TXD = ucByte; // send transmit
   while (!SERIAL_SYNC->EVENTS_READY); // wait for data to be ready
   SERIAL_SYNC->EVENTS_READY = 0; // clear bit
   ucReadByte = SERIAL_SYNC->RXD; // read back byte

   return ucReadByte;
}
#endif // !SYNCHRONOUS_DISABLE

#if !defined (SYNCHRONOUS_DISABLE)
/**
 * @brief Synchronous rx message
 */
static void SyncProc_RxMessage(void)
{
   uint8_t ucRxSize;
   uint8_t ucRxCheckSum;

   bStartMessage = true; // flag the start of a message

   SyncReadWriteByte(MESG_RX_SYNC); // write the read message sync byte

   ucRxSize     = SyncReadWriteByte(0xFF); // read the message size byte
   ucRxCheckSum = MESG_RX_SYNC ^ ucRxSize; // initialze the checksum

   if ((ucRxSize >= SERIAL_RX_BUFFER_SIZE) || (ucRxSize  == 0)) // if the message is too big for our receive buffer or empty
   {
      SYNC_SEN_DEASSERT(); // set the serial enable high (inactive)
      return; // exit
   }

   for (ucRxPtr=0; ucRxPtr<=ucRxSize; ucRxPtr++) // we have to account for the message ID too that's why it's <=
   {
      stRxMessage.ANT_MESSAGE_aucFramedData[ucRxPtr] = SyncReadWriteByte(0xFF); // read the byte
      ucRxCheckSum ^= stRxMessage.ANT_MESSAGE_aucFramedData[ucRxPtr]; // recalculate the checksum
   }

#ifndef MPG
   bEndMessage = true; // flag the end of the message
   ucRxCheckSum ^= SyncReadWriteByte(0xFF); // read the checksum byte and xor it with the calculated checksum

#else
   /* MPG boards have SEN tied to the SPI CS line, so we need to keep it high until 
   the transaction is entirely done.  Add one more SRDY from host to know that the byte 
   was received before deasserting SEN by keeping bEndMessage false before writing, then add
   one more wait for SRDY. */
   ucRxCheckSum ^= SyncReadWriteByte(0xFF); // read the checksum byte and xor it with the calculated checksum
   bEndMessage = true; // flag the end of the message
   SyncSRDYSleep(); // wait for an SRDY before continuing
#endif /* MPG */

   if (!ucRxCheckSum) // if we passed the checksum
   {
      Serial_HoldRx();
      bEventRXSerialMessageProcess = true; // flag that we have a rx serial message to process
   }

   stRxMessage.ANT_MESSAGE_ucSize = ucRxSize; // save the receive message size
}
#endif // !SYNCHRONOUS_DISABLE

/**
 * @brief Synchronous tx message
 */
static void SyncProc_TxMessage(void)
{
#if !defined (SYNCHRONOUS_DISABLE)
   uint8_t *pucData;
   uint8_t ucTxSize;

   bStartMessage = true; // flag the start of a message
   ucTxSize = stTxMessage.ANT_MESSAGE_ucSize; // read out the transmit size
   stTxMessage.ANT_MESSAGE_ucSize = 0; // clear the transmit size

   SyncReadWriteByte(MESG_TX_SYNC); // send the SYNC byte
   SyncReadWriteByte(ucTxSize); // send the size byte

   ucTxSize += 1; // add 1 more bytes to include MessageID
   pucData = (uint8_t *)&stTxMessage.ANT_MESSAGE_aucFramedData[0];// point the data for transmission

   do
   {
      SyncReadWriteByte(*pucData++); // send all bytes
   }
   while (--ucTxSize);

#ifndef MPG
   bEndMessage = true; // flag the end of the message
   SyncReadWriteByte(*pucData); // send the last byte it should be the checksum

#else
   /* MPG boards have SEN tied to the SPI CS line, so we need to keep it high until 
   the transaction is entirely done.  Add one more SRDY from host to know that the byte 
   was received before deasserting SEN by keeping bEndMessage false before writing, then add
   one more wait for SRDY. */
   SyncReadWriteByte(*pucData); // send the last byte it should be the checksum
   bEndMessage = true; // flag the end of the message
   SyncSRDYSleep(); // wait for an SRDY before beginning
#endif /* MPG */
   
   bTransmitting = false; // transmission done.
#endif // !SYNCHRONOUS_DISABLE
}

#if !defined (SYNCHRONOUS_DISABLE)
/**
 * @brief Synchronous MSGRDY check
 */
static bool Sync_IsMSGRDYAsserted(void)
{
   if(bHold)
      return false;

   if (IS_MRDY_ASSERTED())
   {
      return true;
   }
   else
   {
      return false;
   }
}
#endif // !SYNCHRONOUS_DISABLE

/**
 * @brief Asynchronous tx message
 */
static void AsyncProc_TxMessage(void)
{
#if !defined (ASYNCHRONOUS_DISABLE)
   SERIAL_ASYNC->EVENTS_TXDRDY = 0;  // make sure flag is clear to make way for the setting on empty

   if (stTxMessage.ANT_MESSAGE_ucSize && !bEndMessage) // if we have a message that is ready to send
   {
      if (bStartMessage) // we have to write the SYNC byte as well
      {
         bStartMessage = false;
         ucTxPtr = 0; // initialize the message pointer for below

         SERIAL_ASYNC_START_TX();
         SERIAL_ASYNC->TXD = MESG_TX_SYNC; // write the sync byte
      }
      else if (ucTxPtr <= (stTxMessage.ANT_MESSAGE_ucSize+2)) // send the data and checksum
      {
         SERIAL_ASYNC->TXD = stTxMessage.aucMessage[ucTxPtr++];
      }
      else
      {
         SERIAL_ASYNC_STOP_TX();
         stTxMessage.ANT_MESSAGE_ucSize = 0;
         bEndMessage = true;
         bTransmitting = false;
      }
   }
   else
   {
      SERIAL_ASYNC_STOP_TX();
      stTxMessage.ANT_MESSAGE_ucSize = 0;
      bEndMessage = true;
      bTransmitting = false;
   }
#endif // !ASYNCHRONOUS_DISABLE
}

/**
 * @brief Asynchronous rx message
 */
#if !defined (ASYNCHRONOUS_DISABLE)
#define MESG_SIZE_READ     ((uint8_t)0x55) // async control flag
static void AsyncProc_RxMessage(void)
{
   uint8_t ucByte;
   uint8_t ucURxStatus;

   ucURxStatus = SERIAL_ASYNC->ERRORSRC; // read the overflow flag

   SERIAL_ASYNC->EVENTS_RXDRDY = 0;
   ucByte = SERIAL_ASYNC->RXD; // read the incoming char, clear the pending interrupt flag

   if (ucURxStatus & UART_ERRORSRC_FRAMING_Msk) // if we had a character error
   {
      SERIAL_ASYNC->ERRORSRC = ucURxStatus;
      stRxMessage.ANT_MESSAGE_ucSize = 0;
   }
   if (ucURxStatus & (UART_ERRORSRC_PARITY_Msk))
   {
      SERIAL_ASYNC->ERRORSRC = ucURxStatus;
      stRxMessage.ANT_MESSAGE_ucSize = 0;
   }
   if (ucURxStatus & (UART_ERRORSRC_OVERRUN_Msk))
   {
      SERIAL_ASYNC->ERRORSRC = ucURxStatus;
      stRxMessage.ANT_MESSAGE_ucSize = 0;

   }

   if (!stRxMessage.ANT_MESSAGE_ucSize) // we are looking for the sync byte of a message
   {
      if (ucByte == MESG_TX_SYNC) // this is a valid SYNC byte
      {

         stRxMessage.ANT_MESSAGE_ucCheckSum = MESG_TX_SYNC; // init the checksum
         stRxMessage.ANT_MESSAGE_ucSize     = MESG_SIZE_READ; // set the byte pointer to get the size byte
      }
   }
   else if (stRxMessage.ANT_MESSAGE_ucSize == MESG_SIZE_READ) // if we are processing the size byte of a message
   {
      stRxMessage.ANT_MESSAGE_ucSize = 0; // if the size is invalid we want to reset the rx message

      if (ucByte <= MESG_MAX_SIZE_VALUE) // make sure this is a valid message
      {
         stRxMessage.ANT_MESSAGE_ucSize      = ucByte; // save the size of the message
         stRxMessage.ANT_MESSAGE_ucCheckSum ^= ucByte; // calculate checksum
         ucRxPtr       = 0; // set the byte pointer to start collecting the message
      }
   }
   else
   {
      stRxMessage.ANT_MESSAGE_ucCheckSum ^= ucByte; // calculate checksum

      if (ucRxPtr > stRxMessage.ANT_MESSAGE_ucSize) // we have received the whole message + 1 for the message ID
      {
         if (!stRxMessage.ANT_MESSAGE_ucCheckSum) // the checksum passed
         {
            Serial_HoldRx();
            bEventRXSerialMessageProcess = true; // flag that we have a rx serial message to process
         }
         else
         {
            stRxMessage.ANT_MESSAGE_ucSize = 0; // reset the RX message
         }
      }
      else // this is a data byte
      {
         stRxMessage.ANT_MESSAGE_aucFramedData[ucRxPtr++] = ucByte; // save the byte
      }
   }
}
#endif // !ASYNCHRONOUS_DISABLE

/**
 * @brief Interrupt handler for asynchronous serial interface
 */
void Serial_UART0_IRQHandler(void)
{
#if !defined(ASYNCHRONOUS_DISABLE)

   if (SERIAL_ASYNC->EVENTS_RXDRDY && (SERIAL_ASYNC->INTENSET & (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos))) // TODO: TEST
   {
      AsyncProc_RxMessage();
   }

   if (SERIAL_ASYNC->EVENTS_TXDRDY && (SERIAL_ASYNC->INTENSET & (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos))) // TODO: TEST
   {
      AsyncProc_TxMessage();
   }


#endif // !ASYNCHRONOUS_DISABLE
}

/**
 * @brief Interrupt handler for synchronous serial SMSGRDY and SRDY interrupt. Uses GPIOTE 0 and 1
 */
void Serial_GPIOTE_IRQHandler(void)
{
   if (bSyncMode)
   {
   #if !defined (SYNCHRONOUS_DISABLE)
      if (IS_INT_PIN_SMSGRDY_ASSERTED()) // MRDY / SLEEP_ENABLE
      {
      #if !defined (SYNCHRONOUS_DISABLE)
         if (bSyncMode && IS_SRDY_ASSERTED()) // if this is sync, and SRDY is low and not pending, then this is a sync reset operation
         {
            SYNC_SEN_DEASSERT(); // set the SEN to be high
            NVIC_SystemReset(); // initiate System reset now.
         }
         INT_PIN_SMSGRDY_FLAG_CLEAR(); // clear the interrupt status bit
      #endif // !SYNCHRONOUS_DISABLE
      }

      if (IS_INT_PIN_SRDY_ASSERTED())// && (bSRdy == false)) // SRDY is pending and semaphore is waiting
      {
         INT_PIN_SRDY_FLAG_CLEAR(); // clear the interrupt status bit
         bSRdy = true; // set the srdy flag
      }

      if(NRF_GPIOTE->EVENTS_PORT)// && (NRF_GPIOTE->INTENSET & (1UL << GPIOTE_INTENSET_PORT_Pos) ))
      {
         /*Detecting SRDY LOW*/
         if(((NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos ) == GPIO_PIN_CNF_SENSE_Low)
         {
            ucPinSenseSRDY = PIN_SENSE_LOW;
         }
         /*Detecting SRDY HIGH*/
         if(((NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SRDY] & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos ) == GPIO_PIN_CNF_SENSE_High)
         {
            ucPinSenseSRDY = PIN_SENSE_HIGH;
         }
         /*Detecting MRDY LOW*/
         if(((NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos ) == GPIO_PIN_CNF_SENSE_Low)
         {
            ucPinSenseMRDY = PIN_SENSE_LOW;
         }
         /*Detecting MRDY High*/
         if(((NRF_GPIO->PIN_CNF[SERIAL_SYNC_PIN_SMSGRDY] & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos ) == GPIO_PIN_CNF_SENSE_High)
         {
            ucPinSenseMRDY = PIN_SENSE_HIGH;
         }
         NRF_GPIOTE->EVENTS_PORT = 0;
      }
    #endif //!SYNCHRONOUS_DISABLE
   }
   else
   {
   #if !defined (ASYNCHRONOUS_DISABLE)
      if(NRF_GPIOTE->EVENTS_PORT)
         NRF_GPIOTE->EVENTS_PORT = 0;
   #endif //!ASYNCHRONOUS_DISABLE
   }

   sd_nvic_ClearPendingIRQ(GPIOTE_IRQn); // clear interrupt flag
}

/**
 * @brief Interrupt handler for RTC usage by serial interface
 */
void Serial_RTC_IRQHandler(uint32_t ulCurrentTime)
{
#if defined (SERIAL_SLEEP_POLLING_MODE) && defined (APP_RTC_TIMER) && defined (SERIAL_SLEEP_WAKEUP_RTC)
   if (SERIAL_POLL_NRF_RTC->INTENSET & SERIAL_POLL_NRF_RTC_INTENSET_COMPARE_SET)
   {
      uint32_t ulWakeupTime;

      SERIAL_POLL_NRF_RTC->EVENTS_COMPARE[SERIAL_POLL_NRF_RTC_CC_INDEX] = 0; // clear

      // note: counter is always reset to 0 on each interrupt, therefore we can do the following time compare
      if (SERIAL_POLL_NRF_RTC->CC[SERIAL_POLL_NRF_RTC_CC_INDEX] > ulCurrentTime + 2) // not yet reached target time //+2 ensures N+2 away from current time
      {
         ulWakeupTime = SERIAL_POLL_NRF_RTC->CC[SERIAL_POLL_NRF_RTC_CC_INDEX] - ulCurrentTime; // remaining time
      }
      else // on or exceeded target time
      {
         if (!ucDozeIntervalCount)
         {
            if (bSnoozeIntervalBurstMode)
               ulWakeupTime = SERIAL_SLEEP_POLLTIME_SNOOZE_INTERVAL_SHORT; // short wakeup intervals during burst transfers
            else
            {
               ulWakeupTime = ulSnoozeIntervalLong; // long wakeup interval
               if(bSnoozeIntervalLongVariableOnceOnly)
               {
                  bSnoozeIntervalLongVariableOnceOnly = 0;
                  ulSnoozeIntervalLong = SERIAL_SLEEP_POLLTIME_SNOOZE_INTERVAL_LONG;
               }
            }
         }
         else
         {
            ucDozeIntervalCount--; // decrement if we haven't reached to count 0
            ulWakeupTime = SERIAL_SLEEP_POLLTIME_DOZE_INTERVAL;
         }
      }

      SERIAL_POLL_NRF_RTC->CC[SERIAL_POLL_NRF_RTC_CC_INDEX] = ulWakeupTime; // set next wakeup
   }
#endif // SERIAL_SLEEP_POLLING_MODE && APP_RTC_TIMER
}

/**
 * @brief ANT event handler used by serial interface
 */
void Serial_ANTEventHandler(uint8_t ucEvent, ANT_MESSAGE *pstANTMessage)
{
   if ((ucEvent == EVENT_TRANSFER_NEXT_DATA_BLOCK) || (ucEvent == EVENT_TRANSFER_TX_COMPLETED) || (ucEvent == EVENT_TRANSFER_TX_FAILED) || (ucEvent == EVENT_QUE_OVERFLOW))
   {
      if (bQueuedTxBurst) // if a burst transfer process was queued
      {
         bQueuedTxBurst = 0; // clear flag
         Serial_ReleaseRx(); // release input buffer
      }

      if (ucEvent == EVENT_TRANSFER_NEXT_DATA_BLOCK)
      {
         pstANTMessage->ANT_MESSAGE_ucSize = 0; // do not send out this event message
      }
      else if (ucEvent == EVENT_TRANSFER_TX_FAILED)
      {
         ucBurstSequence = 0; // reset input burst sequence upon tx burst failure
      }

   #if defined (SERIAL_SLEEP_POLLING_MODE) && defined (SERIAL_SLEEP_WAKEUP_RTC)
      if ((ucEvent == EVENT_TRANSFER_TX_COMPLETED) || (ucEvent == EVENT_TRANSFER_TX_FAILED) || (ucEvent == EVENT_QUE_OVERFLOW))
      {
         sd_nvic_DisableIRQ(APP_NRF_RTC_IRQN);
         bSnoozeIntervalBurstMode = false; // burst transfer has ended
         sd_nvic_EnableIRQ(APP_NRF_RTC_IRQN);
      }
   #endif // SERIAL_SLEEP_POLLING_MODE
   }
#if defined (SERIAL_SLEEP_POLLING_MODE)  && defined (SERIAL_SLEEP_WAKEUP_RTC)
   else if (ucEvent == EVENT_TRANSFER_TX_START)
   {
      sd_nvic_DisableIRQ(APP_NRF_RTC_IRQN);
      bSnoozeIntervalBurstMode = true; // burst tranfer has started
      sd_nvic_EnableIRQ(APP_NRF_RTC_IRQN);
   }
#endif // SERIAL_SLEEP_POLLING_MODE
}
