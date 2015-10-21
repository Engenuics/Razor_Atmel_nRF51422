 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#ifndef BOARDCONFIG_H
#define BOARDCONFIG_H

#define MPG /* Define to use MPG 51422 hardware */

#ifdef MPG
#define ASYNCHRONOUS_DISABLE
#define TEST_32K_SYNTH

/* Port 0 bit positions */
#define P0_31_               (uint32_t)0x80000000
#define P0_30_NC             (uint32_t)0x40000000
#define P0_29_LED_RED        (uint32_t)0x20000000
#define P0_28_LED_YLW        (uint32_t)0x10000000
#define P0_27_LED_GRN        (uint32_t)0x08000000
#define P0_26_LED_BLU        (uint32_t)0x04000000 
#define P0_25_NC             (uint32_t)0x02000000
#define P0_24_NC             (uint32_t)0x01000000
#define P0_23_NC             (uint32_t)0x00800000
#define P0_22_NC             (uint32_t)0x00400000 
#define P0_21_NC             (uint32_t)0x00200000
#define P0_20_NC             (uint32_t)0x00100000
#define P0_19_NC             (uint32_t)0x00080000
#define P0_18_NC             (uint32_t)0x00040000
#define P0_17_NC             (uint32_t)0x00020000
#define P0_16_TP65           (uint32_t)0x00010000
#define P0_15_TP64           (uint32_t)0x00008000
#define P0_14_TP63           (uint32_t)0x00004000
#define P0_13_ANT_SPI_MOSI   (uint32_t)0x00002000
#define P0_12_ANT_SPI_MISO   (uint32_t)0x00001000
#define P0_11_ANT_SPI_SCK    (uint32_t)0x00000800
#define P0_10_ANT_SEN        (uint32_t)0x00000400
#define P0_09_ANT_SRDY       (uint32_t)0x00000200
#define P0_08_ANT_MRDY       (uint32_t)0x00000100
#define P0_07_NC             (uint32_t)0x00000080
#define P0_06_NC             (uint32_t)0x00000040
#define P0_05_NC             (uint32_t)0x00000020
#define P0_04_NC             (uint32_t)0x00000010
#define P0_03_NC             (uint32_t)0x00000008
#define P0_02_NC             (uint32_t)0x00000004
#define P0_01_NC             (uint32_t)0x00000002 
#define P0_00_NC             (uint32_t)0x00000001 

#define P0_29_LED_RED_CNF   ( (GPIO_DIRCLR_PIN0_Output     << GPIO_PIN_CNF_DIR_Pos)   | \
                              (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) | \
                              (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)  | \
                              (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos) | \
                              (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) )

#define P0_28_LED_YLW_CNF   ( (GPIO_DIRCLR_PIN0_Output     << GPIO_PIN_CNF_DIR_Pos)   | \
                              (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) | \
                              (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)  | \
                              (GPIO_PIN_CNF_DRIVE_S0H1     << GPIO_PIN_CNF_DRIVE_Pos) | \
                              (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) )

#define P0_27_LED_GRN_CNF   ( (GPIO_DIRCLR_PIN0_Output     << GPIO_PIN_CNF_DIR_Pos)   | \
                              (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) | \
                              (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)  | \
                              (GPIO_PIN_CNF_DRIVE_S0H1     << GPIO_PIN_CNF_DRIVE_Pos) | \
                              (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) )

#define P0_26_LED_BLU_CNF   ( (GPIO_DIRCLR_PIN0_Output     << GPIO_PIN_CNF_DIR_Pos)   | \
                              (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) | \
                              (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)  | \
                              (GPIO_PIN_CNF_DRIVE_S0H1     << GPIO_PIN_CNF_DRIVE_Pos) | \
                              (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) )
#endif /* MPG */

 /* Common Board Configuration */
#define APP_RTC_CLOCK_FREQ                            32768
#define APP_RTC_MAX_COUNTER                           0xFFFFFF
#define APP_RTC_PRESCALER                             0 // counter resolution 30.517us

#define APP_NRF_RTC                                   NRF_RTC1
#define APP_NRF_RTC_IRQN                              RTC1_IRQn
#define APP_NRF_RTC_PERPOWER_MSK                      POWER_PERPOWER_RTC1_Msk
#define APP_NRF_RTC_PERRDY_MSK                        POWER_PERRDY_RTC1_Msk

/* Used for Serial wake-up */
#define SERIAL_POLL_NRF_RTC                           APP_NRF_RTC
#define SERIAL_POLL_NRF_RTC_CC_INDEX                  3 // using cc[3]
#define SERIAL_POLL_NRF_RTC_INTENSET_COMPARE_SET      (RTC_INTENSET_COMPARE3_Set << RTC_INTENSET_COMPARE3_Pos)
#define SERIAL_POLL_NRF_RTC_INTENCLR_COMPARE_CLEAR    (RTC_INTENCLR_COMPARE3_Clear << RTC_INTENCLR_COMPARE3_Pos)
#define SERIAL_POLL_NRF_RTC_EVTENSET_COMPARE_SET      (RTC_EVTENSET_COMPARE3_Set << RTC_EVTENSET_COMPARE3_Pos)
#define SERIAL_POLL_NRF_RTC_EVTENCLR_COMPARE_CLEAR    (RTC_EVTENCLR_COMPARE3_Clear << RTC_EVTENCLR_COMPARE3_Pos)


   // Initial pin config
   #define PIN_DIR_INIT                      (0UL)       // TODO:
   #define PIN_OUT_INIT                      (0UL)       // TODO:

#ifndef MPG
   // Serial Communication Configuration
   #define SERIAL_PIN_PORTSEL                (0UL)       // serial mode detection. low =  Asynchronous high = Synchronous
#endif
   
   // Async Serial
   #if !defined (ASYNCHRONOUS_DISABLE)
      #define SERIAL_ASYNC_PIN_RTS           (5UL)       // out
      #define SERIAL_ASYNC_PIN_TXD           (15UL)      // out
      #define SERIAL_ASYNC_PIN_RXD           (12UL)      // in
      #define SERIAL_ASYNC_PIN_SLEEP         (2UL)       // in
      #define SERIAL_ASYNC_PIN_SUSPEND       (23UL)      // in
      #define SERIAL_ASYNC_BR1               (6UL)       // baud rate pin 1 selector
      #define SERIAL_ASYNC_BR2               (24UL)      // baud rate pin 2 selector
      #define SERIAL_ASYNC_BR3               (9UL)       // baud rate pin 3 selector
   #endif //!ASYNCHRONOUS_DISABLE

   // Sync Serial
#ifndef MPG
  /* Default pin configuration */                             
  #if !defined (SYNCHRONOUS_DISABLE)
      #define SERIAL_SYNC_PIN_SMSGRDY        (2UL)       // in
      #define SERIAL_SYNC_PIN_SRDY           (23UL)      // in
      #define SERIAL_SYNC_PIN_SEN            (5UL)       // out
      #define SERIAL_SYNC_PIN_SIN            (12UL)      // in
      #define SERIAL_SYNC_PIN_SOUT           (15UL)      // out
      #define SERIAL_SYNC_PIN_SCLK           (24UL)      // out
//    #define SERIAL_SYNC_PIN_SFLOW          (6UL)       // in. Bit synchronous not supported
      #define SERIAL_SYNC_PIN_BR3            (9UL)       // bit rate pin selector
   #endif //!SYNCHRONOUS_DISABLE

#else
  /* MPG pin configuration */
  #if !defined (SYNCHRONOUS_DISABLE)
      #define SERIAL_SYNC_PIN_SMSGRDY        (8UL)       // in
      #define SERIAL_SYNC_PIN_SRDY           (9UL)       // in
      #define SERIAL_SYNC_PIN_SEN            (10UL)      // out
      #define SERIAL_SYNC_PIN_SIN            (12UL)      // in
      #define SERIAL_SYNC_PIN_SOUT           (13UL)      // out
      #define SERIAL_SYNC_PIN_SCLK           (11UL)      // out
//    #define SERIAL_SYNC_PIN_SFLOW          (6UL)       // in. Bit synchronous not supported
//    #define SERIAL_SYNC_PIN_BR3            (9UL)       // bit rate pin selector MPG: not implemented and we want slow rate which is default if this is not defined
      #define LED_RED_BIT                     (uint32_t)29
      #define LED_YLW_BIT                     (uint32_t)28
      #define LED_GRN_BIT                     (uint32_t)27
      #define LED_BLU_BIT                     (uint32_t)26
   #endif //!SYNCHRONOUS_DISABLE
#endif /* MPG */


#endif // BOARDCONFIG_H
