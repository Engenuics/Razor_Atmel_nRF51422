 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#include "command.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"

#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_error.h"
#include "appconfig.h"
#include "boardconfig.h"
#include "dsi_utility.h"
#include "global.h"
#include "main.h"
#include "serial.h"
#include "system.h"
#include "nrf_error.h"

#define MAX_ANT_CHANNELS                  ((uint8_t)8) // MUST MATCH THE NUMBER OF CHANNELS SUPPORTED BY ANT STACK
#define MAX_NETWORKS                      ((uint8_t)8) // MUST MATCH THE NUMBER OF NETWORKS SUPPORTED BY ANT STACK

#define SERIAL_RESPONSE_ID_OFFSET         ((uint8_t)0) // offset to the RESPONSE mesg ID
#define SERIAL_RESPONSE_OFFSET            ((uint8_t)1) // offset to the RESPONSE value in the serial transmit buffer for a response message
#define SERIAL_DATA_OFFSET_1              ((uint8_t)0) // general data byte number 1 offset
#define SERIAL_DATA_OFFSET_2              ((uint8_t)1) // general data byte number 2 offset
#define SERIAL_DATA_OFFSET_3              ((uint8_t)2) // general data byte number 3 offset
#define SERIAL_DATA_OFFSET_4              ((uint8_t)3) // general data byte number 4 offset

#if defined (USE_INTERFACE_LOCK)
   static volatile bool bInterfaceLock = false;
#endif // USE_INTERFACE_LOCK

#if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
   static uint8_t SetChannelToSerialID(ANT_MESSAGE *pstRxMessage);
#endif // !SERIAL_NUMBER_NOT_AVAILABLE


/**
 * @brief ANT serial burst command message handler
 */
bool Command_BurstMessageProcess(ANT_MESSAGE *pstRxMessage, ANT_MESSAGE *pstTxMessage)
{
   COMMAND_RESPONSE stCmdResp;
   uint8_t ucBurstSegment = 0;
   uint8_t ucSequence = pstRxMessage->ANT_MESSAGE_ucChannel & SEQUENCE_NUMBER_MASK;

   pstRxMessage->ANT_MESSAGE_ucChannel &= ~SEQUENCE_NUMBER_MASK;

   if ((uint8_t)(ucBurstSequence << 1) != (uint8_t)(ucSequence << 1))
   {
      ucBurstSequence = 0;
      stCmdResp.ucResponse = TRANSFER_SEQUENCE_NUMBER_ERROR;
   }
   else // sequence match
   {
      if ((uint8_t)(ucSequence << 1) == (uint8_t)(SEQUENCE_NUMBER_ROLLOVER << 1))
         ucBurstSequence = SEQUENCE_NUMBER_INC;
      else
         ucBurstSequence += SEQUENCE_NUMBER_INC;

      if ((uint8_t)(ucSequence << 1) == (uint8_t)(SEQUENCE_FIRST_MESSAGE << 1))
      {
         ucBurstSegment |= BURST_SEGMENT_START;
      }

      if (ucSequence & SEQUENCE_LAST_MESSAGE)
      {
         ucBurstSequence = 0;
         ucBurstSegment |= BURST_SEGMENT_END;
      }

      if (pstRxMessage->ANT_MESSAGE_ucMesgID == MESG_EXT_BURST_DATA_ID)
      {
         stCmdResp.ucResponse = (uint8_t)sd_ant_burst_handler_request(pstRxMessage->ANT_MESSAGE_ucChannel, pstRxMessage->ANT_MESSAGE_ucSize-(ANT_ID_SIZE + MESG_CHANNEL_NUM_SIZE), pstRxMessage->ANT_MESSAGE_aucPayload + ANT_ID_SIZE, ucBurstSegment);
         if (!stCmdResp.ucResponse)
            stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
      }
      else if (pstRxMessage->ANT_MESSAGE_ucMesgID == MESG_ADV_BURST_DATA_ID)
      {
         stCmdResp.ucResponse = (uint8_t)sd_ant_burst_handler_request(pstRxMessage->ANT_MESSAGE_ucChannel, (pstRxMessage->ANT_MESSAGE_ucSize-MESG_CHANNEL_NUM_SIZE), pstRxMessage->ANT_MESSAGE_aucPayload, ucBurstSegment);
         if (!stCmdResp.ucResponse)
            stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
      }
      else
      {
         stCmdResp.ucResponse = (uint8_t)sd_ant_burst_handler_request(pstRxMessage->ANT_MESSAGE_ucChannel, (pstRxMessage->ANT_MESSAGE_ucSize-MESG_CHANNEL_NUM_SIZE), pstRxMessage->ANT_MESSAGE_aucPayload, ucBurstSegment);
         if (!stCmdResp.ucResponse)
            stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
      }

      if (stCmdResp.ucResponse == NO_RESPONSE_MESSAGE) // input data successfully queued for processing, do not release receive buffer
      {
          return true;
      }
      else
         ucBurstSequence = 0; // reset sequence check, if failed
   }

   stCmdResp.ucChannel    = pstRxMessage->ANT_MESSAGE_ucChannel & CHANNEL_NUMBER_MASK;
   stCmdResp.ucResponseID = pstRxMessage->ANT_MESSAGE_ucMesgID;
   stCmdResp.bExtIDResponse = false;

   Command_ResponseMessage(stCmdResp, pstTxMessage); // send a response message if there is one and release the receive buffer
   return false;
}


/**
 * @brief ANT serial command message handler
 */
void Command_SerialMessageProcess(ANT_MESSAGE *pstRxMessage, ANT_MESSAGE *pstTxMessage)
{
   COMMAND_RESPONSE stCmdResp;
   uint16_t usExtID;
   uint16_t usTemp;

   // initialize a response message
   stCmdResp.ucChannel    = pstRxMessage->ANT_MESSAGE_ucChannel & CHANNEL_NUMBER_MASK; // save the channel number for the response
   stCmdResp.ucResponseID = pstRxMessage->ANT_MESSAGE_ucMesgID; // save the message ID
   stCmdResp.ucResponse   = RESPONSE_NO_ERROR; // initialize the response field to no error
   stCmdResp.bExtIDResponse = false; // by default, non-extended ID response

   if (pstTxMessage != NULL)
      pstTxMessage->ANT_MESSAGE_ucSize = 0;   // init the transmit message size to 0

   #if defined (USE_INTERFACE_LOCK)
      if (bInterfaceLock) // handles interface lockout condition
      {
         if (pstRxMessage->ANT_MESSAGE_ucMesgID == MESG_UNLOCK_INTERFACE_ID)
            bInterfaceLock = false;
         else
            stCmdResp.ucResponse = RETURN_TO_MFG; // standard response when interface locked out

         if (pstTxMessage != NULL)
            Command_ResponseMessage(stCmdResp, pstTxMessage);

         return;
      }
   #endif // USE_INTERFACE_LOCK


   if ((stCmdResp.ucChannel < MAX_ANT_CHANNELS) ||
       (MAX_NETWORKS > MAX_ANT_CHANNELS && pstRxMessage->ANT_MESSAGE_ucMesgID == MESG_NETWORK_KEY_ID) ||
       ((pstRxMessage->ANT_MESSAGE_ucMesgID & MSG_EXT_ID_MASK) == MSG_EXT_ID_MASK))
   {
      switch (stCmdResp.ucResponseID)
      {
         ///////////////////////////////////////////////////////////////////////
         // Data Messages
         ///////////////////////////////////////////////////////////////////////
         case MESG_EXT_BURST_DATA_ID:
            if ((pstRxMessage->ANT_MESSAGE_ucChannel & SEQUENCE_NUMBER_ROLLOVER) == SEQUENCE_FIRST_MESSAGE)
            {
               stCmdResp.ucResponse = (uint8_t)sd_ant_channel_id_set(stCmdResp.ucChannel,
                                                         ((uint16_t)pstRxMessage->ANT_MESSAGE_aucPayload[1] << 8) |  ((uint16_t)pstRxMessage->ANT_MESSAGE_aucPayload[0]), // only try to set the ID if this is the initial burst packet
                                                         pstRxMessage->ANT_MESSAGE_aucPayload[2],
                                                         pstRxMessage->ANT_MESSAGE_aucPayload[3]);

               if (stCmdResp.ucResponse != RESPONSE_NO_ERROR)
                  break; // break and respond with error if the ID was not set correctly
            }
            //fall thru
         case MESG_BURST_DATA_ID:
         case MESG_ADV_BURST_DATA_ID:
            Main_SetQueuedBurst();
            stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
            return;
         case MESG_EXT_ACKNOWLEDGED_DATA_ID:
            sd_ant_channel_id_set(stCmdResp.ucChannel,
                              ((uint16_t)pstRxMessage->ANT_MESSAGE_aucPayload[1] << 8) |  ((uint16_t)pstRxMessage->ANT_MESSAGE_aucPayload[0]),
                                 pstRxMessage->ANT_MESSAGE_aucPayload[2],
                                 pstRxMessage->ANT_MESSAGE_aucPayload[3]);
            stCmdResp.ucResponse = (uint8_t)sd_ant_acknowledge_message_tx(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_ucSize-(ANT_ID_SIZE+MESG_CHANNEL_NUM_SIZE), pstRxMessage->ANT_MESSAGE_aucPayload + ANT_ID_SIZE);
            if (!stCmdResp.ucResponse)
               stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
            break;
         case MESG_ACKNOWLEDGED_DATA_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_acknowledge_message_tx(stCmdResp.ucChannel, (pstRxMessage->ANT_MESSAGE_ucSize-MESG_CHANNEL_NUM_SIZE), pstRxMessage->ANT_MESSAGE_aucPayload);
            if (!stCmdResp.ucResponse)
               stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
            break;
         case MESG_EXT_BROADCAST_DATA_ID:
            sd_ant_channel_id_set(stCmdResp.ucChannel,
                              ((uint16_t)pstRxMessage->ANT_MESSAGE_aucPayload[1] << 8) |  ((uint16_t)pstRxMessage->ANT_MESSAGE_aucPayload[0]),
                              pstRxMessage->ANT_MESSAGE_aucPayload[2],
                              pstRxMessage->ANT_MESSAGE_aucPayload[3]);
            stCmdResp.ucResponse = (uint8_t)sd_ant_broadcast_message_tx(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_ucSize-(ANT_ID_SIZE+MESG_CHANNEL_NUM_SIZE), pstRxMessage->ANT_MESSAGE_aucPayload + ANT_ID_SIZE);
            if (!stCmdResp.ucResponse)
               stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
            break;
         case MESG_BROADCAST_DATA_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_broadcast_message_tx(stCmdResp.ucChannel, (pstRxMessage->ANT_MESSAGE_ucSize-MESG_CHANNEL_NUM_SIZE), pstRxMessage->ANT_MESSAGE_aucPayload);
            if (!stCmdResp.ucResponse)
               stCmdResp.ucResponse = NO_RESPONSE_MESSAGE;
            break;

         ///////////////////////////////////////////////////////////////////////
         // Request Messages
         ///////////////////////////////////////////////////////////////////////
         case MESG_REQUEST_ID:
            if (pstTxMessage == NULL) // don't allow request messages if no transmit buffer provided
               break;

            pstTxMessage->ANT_MESSAGE_ucChannel = stCmdResp.ucChannel;

            switch (pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1])
            {
               case MESG_CHANNEL_STATUS_ID:
                  stCmdResp.ucResponse = sd_ant_channel_status_get(stCmdResp.ucChannel, &pstTxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize   = MESG_CHANNEL_STATUS_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_CHANNEL_STATUS_ID;
                     // the channel number field has already been filled in
                  }
                  break;

               case MESG_CHANNEL_ID_ID:
                  stCmdResp.ucResponse = sd_ant_channel_id_get(stCmdResp.ucChannel,
                                                              (uint16_t*)&usTemp,
                                                              (uint8_t*)&pstTxMessage->ANT_MESSAGE_aucPayload[2],
                                                              (uint8_t*)&pstTxMessage->ANT_MESSAGE_aucPayload[3]);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize   = MESG_CHANNEL_ID_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_CHANNEL_ID_ID;
                     // the channel number field has already been filled in

                     pstTxMessage->ANT_MESSAGE_aucPayload[0] = (uint8_t)usTemp;
                     pstTxMessage->ANT_MESSAGE_aucPayload[1] = (uint8_t)(usTemp >> 8);
                  }
                  break;

               case MESG_CAPABILITIES_ID:
                  stCmdResp.ucResponse = sd_ant_capabilities_get(pstTxMessage->ANT_MESSAGE_aucMesgData);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize = MESG_CAPABILITIES_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_CAPABILITIES_ID;
                     //serial number support is provided by the application level, insert if application supports it
                     pstTxMessage->ANT_MESSAGE_aucMesgData[3] |= CAPABILITIES_SERIAL_NUMBER_ENABLED;
                  #if defined (ADD_BLE_CHANNEL_TO_ANT_CHANNELS)
                     pstTxMessage->ANT_MESSAGE_aucMesgData[0] += 1; // add BLE channel to number of ANT channels
                  #endif // ADD_BLE_CHANNEL_TO_ANT_CHANNELS
                  }
                  break;

               case MESG_VERSION_ID:
                  /*Returns Application version number*/
                  if (pstTxMessage->ANT_MESSAGE_ucChannel == 0)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize = 11;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_VERSION_ID;
                     memcpy(pstTxMessage->ANT_MESSAGE_aucMesgData, acAppVersion, APP_VERSION_SIZE);
                  }
                  /*Returns Softdevice version number*/
                  else if (pstTxMessage->ANT_MESSAGE_ucChannel == 1)
                  {
                     stCmdResp.ucResponse = sd_ant_version_get(pstTxMessage->ANT_MESSAGE_aucMesgData);
                     if (!stCmdResp.ucResponse)
                     {
                        pstTxMessage->ANT_MESSAGE_ucSize = 11;
                        pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_VERSION_ID;
                     }
                  }
                  /*Chan Number must only be 0=App or 1=Softdevice*/
                  else
                  {
                     stCmdResp.ucResponse = INVALID_MESSAGE;
                  }
                  break;


            #if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
               case MESG_GET_SERIAL_NUM_ID:
                  System_GetSerialNumberMesg(pstTxMessage);
                  break;
            #endif // !SERIAL_NUMBER_NOT_AVAILABLE

               case MESG_CONFIG_ADV_BURST_ID:
                  stCmdResp.ucResponse = sd_ant_adv_burst_config_get(stCmdResp.ucChannel, pstTxMessage->ANT_MESSAGE_aucMesgData);
                  if (!stCmdResp.ucResponse)
                  {
                     if (stCmdResp.ucChannel)
                        pstTxMessage->ANT_MESSAGE_ucSize = MESG_CONFIG_ADV_BURST_REQ_CONFIG_SIZE;
                     else
                        pstTxMessage->ANT_MESSAGE_ucSize = MESG_CONFIG_ADV_BURST_REQ_CAPABILITIES_SIZE;

                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_CONFIG_ADV_BURST_ID;
                  }
                  break;

               case MESG_COEX_PRIORITY_CONFIG_ID:
                  stCmdResp.ucResponse = sd_ant_coex_config_get(stCmdResp.ucChannel, pstTxMessage->ANT_MESSAGE_aucPayload, NULL);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize = MESG_COEX_PRIORITY_CONFIG_REQ_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_COEX_PRIORITY_CONFIG_ID;
                  }
                  break;

               case MESG_COEX_ADV_PRIORITY_CONFIG_ID:
                  stCmdResp.ucResponse = sd_ant_coex_config_get(stCmdResp.ucChannel, NULL, pstTxMessage->ANT_MESSAGE_aucPayload);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize = MESG_COEX_ADV_PRIORITY_CONFIG_REQ_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_COEX_ADV_PRIORITY_CONFIG_ID;
                  }
               break;


               case MESG_ACTIVE_SEARCH_SHARING_ID:
                  stCmdResp.ucResponse = sd_ant_active_search_sharing_cycles_get(stCmdResp.ucChannel, &pstTxMessage->ANT_MESSAGE_aucPayload[0]);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize = MESG_ACTIVE_SEARCH_SHARING_REQ_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_ACTIVE_SEARCH_SHARING_ID;
                  }
                  break;


               case MESG_EVENT_FILTER_CONFIG_ID:
                  stCmdResp.ucResponse = sd_ant_event_filtering_get((uint16_t*)&usTemp);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize = MESG_EVENT_FILTER_CONFIG_REQ_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_EVENT_FILTER_CONFIG_ID;

                     pstTxMessage->ANT_MESSAGE_aucPayload[0] = (uint8_t)usTemp;
                     pstTxMessage->ANT_MESSAGE_aucPayload[1] = (uint8_t)(usTemp >> 8);
                  }
                  break;

               case MESG_SDU_SET_MASK_ID:
                  stCmdResp.ucResponse = sd_ant_sdu_mask_get(stCmdResp.ucChannel, pstTxMessage->ANT_MESSAGE_aucPayload);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_SDU_SET_MASK_ID;
                     pstTxMessage->ANT_MESSAGE_ucSize = MESG_ANT_MAX_PAYLOAD_SIZE + MESG_CHANNEL_NUM_SIZE;
                  }
                  break;


               case MESG_ENCRYPT_ENABLE_ID:
                  stCmdResp.ucResponse = sd_ant_crypto_info_get(stCmdResp.ucChannel, pstTxMessage->ANT_MESSAGE_aucPayload);
                  if (!stCmdResp.ucResponse)
                  {
                     switch (stCmdResp.ucChannel)
                     {
                        case ENCRYPTION_INFO_GET_SUPPORTED_MODE:
                           pstTxMessage->ANT_MESSAGE_ucSize = MESG_CONFIG_ENCRYPT_REQ_CAPABILITIES_SIZE;
                           break;
                        case ENCRYPTION_INFO_GET_CRYPTO_ID:
                           pstTxMessage->ANT_MESSAGE_ucSize = MESG_CONFIG_ENCRYPT_REQ_CONFIG_ID_SIZE;
                           break;
                        case ENCRYPTION_INFO_GET_CUSTOM_USER_DATA:
                           pstTxMessage->ANT_MESSAGE_ucSize = MESG_CONFIG_ENCRYPT_REQ_CONFIG_USER_DATA_SIZE;
                           break;
                     }

                     pstTxMessage->ANT_MESSAGE_ucChannel = stCmdResp.ucChannel;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_ENCRYPT_ENABLE_ID;
                  }
                  break;

               case MESG_RFACTIVE_NOTIFICATION_ID:
                  stCmdResp.ucResponse = sd_ant_rfactive_notification_config_get((uint8_t*)&pstTxMessage->ANT_MESSAGE_aucPayload[0], (uint16_t*)&usTemp);
                  if (!stCmdResp.ucResponse)
                  {
                     pstTxMessage->ANT_MESSAGE_ucSize = MESG_RFACTIVE_NOTIFICATION_SIZE;
                     pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_RFACTIVE_NOTIFICATION_ID;

                     pstTxMessage->ANT_MESSAGE_aucPayload[1] = (uint8_t)usTemp;
                     pstTxMessage->ANT_MESSAGE_aucPayload[2] = (uint8_t)(usTemp >> 8);
                  }
                  break;


               default:
                  stCmdResp.ucResponse = INVALID_MESSAGE;
                  break;
            }
            break;

         ///////////////////////////////////////////////////////////////////////
         // Command Messages
         ///////////////////////////////////////////////////////////////////////
         case MESG_ASSIGN_CHANNEL_ID:
            if (pstRxMessage->ANT_MESSAGE_ucSize > MESG_ASSIGN_CHANNEL_SIZE )
            {
               stCmdResp.ucResponse = (uint8_t)sd_ant_channel_assign(stCmdResp.ucChannel,
                                             pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1],
                                             pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2],
                                             pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_3]);
            }
            else
            {
               stCmdResp.ucResponse = (uint8_t)sd_ant_channel_assign(stCmdResp.ucChannel,
                                             pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1],
                                             pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2],
                                             0);
            }

            break;

         case MESG_UNASSIGN_CHANNEL_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_channel_unassign(stCmdResp.ucChannel);
            break;

         case MESG_OPEN_CHANNEL_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_channel_open(stCmdResp.ucChannel);
            break;

         case MESG_CLOSE_CHANNEL_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_channel_close(stCmdResp.ucChannel);
            break;

         case MESG_CHANNEL_ID_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_channel_id_set(stCmdResp.ucChannel,
                        DSI_GetUShort(pstRxMessage->ANT_MESSAGE_aucPayload),
                                 pstRxMessage->ANT_MESSAGE_aucPayload[2],
                                 pstRxMessage->ANT_MESSAGE_aucPayload[3]);
            break;

         case MESG_CHANNEL_MESG_PERIOD_ID:
            sd_ant_channel_period_set(stCmdResp.ucChannel, DSI_GetUShort(pstRxMessage->ANT_MESSAGE_aucPayload));
            break;

         case MESG_PROX_SEARCH_CONFIG_ID:
            sd_ant_prox_search_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;

         case MESG_CHANNEL_RADIO_FREQ_ID:
            sd_ant_channel_radio_freq_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;

         case MESG_RADIO_TX_POWER_ID:
            {
               uint8_t i;

               if (pstRxMessage->ANT_MESSAGE_ucSize > MESG_RADIO_TX_POWER_SIZE )
               {
                  for (i=0; i<MAX_ANT_CHANNELS; i++)
                     sd_ant_channel_radio_tx_power_set(i, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2]);
               }
               else
               {
                  for (i=0; i<MAX_ANT_CHANNELS; i++)
                     sd_ant_channel_radio_tx_power_set(i, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], 0);
               }
            }
            break;

         case MESG_CHANNEL_RADIO_TX_POWER_ID:
            if (pstRxMessage->ANT_MESSAGE_ucSize > MESG_CHANNEL_RADIO_TX_POWER_SIZE )
               sd_ant_channel_radio_tx_power_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2]);
            else
               sd_ant_channel_radio_tx_power_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], 0);
            break;

         case MESG_CHANNEL_SEARCH_TIMEOUT_ID:
            sd_ant_channel_rx_search_timeout_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;

         case MESG_SEARCH_WAVEFORM_ID:
            sd_ant_search_waveform_set(stCmdResp.ucChannel, DSI_GetUShort(pstRxMessage->ANT_MESSAGE_aucPayload));
            break;

         case MESG_NETWORK_KEY_ID:
            if (stCmdResp.ucChannel < MAX_NETWORKS)
               sd_ant_network_address_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload);
            else
               stCmdResp.ucResponse = INVALID_MESSAGE;
            break;

         case MESG_ANTLIB_CONFIG_ID:
            sd_ant_lib_config_clear(ANT_LIB_CONFIG_MASK_ALL);
            stCmdResp.ucResponse = (uint8_t)sd_ant_lib_config_set(ANT_LIB_CONFIG_RADIO_CONFIG_ALWAYS|pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;

         case MESG_RX_EXT_MESGS_ENABLE_ID:
            if (pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1])
               stCmdResp.ucResponse = (uint8_t)sd_ant_lib_config_set(ANT_LIB_CONFIG_MESG_OUT_INC_DEVICE_ID);
            else
               sd_ant_lib_config_clear(ANT_LIB_CONFIG_MESG_OUT_INC_DEVICE_ID);
            break;

         case MESG_RADIO_CW_INIT_ID:
            sd_ant_cw_test_mode_init();
            break;

         case MESG_RADIO_CW_MODE_ID:
            if (pstRxMessage->ANT_MESSAGE_ucSize > MESG_RADIO_CW_MODE_SIZE)
               sd_ant_cw_test_mode(pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_3]);
            else
               sd_ant_cw_test_mode(pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], 0);
            break;

         case MESG_SYSTEM_RESET_ID:
            stCmdResp.ucResponse = System_Reset(RESET_CMD);
            break;

         case MESG_SLEEP_ID:
            stCmdResp.ucResponse = System_SetDeepSleep(pstRxMessage);

            break;

         case MESG_ID_LIST_ADD_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_id_list_add(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload, *(pstRxMessage->ANT_MESSAGE_aucPayload + 4));
            break;

         case MESG_ID_LIST_CONFIG_ID:
            stCmdResp.ucResponse =(uint8_t) sd_ant_id_list_config(stCmdResp.ucChannel, *pstRxMessage->ANT_MESSAGE_aucPayload, *(pstRxMessage->ANT_MESSAGE_aucPayload + 1));
            break;

         case MESG_OPEN_RX_SCAN_ID:
            if (pstRxMessage->ANT_MESSAGE_ucSize != MESG_OPEN_RX_SCAN_SIZE)
               stCmdResp.ucResponse = (uint8_t)sd_ant_rx_scan_mode_start(0);
            else
               stCmdResp.ucResponse = (uint8_t)sd_ant_rx_scan_mode_start(pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;


         case MESG_SET_LP_SEARCH_TIMEOUT_ID:
            sd_ant_channel_low_priority_rx_search_timeout_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;

        #if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
         case MESG_SERIAL_NUM_SET_CHANNEL_ID_ID:
            stCmdResp.ucResponse = SetChannelToSerialID(pstRxMessage);
            break;
        #endif // !SERIAL_NUMBER_NOT_AVAILABLE

         case MESG_SET_SEARCH_CH_PRIORITY_ID:
            sd_ant_search_channel_priority_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;


         case MESG_AUTO_FREQ_CONFIG_ID:
            sd_ant_auto_freq_hop_table_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2],
                                     pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_3]);

            break;

         case MESG_CONFIG_ADV_BURST_ID:
            stCmdResp.ucResponse = (uint8_t)sd_ant_adv_burst_config_set(pstRxMessage->ANT_MESSAGE_aucPayload, pstRxMessage->ANT_MESSAGE_ucSize - 1);
            break;

         case MESG_COEX_PRIORITY_CONFIG_ID:
            sd_ant_coex_config_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload, NULL);
            break;

         case MESG_COEX_ADV_PRIORITY_CONFIG_ID:
            sd_ant_coex_config_set(stCmdResp.ucChannel, NULL, pstRxMessage->ANT_MESSAGE_aucPayload);
            break;



         case MESG_EVENT_FILTER_CONFIG_ID:
            sd_ant_event_filtering_set(DSI_GetUShort(&(pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1])));
            break;

         case MESG_ACTIVE_SEARCH_SHARING_ID:
            sd_ant_active_search_sharing_cycles_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;

         case MESG_SDU_CONFIG_ID:
            stCmdResp.ucResponse = sd_ant_sdu_mask_config(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1]);
            break;

         case MESG_SDU_SET_MASK_ID:
            stCmdResp.ucResponse = sd_ant_sdu_mask_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload);
            break;


         case MESG_ENCRYPT_ENABLE_ID:
            stCmdResp.ucResponse = sd_ant_crypto_channel_enable(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2], pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_3]);
            break;

         case MESG_SET_ENCRYPT_KEY_ID:
            stCmdResp.ucResponse = sd_ant_crypto_key_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload);
            break;

         case MESG_SET_ENCRYPT_INFO_ID:
            stCmdResp.ucResponse = sd_ant_crypto_info_set(stCmdResp.ucChannel, pstRxMessage->ANT_MESSAGE_aucPayload);
            break;


         case MESG_RFACTIVE_NOTIFICATION_ID:
            stCmdResp.ucResponse = sd_ant_rfactive_notification_config_set(pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1], DSI_GetUShort(&(pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2])));
            break;

         ///////////////////////////////////////////////////////////////////////
         // Extended ID Messages
         ///////////////////////////////////////////////////////////////////////
         case MESG_EXT_ID_0:
         case MESG_EXT_ID_1:
         case MESG_EXT_ID_2:
         {
            bool bInvalidMessage = false;

            usExtID = (((uint16_t)(pstRxMessage->ANT_MESSAGE_ucMesgID)) << 8) | (pstRxMessage->ANT_MESSAGE_ucSubID);

            stCmdResp.ucResponseID = pstRxMessage->ANT_MESSAGE_ucMesgID;
            stCmdResp.ucResponseSubID = pstRxMessage->ANT_MESSAGE_ucSubID;
            stCmdResp.ucChannel = 0;

            switch (usExtID)
            {
               /////////////////////////
               case MESG_EXT_REQUEST_ID:
               {

                  //
                  usExtID =(((uint16_t)(pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1])) << 8) | (pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2]);
                  pstTxMessage->ANT_MESSAGE_ucMesgID = (uint8_t) (usExtID >> 8);
                  pstTxMessage->ANT_MESSAGE_ucSubID = (uint8_t) usExtID;

                  switch(usExtID)
                  {
                     default:
                        bInvalidMessage = true;
                        break;
                  }

                  if (stCmdResp.ucResponse)
                  {
                     stCmdResp.ucResponseID = pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_1];
                     stCmdResp.ucResponseSubID = pstRxMessage->ANT_MESSAGE_aucPayload[SERIAL_DATA_OFFSET_2];
                     stCmdResp.bExtIDResponse = true;
                  }
                  //

                  break;
               } // case MESG_EXT_REQUEST_ID

               default:
                  bInvalidMessage = true;
                  break;

            }

            if (bInvalidMessage)
               stCmdResp.ucResponse = INVALID_MESSAGE;
            else
               stCmdResp.bExtIDResponse = true;
            //
            //

            break;
         }


         default:
            stCmdResp.ucResponse = INVALID_MESSAGE;
            break;
      }
   }
   else
   {
      stCmdResp.ucResponse = INVALID_MESSAGE;
   }

   if (stCmdResp.ucResponse == NRF_ERROR_NOT_SUPPORTED)
      stCmdResp.ucResponse = INVALID_MESSAGE;

   if (pstTxMessage == NULL) // don't send a response if no buffer provided and don't release RTS
      return;

   Command_ResponseMessage(stCmdResp, pstTxMessage);
}

/**
 * @brief ANT serial command response handler
 */
void Command_ResponseMessage(COMMAND_RESPONSE stCmdResp, ANT_MESSAGE *pstTxMessage)
{

   if (!pstTxMessage->ANT_MESSAGE_ucSize)
   {
      if (stCmdResp.bExtIDResponse) // currently all extended ID messages require a response, this must be changed if that changes
      {
         pstTxMessage->ANT_MESSAGE_ucSize = 4;
         pstTxMessage->ANT_MESSAGE_ucMesgID = (uint8_t)(MESG_EXT_RESPONSE_ID >> 8);
         pstTxMessage->ANT_MESSAGE_ucSubID = (uint8_t)(MESG_EXT_RESPONSE_ID);
         pstTxMessage->ANT_MESSAGE_aucPayload[0] = stCmdResp.ucResponseID;
         pstTxMessage->ANT_MESSAGE_aucPayload[1] = stCmdResp.ucResponseSubID;
         pstTxMessage->ANT_MESSAGE_aucPayload[2] = stCmdResp.ucResponse;
      }
      else if (stCmdResp.ucResponse != NO_RESPONSE_MESSAGE) // check if we need to create a default response
      {
         pstTxMessage->ANT_MESSAGE_ucSize = MESG_RESPONSE_EVENT_SIZE; // send a response message
         pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_RESPONSE_EVENT_ID; // set the response ID
         pstTxMessage->ANT_MESSAGE_ucChannel = stCmdResp.ucChannel; // set the channel number
         pstTxMessage->ANT_MESSAGE_aucPayload[SERIAL_RESPONSE_ID_OFFSET]= stCmdResp.ucResponseID; // set the mesg ID we are responding too
         pstTxMessage->ANT_MESSAGE_aucPayload[SERIAL_RESPONSE_OFFSET]= stCmdResp.ucResponse; // set the response code
      }
   }

   
#ifdef MPG
   /* Parse response messages to pick out certain messages to which LEDs will respond:

                                                      YELLOW    GREEN     BLUE
   START STATE                                          ON       OFF      OFF
   
MESG_RESPONSE_EVENT_ID (0x40)
   MESG_ASSIGN_CHANNEL_ID > RESPONSE_NO_ERROR          OFF       ON        x
   MESG_UNASSIGN_CHANNEL_ID > RESPONSE_NO_ERROR        ON        OFF      OFF
   MESG_OPEN_CHANNEL_ID > RESPONSE_NO_ERROR            OFF       ON       ON
   EVENT_CHANNEL_CLOSED                                x         ON       OFF
   
   x = existing state

   LEDs for reference:
   NRF_GPIO->OUTSET = (P0_29_LED_RED | P0_28_LED_YLW | P0_27_LED_GRN | P0_26_LED_BLU);
   NRF_GPIO->OUTCLR = (P0_29_LED_RED | P0_28_LED_YLW | P0_27_LED_GRN | P0_26_LED_BLU);
   */
    
   /* Channel Event */
   if(stCmdResp.ucResponseID == MESG_EVENT_ID)
   {
     if(stCmdResp.ucResponseSubID == EVENT_CHANNEL_CLOSED)
     {
        NRF_GPIO->OUTCLR = (P0_26_LED_BLU);
     }
   }
   else
   {
     switch(stCmdResp.ucResponseID)
     {
       case(MESG_ASSIGN_CHANNEL_ID):
       {
         if(stCmdResp.ucResponseSubID == RESPONSE_NO_ERROR)
         {
           NRF_GPIO->OUTSET = (P0_27_LED_GRN);
           NRF_GPIO->OUTCLR = (P0_28_LED_YLW);
         }
       }
         
       case(MESG_UNASSIGN_CHANNEL_ID):
       {
         if(stCmdResp.ucResponseSubID == RESPONSE_NO_ERROR)
         {
           NRF_GPIO->OUTCLR = (P0_27_LED_GRN);
           NRF_GPIO->OUTSET = (P0_28_LED_YLW);
         }
       }

       case(MESG_OPEN_CHANNEL_ID):
       {
         if(stCmdResp.ucResponseSubID == RESPONSE_NO_ERROR)
         {
           NRF_GPIO->OUTSET = (P0_26_LED_BLU);
         }
       }

     default:
       {
         /* No action */
       }
     } /* end switch */
   }

#endif /* MPG */
   
   Serial_ReleaseRx(); // release the serial receive buffer
}

#if defined (USE_INTERFACE_LOCK)
/**
 * @brief ANT serial command interface lock
 */
void Command_SetInterfaceLock(bool bLock)
{
   bInterfaceLock = bLock;
}
#endif // USE_INTERFACE_LOCK

#if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
static uint8_t SetChannelToSerialID(ANT_MESSAGE *pstRxMessage)
{
   System_GetSerialNumber(&(pstRxMessage->ANT_MESSAGE_aucPayload[2]));                    // use a free part of the read buffer as scratch pad

   if (!pstRxMessage->ANT_MESSAGE_aucPayload[2] && !pstRxMessage->ANT_MESSAGE_aucPayload[3]) // if bottom 2 bytes of serial number are 0
      pstRxMessage->ANT_MESSAGE_aucPayload[2] = 1;                                        // set device number to 1

   pstRxMessage->ANT_MESSAGE_aucPayload[4] = pstRxMessage->ANT_MESSAGE_aucPayload[0];     // copy in the provided Device Type
   pstRxMessage->ANT_MESSAGE_aucPayload[5] = pstRxMessage->ANT_MESSAGE_aucPayload[1];     // copy in the provided Transmission Type

   return (uint8_t)sd_ant_channel_id_set(pstRxMessage->ANT_MESSAGE_ucChannel,
                                         DSI_GetUShort(&pstRxMessage->ANT_MESSAGE_aucPayload[2]),
                                         pstRxMessage->ANT_MESSAGE_aucPayload[4],
                                         pstRxMessage->ANT_MESSAGE_aucPayload[5]);        // write the full channel ID to ANT channel
}
#endif // !SERIAL_NUMBER_NOT_AVAILABLE
