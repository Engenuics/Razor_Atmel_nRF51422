 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#ifndef COMMAND_H
#define COMMAND_H

#include <stdbool.h>
#include <stdint.h>
#include "ant_parameters.h"
#include "appconfig.h"

typedef struct
{
   uint8_t ucChannel;
   uint8_t ucResponseID;
   uint8_t ucResponseSubID;
   uint8_t ucResponse;
   bool bExtIDResponse;

} COMMAND_RESPONSE;

/**
 * @brief ANT serial burst command message handler
 */
bool Command_BurstMessageProcess(ANT_MESSAGE *pstRxMessage, ANT_MESSAGE *pstTxMessage);


/**
 * @brief ANT serial command message handler
 */
void Command_SerialMessageProcess(ANT_MESSAGE *pstRxMessage, ANT_MESSAGE *pstTxMessage);

/**
 * @brief ANT serial command response handler
 */
void Command_ResponseMessage(COMMAND_RESPONSE stCmdResponse, ANT_MESSAGE *pstTxMessage);

#if defined (USE_INTERFACE_LOCK)
/**
 * @brief ANT serial command interface lock
 */
void Command_SetInterfaceLock(BOOL bLock);
#endif // USE_INTERFACE_LOCK

#endif // COMMAND_H
