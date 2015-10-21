 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "ant_parameters.h"

/**
 * @brief Set application queued burst mode
 */
void Main_SetQueuedBurst(void);

/**
 * @brief Application debug command handler
 */
uint8_t Main_DebugMsgProcess(uint8_t *aucPayload, ANT_MESSAGE *pstTxMessage);

#endif // MAIN_H
