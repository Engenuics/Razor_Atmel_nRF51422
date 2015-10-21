 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdbool.h>
#include <stdint.h>
#include "appconfig.h"

/*
 *  Global control flags: Must be accessed/changed atomically as they can be accessed by multiple contexts
 *  If bitfields are used, the entire bitfield operation must be atomic!!
 */
extern volatile bool bEventRXSerialMessageProcess;
extern volatile bool bEventANTProcessStart;
extern volatile bool bEventANTProcess;
extern volatile bool bEventBurstMessageProcess;
extern volatile bool bEventStartupMessage;
extern volatile bool bQueuedTxBurst;


extern uint8_t ucBurstSequence;

#define APP_VERSION_SIZE    11
extern const char acAppVersion[];

#endif // GLOBAL_H
