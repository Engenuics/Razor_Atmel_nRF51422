 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#include "global.h"

#include <stdbool.h>
#include <stdint.h>

#include "appconfig.h"

/*
 * Global control flags: Must be accessed/changed atomically as they can be accessed by multiple contexts
 * If bitfields are used, the entire bitfield operation must be atomic!!
 */
volatile bool bEventRXSerialMessageProcess;
volatile bool bEventANTProcessStart;
volatile bool bEventANTProcess;
volatile bool bEventBurstMessageProcess;
volatile bool bEventStartupMessage;
volatile bool bQueuedTxBurst;


uint8_t ucBurstSequence;
