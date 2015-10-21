 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

/*
 * NOTES:
 *
 * version "AAA#.##B##"
 *
 * SW_VER_MAJOR   - Increases on any released applicaion major feature update/changes or new features
 * SW_VER_MINOR   - Increases on any release application minor feature update i.e. Bug fixing and minor features.
 * SW_VER_PREFIX  - Is fixed on this firmware.
 * SW_VER_POSTFIX - Increases on any internal development builds. OR might be used for tagging special builds. OR might be used on branch build
  */



   #define     SW_VER_MAJOR      "0."
   #define     SW_VER_MINOR      "02"

   #define     SW_VER_PREFIX     "BDD" // NRF51 as ANT Network processor
   #define     SW_VER_POSTFIX    "B01"

/***************************************************************************
*/
const char acAppVersion[] = SW_VER_PREFIX SW_VER_MAJOR SW_VER_MINOR SW_VER_POSTFIX;  // Max 11 characters including null
