 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#include "dsi_utility.h"

#include <stdbool.h>
#include <stdint.h>

#include "appconfig.h"

typedef union
{
   uint16_t usData;
   struct
   {
      uint8_t ucLow;
      uint8_t ucHigh;
   } stBytes;
} uint16_t_UNION;

typedef union
{
   uint8_t aucData[4];
   uint32_t ulData;
   struct
   {
      // The least significant byte of the uint32_t in this structure is
      // referenced by ucByte0.
      uint8_t ucByte0;
      uint8_t ucByte1;
      uint8_t ucByte2;
      uint8_t ucByte3;
   } stBytes;
} uint32_t_UNION;

/**
 * @brief Read unsigned 16-bit from buffer (little endian)
 */
uint16_t DSI_GetUShort(uint8_t *pucData)
{
   uint16_t_UNION stData;

   stData.stBytes.ucLow  = *pucData++;
   stData.stBytes.ucHigh = *pucData;

   return stData.usData;
}

/**
 * @brief Read unsigned 32-bit from buffer (little endian)
 */
uint32_t DSI_GetULong(uint8_t *pucData)
{
   uint32_t_UNION stData;

   stData.stBytes.ucByte0 = *pucData++;
   stData.stBytes.ucByte1 = *pucData++;
   stData.stBytes.ucByte2 = *pucData++;
   stData.stBytes.ucByte3 = *pucData;

   return stData.ulData;
}

/**
 * @brief Buffer copy utility
 */
void DSI_memcpy(uint8_t *pucDest, uint8_t *pucSrc, uint8_t ucSize)
{
   while (ucSize--)
      *(pucDest++) = *(pucSrc++);
}

/**
 * @brief Buffer set utility
 */
void DSI_memset(uint8_t *pucDest, uint8_t ucValue, uint8_t ucSize)
{
   while (ucSize--)
      *(pucDest++) = ucValue;
}

/**
 * @brief Buffer compare utility
 */
bool DSI_memcmp(uint8_t *pucSrc1, uint8_t *pucSrc2, uint8_t ucSize)
{
   while (ucSize--)
   {
      if (*(pucSrc1++) != (*(pucSrc2++)))
         return 1; // no match
   }

   return 0; // match, replicate memcmp return behaviour
}

