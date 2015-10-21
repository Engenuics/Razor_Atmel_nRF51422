 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#ifndef DSI_UTILITY_H
#define DSI_UTILITY_H

#include <stdbool.h>
#include <stdint.h>
#include "appconfig.h"

/**
 * @brief Read unsigned 16-bit from buffer (little endian)
 */
uint16_t DSI_GetUShort(uint8_t *pucData);

/**
 * @brief Read unsigned 32-bit from buffer (little endian)
 */
uint32_t DSI_GetULong(uint8_t *pucData);

/**
 * @brief Buffer copy utility
 */
void DSI_memcpy(uint8_t *pucDest, uint8_t *pucSrc, uint8_t ucSize);

/**
 * @brief Buffer set utility
 */
void DSI_memset(uint8_t *pucDest, uint8_t ucValue, uint8_t ucSize);

/**
 * @brief Buffer compare utility
 */
bool DSI_memcmp(uint8_t *pucSrc1, uint8_t *pucSrc2, uint8_t ucSize);


#endif // DSI_UTILITY_H
