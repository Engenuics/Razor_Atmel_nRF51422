/*
 * Copyright (c) 2011 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */
 /**
  @addtogroup app_svc_example
  @{


  @brief Demo Application SVC implementation

*/

/** @brief Application SVC Handler Implementation.
 *
 * This will only receive system calls where 0x00 <= SVC Number < SDM_SVC_BASE.
 * All other numbers are used by the nRF SoftDevice
 *
 * @param svc_number The SVC number
 * @param *svc_args List of up to four parameter values
 */
void C_SVC_Handler(unsigned int svc_number, unsigned int *svc_args)
{
  switch (svc_number)
  {
    default:
      break;
  }
}

/**
 @}
 */
