/*
 * Copyright (c) 2011 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */
 /**
  @addtogroup demo_application_dd
  @{
  @defgroup app_svc_example Application System Call Example
  @{


  @brief Demo App SVC Interface

This interface is used to demonstrate that the application can implement System
calls for its own purposes.  This will be used by an application for execution
of a RTOS scheduler or other code which must be executed in a fixed priority.

@note System calls in the application are not threadsafe with Application(High)
interrupt priority level functions and therefore an RTOS implementation must not
allow threads to run in Application(High) priority.
*/

#ifndef APP_SVC_H__
#define APP_SVC_H__

#endif // APP_SVC_H__

/**
 @}
 @}
 */
