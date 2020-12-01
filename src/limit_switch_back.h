/** @file limit_switch_back.h
 *      This file contains a task that runs the back limit switch.
 * 
 *  @brief  Operates back limit switch(es) to detect if limit switch pressed. Raises flag if it is pressed.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include "Arduino.h"
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif
#include "PrintStream.h"
#include <Wire.h>
#include "taskqueue.h"

void task_limit_back (void* p_params); // the task function
