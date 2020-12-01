/** @file motor.cpp
 *      This file contains a task that runs the motors.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include <Arduino.h>
#include <PrintStream.h>
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif
#include <Wire.h>
#include "taskqueue.h"


void task_motor (void* p_params); // the task function
