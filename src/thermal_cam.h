/** @file thermal_cam.h
 *      This file contains a task that collects data from the thermal camera.
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
#include <Adafruit_AMG88xx.h>
#include "taskqueue.h"

void task_thermal (void* p_params); // the task function
