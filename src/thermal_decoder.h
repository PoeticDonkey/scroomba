/** @file thermal_decoder.h
 *      This file contains a task that decodes information from the thermal camera 
 *      to find and track a person for the Scroomba robot.
 * 
 *  @brief Task that analyzes the thermal camera temperature array for people, sends result to mastermind.
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

void task_thermaldecoder (void* p_params); // the task function


