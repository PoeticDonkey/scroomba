/** @file limit_switch_front.cpp
 *      This file contains a task that runs the front limit switch.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include "limit_switch_front.h"

extern Queue<uint8_t> limitdetect_front;

/** @brief   Task which handles the front limit switch
 *  @details This task initializes the pins and raises a flag for
 *           Mastermind if the front limit switches were pressed.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_limit_front (void* p_params)
{
    (void)p_params;             // Does nothing but shut up a compiler warning  
    
    const uint8_t pin = D8;     // pin address
    pinMode(pin,INPUT);         // enable the pin as an input

    for (;;)
    {
        //checks if front limit switch is pressed
        if (limitdetect_front.any())
        {
            vTaskDelay(500); // do nothing until mastermind clears it
        }
        else if (digitalRead(pin))      //If the pin is high, then limit switch detected a boundary
        {
            limitdetect_front.put(2); // put dir back value in queue
        }
        vTaskDelay(50); // Delays things as needed
    }
}