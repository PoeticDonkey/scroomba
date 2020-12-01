/** @file limit_switch_back.h
 *      This file contains a task that runs the back limit switch.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include "limit_switch_back.h"

extern Queue<uint8_t> limitdetect_back;

/** @brief   Task which handles the back limit switches
 *  @details This task initializes the pins and raises a flag for
 *           Mastermind if the back limit switches were pressed.
 *  @param   p_params A pointer to function parameters which we don't use.
 */


void task_limit_back (void* p_params)
{
    (void)p_params;             // Does nothing but shut up a compiler warning  
    
    const uint8_t pin = D9;     // pin address
    pinMode(pin,INPUT);         // enable the pin as an input

    for (;;)
    {
        //checks if limit switches are pressed
        if (limitdetect_back.any())
        {
            vTaskDelay(500);
        }
        else if (digitalRead(pin))      //If the pin is high, then limit switch detected a boundary
        {
            limitdetect_back.put(0); // put dir stop value in queue
        }
        vTaskDelay(100); // Delays things so we can actually see stuff happening
    }
}