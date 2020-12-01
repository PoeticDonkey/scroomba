/** @file main.cpp
 *    This file contains the code to run the Scoomba.
 *
 *  @author  JR Ridgely
 *  @author  Scott Mangin
 *  @author  Nicholas Holman
 *  @author  Michael Conn
 * 
 *  @date   2020-Nov-12 Original file
 */

#include <Arduino.h>
#include <PrintStream.h>
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "taskqueue.h"
#include "taskshare.h"
#include "limit_switch_back.h"
#include "limit_switch_front.h"
#include "motor.h"
#include "thermal_cam.h"
#include "thermal_decoder.h"

Queue<float> thermaldata (640, "Thermal Data"); //Thermal Camera Data Queue
Queue<uint8_t> motordirection (1, "Motor Task Parameters"); // What motors should do queue
Queue<uint8_t> motorpower (1, "Motor Task Parameters"); // What motors should do queue
Queue<uint8_t> limitdetect_back (1, "Back Limit Switch Detection Flag"); // Back Limit Switch Flag
Queue<uint8_t> limitdetect_front (1, "Front Limit Switch Detection Flag"); // Front Limit Switch Flag
Queue<uint8_t> stop_hunt (1, "Stop Thermal Hunt Flag"); // Flag to signal the thermal cam should stop hunting
Queue<uint8_t> reset_this (1, "Reset Hunt Flag"); // Flag to reset thermal cam
Queue<uint8_t> direction (1, "Person Direction Flag"); // Direction of detected person

/** @brief   Task which is the mastermind of the program. 
 *  @details This task is the brain of the Scroomba that decides what should happen.
 *           Has initialize, hunt/wait, reverse, and reset states.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_mastermind (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning  

    uint8_t dir = 0;           // direction defaults to stopped
    byte state_m = 0;          // state defaults to initialization

    for (;;)
    {   
        if (state_m == 0) //Initialization State
        {
            state_m = 1; // transition to waiting/hunting
            motordirection.put(1); // must always pass a direction when passing a motor power, default forward
            motorpower.put(0); // motors default to stopped
        }
        else if (state_m == 1) // Waiting/Hunting State
        {
            if (limitdetect_front.any()) // flagged if the front bumper hits something
            {
                limitdetect_front.get(dir); // clear the bumper flag
                state_m = 2; // transition to the reverse state
                stop_hunt.put(1);   // tells the thermaldecoder to stop hunting
            }
            else if (direction.any()) // triggers if thermal cam finds a person
            {
                direction.get(dir); // get the direction the person is in
                if (dir == 0) // stopped state
                {        
                    motordirection.put(1); // doesn't really matter if 1/2/3/4 but needs to be one of them
                    motorpower.put(0);
                }
                else if (dir == 1) // forwards state
                {
                    motordirection.put(1);  // 1 indicates forward to the motors
                    motorpower.put(125);    // power to the motors; slowed down due to removing ToF sensor
                }
                else if (dir == 3) // left turn state
                {
                    motordirection.put(3);  // 3 indicates left to the motors
                    motorpower.put(125);    // power to the motors; slowed down due to removing ToF sensor
                }
                else if (dir == 4) // right turn state
                {
                    motordirection.put(4);  // 4 indicates right to the motors
                    motorpower.put(125);    // power to the motors; slowed down due to removing ToF sensor
                }
            }               
        }
        else if (state_m == 2) // Reverse State
        {
            if (limitdetect_front.any()) // in case this got pressed again, or
            {
                limitdetect_front.get(dir);
            }
            else if (stop_hunt.is_empty()) // just in case this somehow got cleared when it shouldn't
            {
                stop_hunt.put(1);
            }
            motordirection.put(2);
            motorpower.put(125);

            //bot should keep backing up until the limit switches are pressed.
            //when pressed, bot should stop backing up and begin reset.
            if(limitdetect_back.any()) // triggers if limit switch hit something
            {                
                motordirection.put(1);
                motorpower.put(0);
                state_m = 3; //Reset State
            }
        }
        else if (state_m == 3) //Reset State
        {
            //Method to unpress back limit switch
            //and then transition to stopped state.

            motordirection.put(1);
            motorpower.put(150);
            Serial.println("inch forward"); // so we see this happened during debug
            vTaskDelay(500); // experimental number to give it time to move forward
            motordirection.put(1);
            motorpower.put(0);
            vTaskDelay(500); // let have time to come to a stop 
            limitdetect_back.get(dir); // legacy from old method but still need to clear limitdetect
            reset_this.put(1); // reset system when back limit switch hit
            // crash_detect_active.put(1); // reset ToF active use flag
            state_m = 1;       // go back to waiting/hunting state
        }
        else // should never get here
        {
            Serial.println("Something is very wrong in mastermind");
            Serial.println("reinitializing");
            state_m = 0;    // reinitialize to try and fix things
        }
        vTaskDelay(10); // Delays things so we can actually see stuff happening
    }
}

void setup () 
{
    // Start the serial port, wait a short time, then say hello. Use the
    // non-RTOS delay() function because the RTOS hasn't been started yet
    Serial.begin (115200);
    delay (2000);
    Serial << endl << endl << "ME507 UI Lab Starting Program" << endl;

    // Create a task which runs the thermal camera
    xTaskCreate (task_thermal,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);

    // Create a task which runs the thermal camera decoder
    xTaskCreate (task_thermaldecoder,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);
    
    // creates a task which runs the mastermind function
    xTaskCreate (task_mastermind,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);    
    
    // creates a task which runs the back limit switch
    xTaskCreate (task_limit_back,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL); 
    
    // creates a task which runs the front limit switch
    xTaskCreate (task_limit_front,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);

    // creates a task that runs the motors
    xTaskCreate (task_motor,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL); 

    // If using an STM32, we need to call the scheduler startup function now;
    // if using an ESP32, it has already been called for us
    #if (defined STM32L4xx || defined STM32F4xx)
        vTaskStartScheduler ();
    #endif
}


/** @brief   Arduino's low-priority loop function, which we don't use.
 *  @details A non-RTOS Arduino program runs all of its continuously running
 *           code in this function after @c setup() has finished. When using
 *           FreeRTOS, @c loop() implements a low priority task on most
 *           microcontrollers, and crashes on some others, so we'll not use it.
 */
void loop () 
{
}
