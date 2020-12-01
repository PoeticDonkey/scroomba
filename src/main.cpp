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

Queue<float> thermaldata (640, "Thermal Data"); //Thermal Camera Data Queue
Queue<uint8_t> motordirection (1, "Motor Task Parameters"); // What motors should do queue
Queue<uint8_t> motorpower (1, "Motor Task Parameters"); // What motors should do queue
Queue<uint8_t> limitdetect_back (1, "Back Limit Switch Detection Flag"); // Back Limit Switch Flag
Queue<uint8_t> limitdetect_front (1, "Front Limit Switch Detection Flag"); // Front Limit Switch Flag
Queue<uint8_t> stop_hunt (1, "Stop Thermal Hunt Flag"); // Flag to signal the thermal cam should stop hunting
Queue<uint8_t> reset_this (1, "Reset Hunt Flag"); // Flag to reset thermal cam
Queue<uint8_t> direction (1, "Person Direction Flag"); // Direction of detected person


/** @brief   Task which runs the Thermal Camera. 
 *  @details This task initializes and collects data from the Thermal Camera into a [64] array.
 *           Every 8 values moves from top to bottom in the FoV.
 *           Every group of 8 values moves from left to right in the FoV. *           
 *  @param   p_params A pointer to function parameters which we don't use.
 *  @param   amg The thermal camera object.
 *  @param   pixels The array that stores the thermal camera data.
 */

void task_thermal (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    Adafruit_AMG88xx amg;
    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
    bool status = 0;
    
    // default settings
    status = amg.begin();
    if (!status) 
    {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    vTaskDelay(100); // let sensor boot up
    for (;;)
    {
        //read all the pixels
        amg.readPixels(pixels);
        for(uint8_t i = 1; i <= 64; i++)
        {
            thermaldata.put(pixels[i-1]);
        }
        //delay a bit
        vTaskDelay(100);
    }
}

/** @brief   Task which interperates the thermal camera data. 
 *  @details This task takes the thermal camera data and makes sense of it.
 *           It calibrates to ambient conditions and differentials to the
 *           calibration matrix are used to judge if a person is there.
 *           When a person is detected, their position is fed to mastermind
 *           for course correction.
 *           Stops hunt and resets when signaled by mastermind.
 *           Warning: Messing up calibration gives bad results!
 *  @param   p_params A pointer to function parameters which we don't use.
 */

void task_thermaldecoder (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];  // takes the current thermal camera data
    float ambient[AMG88xx_PIXEL_ARRAY_SIZE]; // the ambient conditions calibration data
    float diff[AMG88xx_PIXEL_ARRAY_SIZE];    // the differential between ambient and pixels        

    bool calib = false;  // program starts in need of calibration
    bool detect = false; // program starts without having seen something

    uint8_t count = 0;   // program starts without any calibration cycles done

    const uint8_t LEFT = 3;     // match the motor driver left value
    const uint8_t MIDDLE = 1;   // match the motor driver middle value
    const uint8_t RIGHT = 4;    // match the motor driver right value

    uint8_t high_v = 0;         // highest value when checking the array
    uint8_t high_i = 0;         // index of highest value in 0 to 63 form

    uint8_t reset = 0;          // used to trash reset flag 

    for (;;)
    {
        if(reset_this.any()) // reset actions
        {
            // Scroomba should no longer be calibrated or in dectected mode
            calib = false;
            detect = false;
            count = 0;
            high_v = 0;
            high_i = 0;
            reset_this.get(reset); // clear reset flag
            if (stop_hunt.any())
            {
                stop_hunt.get(reset); // clear stop hunt flag
            }            
            Serial.println("Scroomba reset!");
        }
        
        if(thermaldata.any()) // only decode if there is data from the thermal camera
        {
            for(uint8_t i = 1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
            {
                thermaldata.get(pixels[i-1]);
                if (stop_hunt.any()) // flagged if backing up, cleared when scroomba is ready to reset
                {
                    if (direction.any()) // just in case stuff gets into it when it shouldn't
                    {
                        direction.get(reset); // get rid of the data
                    }
                }
                else if (!calib)          // get the data for calibration ambient maxtrix
                {
                    if (count == 0) // special case for the first time through in calibration
                    {
                        ambient[i-1] = pixels[i-1];
                    }
                    else
                    {
                        ambient[i-1] = pixels[i-1] + ambient[i-1];
                    }
                }
                else if (!detect) // looking for a person
                {
                    diff[i-1] = pixels[i-1] - ambient[i-1];
                    if (diff[i-1]>=3)   // checking if differential is greater than threshold for person
                    {
                        detect = true;          // switch to detected mode
                        high_v = pixels[i-1];   // highest value is now from here
                        high_i = i-1;           // remember highest value index                    
                    }
                }                
                else // tracking the detected person
                {
                    if (pixels[i-1]>high_v) // check if the pixel is greater than highest recorded temp reading
                        {
                            high_v = pixels[i-1]; // record the new highest temp reading
                            high_i = i-1; // record the new index of this reading
                        }
                }
            }

            if (stop_hunt.is_empty()) // keeps from passing data to mastermind when not in hunting/waiting mode
            {
                if (calib)      // must be calibrated to pass data
                {             
                    if (detect)   // must have detected something to pass data
                    {   
                        // check the index of the highest value
                        // pass corresponding direction to mastermind
                        // (using the direction queue)
                        // current configuration uses a wide middle band for the field of view
                        
                        if (high_i<16) // 24 for wide sides FoV config.
                        {
                            direction.put(RIGHT);
                            //Serial.println("right"); // debug check
                        }
                        else if (high_i>=48) // 40 for wide sides FoV config.
                        {
                            direction.put(LEFT);
                            //Serial.println("left"); // debug check
                        }
                        else
                        {
                            direction.put(MIDDLE);
                            //Serial.println("middle"); // debug check
                        }
                        high_v = 0; // reset high value after passing data
                        high_i = 0; // reset high value index after passing data                    
                    }
                    else
                    {
                        // waiting to dectect someone
                    }
                }
                else // rest of calibration
                {
                    count++; // keep track of times calibration data is taken
                    if (count >= 50) // set the number of times calibration data should be averaged over
                    {
                        for(uint8_t i = 1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
                        {
                            ambient[i-1] = ambient[i-1]/count;
                        }
                        calib = true; // stops calibration mode
                    }
                }
            }
            else
            {
               // not hunting
            }
        }        
    }
}

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
            //Experimental method to unpress limit switches
            //and then transition to stopped state.
            //Only works for limit switches on back of bot.

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

/** @brief   Task which handles the back limit switches
 *  @details This task initializes the pins and raises a flag for
 *           Mastermind if the back limit switches were pressed.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_limit_back (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning  
    
    const uint8_t pin = D9; //FIND REAL PINS

    pinMode(pin,INPUT);

//    float status = 0; // I decided I don't like this method -- Michael

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
            
            /*
            limitdetect.put(1);
            status = 1;

            while(status)
            {
                limitdetect.peek(status);
                vTaskDelay(100);
            }
            */
        }
        vTaskDelay(100); // Delays things so we can actually see stuff happening
    }
}

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

/** @brief   Task which runs the motors. 
 *  @details This task initializes the motors and runs them according to
 *           input from Mastermind
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_motor (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning  

    //Set Pins for motors
    const uint8_t enA = D2; //PA_10
    const uint8_t enB = A4; //PC_1
    const uint8_t in1 = D5; //PB_4
    const uint8_t in2 = D4; //PB_5
    const uint8_t in3 = A0; //PA_0
    const uint8_t in4 = A1; //PA_1

    pinMode(enA,OUTPUT);
    pinMode(enB,OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);

    uint8_t motordata [3];
    uint8_t motorapwm = 0;
    uint8_t motorbpwm = 0;

    for (;;)
    {
        if(motorpower.any()) // flagged if mastermind sends a change to the motor power
        {
            motordirection.get(motordata[0]);
            motorpower.get(motordata[1]);

            //Set Direction
            if(motordata[0] == 1) //Forwards Direction
            {
                pinMode(in1, OUTPUT); // this shouldn't do anything, but the code breaks without it
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);
                
                digitalWrite(in1, LOW);
                digitalWrite(in4, LOW);
                motorapwm = in2;
                motorbpwm = in3;
            }
            else if(motordata[0] == 2) //Reverse Direction
            {
                pinMode(in1, OUTPUT); // this shouldn't do anything, but the code breaks without it
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);

                digitalWrite(in2, LOW);
                digitalWrite(in3, LOW);
                motorapwm = in1;
                motorbpwm = in4;    
            }
            else if(motordata[0] == 3) //Left Turn
            {
                pinMode(in1, OUTPUT); // this shouldn't do anything, but the code breaks without it
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);
                
                digitalWrite(in1, LOW);
                digitalWrite(in3, LOW);
                motorapwm = in2;
                motorbpwm = in4;
            }
            else if(motordata[0] == 4) //Right Turn
            {
                pinMode(in1, OUTPUT); // this shouldn't do anything, but the code breaks without it
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);

                digitalWrite(in2, LOW);
                digitalWrite(in4, LOW);
                motorapwm = in1;
                motorbpwm = in3;
            }

            //Set PWM signal
            analogWrite(motorapwm, motordata[1]);
            analogWrite(motorbpwm, motordata[1]);

            vTaskDelay(10);
        }
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
