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

#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "taskqueue.h"
#include "taskshare.h"

Queue<float> thermaldata (640, "Thermal Data"); //Thermal Camera Data Queue
Queue<uint8_t> motordirection (1, "Motor Task Parameters"); // What motors should do queue
Queue<uint8_t> motorpower (1, "Motor Task Parameters"); // What motors should do queue
Queue<uint8_t> limitdetect (10, "Limit Switch Detection Flag"); // Limit Switch Flag
Queue<uint8_t> reset_this (10, "Reset Hunt Flag"); // Flag to reset thermal cam
Queue<uint8_t> direction (10, "Person Direction Flag"); // Direction of detected person
Queue<uint8_t> too_close (10, "ToF Crash Prevention Flag"); // Stop before running into something

/** @brief   Task which runs the ToF sensor. 
 *  @details This task initializes and runs the Time of Flight sensor.
 *           Current bugs: reports ~50 mm when something isn't in the range.
 *                         inconsistent measuremeent past 200mm.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_tof (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    Adafruit_VL53L0X lox = Adafruit_VL53L0X();

    Serial.println("Adafruit VL53L0X test");
    if (!lox.begin()) 
    {
        Serial.println(F("Failed to boot VL53L0X"));
        while(1);
    }
    // power 
    Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

    for (;;)
    {
        VL53L0X_RangingMeasurementData_t measure;
    
        Serial.print("Reading a measurement... ");
        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

        if (measure.RangeStatus != 4)   // phase failures have incorrect data
        {
            if(measure.RangeMilliMeter<=50)
            {
                too_close.put(2);
                Serial.print("Distance (mm): ");
                Serial.print(measure.RangeMilliMeter); Serial.println(" too close!");
            }
            else
            {
                Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
            }            
        } 
        else 
        {
            //Serial.println(" out of range ");
        }
        vTaskDelay(1000); // Delays things so we can actually see stuff happening

    }
}

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
    
    Serial.println(F("AMG88xx pixels"));

    bool status = 0;
    
    // default settings
    status = amg.begin();
    if (!status) 
    {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    
    // Serial.println("-- Pixels Test --");

    // Serial.println();

    vTaskDelay(100); // let sensor boot up

    for (;;)
    {
        //read all the pixels
        amg.readPixels(pixels);

        for(uint8_t i = 1; i <= 64; i++)
        {
            thermaldata.put(pixels[i-1]);
        }

        // Serial.print("[");
        // for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
        // {
        //     Serial.print(pixels[i-1]);
        //     Serial.print(", ");
        //     if( i%8 == 0 ) Serial.println();
        // }
        // Serial.println("]");
        // Serial.println();

        //delay a bit
        vTaskDelay(2000);
    }

}

/** @brief   Task which interperates the thermal camera data. 
 *  @details This task takes the thermal camera data and makes sense of it.
 *           It calibrates and then judges from that calibration if a person is there.
 *           Warning: Messing up calibration gives bad results!
 *  @param   p_params A pointer to function parameters which we don't use.
 */

void task_thermaldecoder (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
    float ambient[AMG88xx_PIXEL_ARRAY_SIZE];
    float diff[AMG88xx_PIXEL_ARRAY_SIZE];   // the diff between ambient and pixels         

    bool calib = false; // program starts in need of calibration
    bool detect = false; // program starts without having seen something

    uint8_t count = 0;

    const uint8_t LEFT = 3;     // match the motor driver
    const uint8_t MIDDLE = 1;   // match the motor driver
    const uint8_t RIGHT = 4;    // match the motor driver

    uint8_t high_v = 0;         // highest value when checking the array
    uint8_t high_i = 0;         // index of highest value in 0 to 63 form

    // uint8_t tbd = 0;            // replace with thing to share direction later
    uint8_t reset = 0;          // used to trash reset flag 

    for (;;)
    {
        
        if(reset_this.any()) // reset actions
        {
            calib = false;
            detect = false;
            count = 0;
            high_v = 0;
            high_i = 0;
            reset_this.get(reset); // clear reset flag
            Serial.println("reset!");
        }
        else
        {
            reset = 0; // was worried about two if statements next to each other
        }
        
        if(thermaldata.any())
        {
            for(uint8_t i = 1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
            {
                thermaldata.get(pixels[i-1]);
                if (!calib)          // get the data for calibration
                {
                    if (count == 0) // is there a better way to handle the first time?
                    {
                        ambient[i-1] = pixels[i-1];
                    }
                    else
                    {
                        ambient[i-1] = pixels[i-1] + ambient[i-1];
                    }
                }
                else if (!detect) // looking for person
                {
                    diff[i-1] = pixels[i-1] - ambient[i-1];
                    if (diff[i-1]>=3) // checking if differential is greater than threshold for person
                    {
                        detect = true; // switch to detected mode
                        high_v = pixels[i-1]; // highest value is now from here
                        high_i = i-1;           // remember highest value index                    
                    }
                }
                else
                {
                    if (pixels[i-1]>high_v) // checking if differential is greater than highest val
                        {
                            high_v = pixels[i-1];
                            high_i = i-1;
                        }
                }
            }

            // code to check if it's working right

            if (calib)      // figure out when we should leave the calib state
            {             
                if (detect)   // figure out when we get out of the detected state
                {
                    /*
                    Serial.println("Temperatures");
                    Serial.print("[");
                    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
                    {
                        Serial.print(pixels[i-1]);
                        Serial.print(", ");
                        if( i%8 == 0 ) Serial.println();
                    }
                    Serial.println("]");
                    Serial.println();     
                    */               
                    
                    if (high_i<16) // was 24
                    {
                        direction.put(RIGHT);
                        //tbd = RIGHT;
                        Serial.println("GO RIGHT!");
                    }
                    else if (high_i>=48) // was 40
                    {
                        direction.put(LEFT);
                        //tbd = LEFT;
                        Serial.println("GO LEFT!");
                    }
                    else
                    {
                        direction.put(MIDDLE);
                        //tbd = MIDDLE;
                        Serial.println("Middle");
                    }
                    high_v = 0;
                    high_i = 0;                    
                }
                else
                {
                    Serial.println("Waiting");
                    /*
                    Serial.println("Temperature Differential");
                    Serial.print("[");
                    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
                    {
                        Serial.print(diff[i-1]);
                        Serial.print(", ");
                        if( i%8 == 0 ) Serial.println();
                    }
                    Serial.println("]");
                    Serial.println();
                    */
                }
            }
            
            if(!calib) // rest of calibration
            {
                Serial.println("calibrating"); // temp code while we're testing

                count++; // keep track of times calibration data is taken
                if (count >= 10) // set the number of times calibration data should be averaged over
                {
                    for(uint8_t i = 1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
                    {
                        ambient[i-1] = ambient[i-1]/count;
                    }
                    calib = true; // stops calibration mode
                }
            }

            /* // commented out so we see just the ambient data for now.
            
            // code to print the array for us to see
            Serial.print("[");
            for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++) // why does this work when that size shouldn't be defined?
            {
                Serial.print(pixels[i-1]);
                Serial.print(", ");
                if( i%8 == 0 ) Serial.println();
            }
            Serial.println("]");
            Serial.println();
            */
        }        
    }
}

/** @brief   Task which is the mastermind of the program. 
 *  @details This task is the brain of the Scroomba that decides what should happen.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_mastermind (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning  

    uint8_t dir = 0;           // direction defaults to stopped

    for (;;)
    {   
        /* 
        direction determind from sensors
        checks in order of importance
        */
        
        /*
        if(limitdetect.any()) // triggers if limit switch hit something
        {         
            
            //Experimental method to unpress limit switches
            //and then transition to stopped state.
            //Only would work when limit switches are on back.
                        
            motorparams.put(1);
            motorparams.put(200);
            motorparams.put(200);
            Serial.println("inch forward");
            vTaskDelay(200);
            limitdetect.get(dir);
            reset_this.put(1); // reset system when back limit switch hit
        }
        */
        if (too_close.any()) // triggers if ToF sees something too close
        {
            too_close.get(dir);
        }
        else if (direction.any()) // triggers if thermal cam finds a person
        {
            direction.get(dir);
        }

        if (dir == 0) // stopped state
        {        
            motordirection.put(1); // doesn't really matter if 1/2/3/4 but needs to be one of them
            motorpower.put(0);
        }
        else if (dir == 1) // forwards state
        {
            motordirection.put(1);
            motorpower.put(150);
        }
        else if (dir == 2) // backwards state
        {
            motordirection.put(2);
            motorpower.put(150);
        }
        else if (dir == 3) // left turn state
        {
            motordirection.put(3);
            motorpower.put(150);
        }
        else if (dir == 4) // right turn state
        {
            motordirection.put(4);
            motorpower.put(150);
        }    
   
        vTaskDelay(1000); // Delays things so we can actually see stuff happening
    }
}

/** @brief   Task which interacts with a user. 
 *  @details This task demonstrates how to use a FreeRTOS task for interacting
 *           with some user while other more important things are going on.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_limit (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning  
    
    const uint8_t pin = 11; //FIND REAL PINS
    const uint8_t pin2 = 11; //FIND REAL PINS

//    float status = 0; // I decided I don't like this method -- Michael

    for (;;)
    {
        //checks if limit switches are pressed
        if (digitalRead(pin) || digitalRead(pin2))      //If the pin is high, then limit switch detected a boundary
        {
            limitdetect.put(0); // put dir stop value in queue
            while(limitdetect.any())
            {
                vTaskDelay(100); // do nothing until mastermind
            }
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
        vTaskDelay(50); // Delays things so we can actually see stuff happening
    }
}

/** @brief   Task which interacts with a user. 
 *  @details This task demonstrates how to use a FreeRTOS task for interacting
 *           with some user while other more important things are going on.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_motor (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning  

    //Set Pins
    const uint8_t enA = D2; //PA_10
    const uint8_t enB = A4; //PC_1
    const uint8_t in1 = D5; //PB_4
    const uint8_t in2 = D4; //PB_5
    const uint8_t in3 = A0; //PA_0
    const uint8_t in4 = A1; //PA_1'

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
        if(motorpower.any())
        {
            // //Receiving motor parameters
            // for(uint8_t i = 1; i<=4; i++)
            // {
            //     motorparams.get(motordata[i-1]);
            // }

            // Serial.print("Motor Parameters: ");
            // Serial.println(motordata[0]);
            // Serial.println(motordata[1]);
            // Serial.println(motordata[2]);
            motordirection.get(motordata[0]);
            motorpower.get(motordata[1]);

            Serial.print("Motor Parameters: ");
            Serial.println(motordata[0]);
            Serial.println(motordata[1]);

            //Set Direction
            if(motordata[0] == 1) //Forwards Direction
            {
                Serial.println("forward");

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
                Serial.println("backward");

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
                Serial.println("left");

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
                Serial.println("right");

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

    // Create a task which prints a slightly disagreeable message
    xTaskCreate (task_tof,
                 "User Int.",                     // Name for printouts
                 1536,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 4,                               // Priority
                 NULL);                           // Task handle

    // Create a task which prints a more agreeable message
    xTaskCreate (task_thermal,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);

    // Create a task which prints a more agreeable message
    xTaskCreate (task_thermaldecoder,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);

    xTaskCreate (task_mastermind,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL);    

    xTaskCreate (task_limit,
                 "Simul.",
                 1024,                            // Stack size
                 NULL,
                 4,                               // Priority
                 NULL); 

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
