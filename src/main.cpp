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
Queue<float> motorparams (3, "Motor Task Parameters"); //Thermal Camera Data Queue

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
            Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
        } 
        else 
        {
            Serial.println(" out of range ");
        }
    
        vTaskDelay(10000); // Delays things so we can actually see stuff happening

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
    float diff[AMG88xx_PIXEL_ARRAY_SIZE];   // the diff         

    bool calib = false; // program starts in need of calibration
    bool detect = false; // program starts without having seen something

    uint8_t count = 0;

    const uint8_t LEFT = 3;     // match the motor driver
    const uint8_t MIDDLE = 1;   // match the motor driver
    const uint8_t RIGHT = 4;    // match the motor driver

    uint8_t high_v = 0;         // highest value when checking the array
    uint8_t high_i = 0;         // index of highest value in 0 to 63 form


    uint8_t tbd = 0;            // replace with a share later   

    for (;;)
    {
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
                    if (diff[i-1]>=2) // checking if differential is greater than threshold for person
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
            
                if (detect)   // figure out when we get out of the detected state
                {
                    if (high_i<16) // was 24
                    {
                        tbd = RIGHT;
                        Serial.println("GO RIGHT!");
                    }
                    else if (high_i>=48) // was 40
                    {
                        tbd = LEFT;
                        Serial.println("GO LEFT!");
                    }
                    else
                    {
                        tbd = MIDDLE;
                        Serial.println("middle?");
                    }
                    high_v = 0;
                    high_i = 0;                    
                }
                else
                {
                    Serial.println("Waiting");
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
        
        
        // vTaskDelay(1000); // Delays things so we can actually see stuff happening

    }
}

/** @brief   Task which is the mastermind of the program. 
 *  @details This task is the brain of the Scroomba that decides what should happen.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_mastermind (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning  

    for (;;)
    {
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

    float status = 0;

    for (;;)
    {
        //checks if limit switches are pressed
        if (digitalRead(pin) || digitalRead(pin2))      //If the pin is high, then limit switch detected a boundary
        {
            limitdetect.put(1);
            status = 1;

            while(status)
            {
                limitdetect.peek(status);
                vTaskDelay(100)
            }

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
    const uint8_t enA = 11;
    const uint8_t enB = 11;
    const uint8_t in1 = 11;
    const uint8_t in2 = 11;
    const uint8_t in3 = 11;
    const uint8_t in4 = 11;

    float motordata [3];

    for (;;)
    {
        if(motorparams.any())
        {
            //Receiving motor parameters
            for(uint8_t i = 1; i<=4; i++)
            {
                motorparams.get(motordata[i-1]);
            }

            //Set Direction
            if(motordata[0] == 1) //Forwards Direction
            {
                pinMode(in1, HIGH);
                pinMode(in2, LOW);

                pinMode(in3, LOW);
                pinMode(in4, HIGH);
            }
            else if(motordata[0] == 2) //Reverse Direction
            {
                pinMode(in1, LOW);
                pinMode(in2, HIGH);

                pinMode(in3, HIGH);
                pinMode(in4, LOW);
            }
            else if(motordata[0] == 3) //Left Turn
            {
                pinMode(in1, HIGH);
                pinMode(in2, LOW);

                pinMode(in3, HIGH);
                pinMode(in4, LOW);
            }
            else if(motordata[0] == 4) //Right Turn
            {
                pinMode(in1, LOW);
                pinMode(in2, HIGH);

                pinMode(in3, LOW);
                pinMode(in4, HIGH);
            }

            //Set PWM signal
            analogWrite(enA, motordata[1]);
            analogWrite(enB, motordata[2]);

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
                 1,                               // Priority
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
