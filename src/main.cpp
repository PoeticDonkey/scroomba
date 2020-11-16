/** @file main.cpp
 *    This file contains a crude program for inclination of an IMU. If given more time and further
 *    resources, this file would need to be reworked.
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

/** @brief   Task which interacts with a user. 
 *  @details This task demonstrates how to use a FreeRTOS task for interacting
 *           with some user while other more important things are going on.
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


void task_thermal (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    Adafruit_AMG88xx amg;
    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
    
    Serial.println(F("AMG88xx pixels"));

    bool status;
    
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

/** @brief   Task which interacts with a user. 
 *  @details This task demonstrates how to use a FreeRTOS task for interacting
 *           with some user while other more important things are going on.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_thermaldecoder (void* p_params)
{
    (void)p_params;            // Does nothing but shut up a compiler warning

    float pixels[64];     

    for (;;)
    {
        if(thermaldata.any())
        {
            for(uint8_t i = 1; i<=64; i++)
            {
                thermaldata.get(pixels[i-1]);
            }

            Serial.print("[");
            for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++)
            {
                Serial.print(pixels[i-1]);
                Serial.print(", ");
                if( i%8 == 0 ) Serial.println();
            }
            Serial.println("]");
            Serial.println();
        }
        
        // vTaskDelay(1000); // Delays things so we can actually see stuff happening

    }
}

/** @brief   Task which interacts with a user. 
 *  @details This task demonstrates how to use a FreeRTOS task for interacting
 *           with some user while other more important things are going on.
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
                //Set in pins one way
            }
            else if(motordata[0] == 2) //Reverse Direction
            {
                //Set pins other way
            }
            else if(motordata[0] == 3) //Left Turn
            {
                //Set both in pins one way
            }
            else if(motordata[0] == 4) //Right Turn
            {
                //Set both in pins other way
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
