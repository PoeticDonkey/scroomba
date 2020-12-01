/** @file thermal_cam.cpp
 *      This file contains a task that collects data from the thermal camera.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include "thermal_cam.h"

extern Queue<float> thermaldata;


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