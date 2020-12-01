/** @file thermal_decoder.cpp
 *      This file contains a task that decodes information from the thermal camera 
 *      to find and track a person for the Scroomba.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include "thermal_decoder.h"

extern Queue<float> thermaldata;
extern Queue<uint8_t> direction;
extern Queue<uint8_t> stop_hunt;
extern Queue<uint8_t> reset_this;

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
