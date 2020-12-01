/** @file motor.cpp
 *      This file contains a task that runs the motors.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include "motor.h"

extern Queue<uint8_t> motordirection;
extern Queue<uint8_t> motorpower;

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