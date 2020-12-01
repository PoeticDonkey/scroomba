/** @file motor.cpp
 *      This file contains a task that operates the motors based on the direction and power inpu
 *      sent by the mastermind task.
 * 
 *  @details This task initializes the motors and operates their power and direction based
 *           on the configuration of the in pins, changing based on input from mastermind task.
 * 
 *  @author Michael Conn
 *  @author Scott Mangin
 *  @author Nicholas Holman
 * 
 *  @date   2020-Dec-01 Original file
 */

#include "motor.h"

extern Queue<uint8_t> motordirection; ///<Super-boolean for direction of travel Queue
extern Queue<uint8_t> motorpower; ///<Duty cycle value for designated motor pins Queue

/** @brief   Motor Driver and Direction task for both robot chassis motors, specific to Scroomba.
 *  @details This task initializes the motors and operates their power and direction based
 *           on the configuration of the in pins, changing based on input from mastermind task.
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

    //Set all used pins as output pins
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    //Initialize the motor enable pins to be on, since motor driver only has PWM on in pins.
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);

    //Set variables
    uint8_t motordata [2]; ///<Local array to store motor data sent by mastermind
    uint8_t motorapwm = 0; //Variable to set the desired PWM in pin for motor A
    uint8_t motorbpwm = 0; //Variable to set the desired PWM in pin for motor B

    for (;;)
    {
        if(motorpower.any()) // flagged if mastermind sends a change to the motor power share
        {
            motordirection.get(motordata[0]);
            motorpower.get(motordata[1]);

            //Set Direction
            if(motordata[0] == 1) //Forwards Direction
            {
                //The initialization of the pinmodes for the in pins shouldn't do anything, but the code breaks without it
                pinMode(in1, OUTPUT);
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);
                
                //Sets the direction through the configuration of in pins that are low and in pins that will be PWM
                digitalWrite(in1, LOW);
                digitalWrite(in4, LOW);
                motorapwm = in2;
                motorbpwm = in3;
            }
            else if(motordata[0] == 2) //Reverse Direction
            {
                //The initialization of the pinmodes for the in pins shouldn't do anything, but the code breaks without it
                pinMode(in1, OUTPUT); 
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);

                //Sets the direction through the configuration of in pins that are low and in pins that will be PWM
                digitalWrite(in2, LOW);
                digitalWrite(in3, LOW);
                motorapwm = in1;
                motorbpwm = in4;    
            }
            else if(motordata[0] == 3) //Left Turn
            {
                //The initialization of the pinmodes for the in pins shouldn't do anything, but the code breaks without it
                pinMode(in1, OUTPUT);
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);
                
                //Sets the direction through the configuration of in pins that are low and in pins that will be PWM
                digitalWrite(in1, LOW);
                digitalWrite(in3, LOW);
                motorapwm = in2;
                motorbpwm = in4;
            }
            else if(motordata[0] == 4) //Right Turn
            {
                //The initialization of the pinmodes for the in pins shouldn't do anything, but the code breaks without it
                pinMode(in1, OUTPUT);
                pinMode(in2, OUTPUT);
                pinMode(in3, OUTPUT);
                pinMode(in4, OUTPUT);
                
                //Sets the direction through the configuration of in pins that are low and in pins that will be PWM
                digitalWrite(in2, LOW);
                digitalWrite(in4, LOW);
                motorapwm = in1;
                motorbpwm = in3;
            }

            //Set PWM signal to the desired in pins of motor A and motor B
            analogWrite(motorapwm, motordata[1]);
            analogWrite(motorbpwm, motordata[1]);

            vTaskDelay(10);
        }
    }
}