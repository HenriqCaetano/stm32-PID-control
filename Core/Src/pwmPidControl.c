#include "pwmPidControl.h"

#include <stdio.h>

//the min and max values should consider the duty cycle %
//example: if the minValue is 10% duty cycle, the minimalPwm must be a value that reflects it
void pidInit(Pid* p, float minimalPwm, float maximumPwm, float kp, float ki, float kd){
    p->max = maximumPwm;
    p->min = minimalPwm;
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}


/// @brief given current and target speds, computes PWM increment
/// @param setPoint target speed in encoder pulses
/// @param feedBackValue current speeed in encoder pulses
/// @param p PID controller
/// @return increment to PWM
float computePwmValue(float setPoint, float feedBackValue, Pid* p){
    float pastError = p->error;


    if(setPoint < 0) setPoint *= -1;
    if (feedBackValue < 0) feedBackValue *= -1;

    p->error = setPoint - feedBackValue; //get current error
    p->integralValue += p->error; //update integralError

    float result = 0;
    //calculate pwm output
    result = p->kp * p->error; + p->ki * p->integralValue + p->kd * (pastError - p->error);

    //deals with limit values
    if(result > p->max) result = p->max;
    else if(result < p->min) result = p->min;

    //printf("%f\r\n", result);
    return result;

}
