#ifndef __ESP32_MOTOR_BASIC_H__
#define __ESP32_MOTOR_BASIC_H__

#include "Arduino.h"
#include <stdio.h>
#include "esp32-hal-ledc.h"

#define freq                1000        //pwm频率
#define resolution          10          //分辨率 2^10 = 1024

#define OK 			        1
#define ERROR 		        0

/* 电机ID */
#define MOTOR1              1//L
#define MOTOR2              2//R

/* L_MOTOR_PIN */    
#define MOTOR_L_ID          1
#define MOTOR_L_IN1         9
#define MOTOR_L_IN2         46
#define MOTOR_L_PWM         3

/* R_MOTOR_PIN */  
#define MOTOR_R_ID          2
#define MOTOR_R_IN1         14
#define MOTOR_R_IN2         21
#define MOTOR_R_PWM         13

class Esp32_DCMotor
{
private:
    /* 电机速度，单位mm */
    int16_t speeds[4]{0, 0};
    bool mMotorAttached[4]{false, false, false, false};

public:
    Esp32_DCMotor() = default;
    ~Esp32_DCMotor() = default;
    bool attachMotor(uint8_t id, uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioPwm);
    void stopMotor(int8_t motorId);
    void updateMotorSpeed(int8_t id, int16_t pwmValue);
};

#endif // __ESP32_MOTOR_BASIC_H__