#include "esp32_motor_basic.h"
#include "Arduino.h"
#include "math.h"

/**
 * @brief       电机引脚映射，pwm通道配置
 * @param       电机id
 * @param       IN1
 * @param       IN2
 * @param       PWM
 * @retval      OK || ERROR
 */
bool Esp32_DCMotor::attachMotor(uint8_t id, uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioPwm)
{
    /*chan_computer*/
    uint8_t chan;
    if(id == 1){
        chan = 0;
    }
    else if(id == 2){
        chan = 1;
    }
    else if(id == 3){
        chan = 2;
    }
    else if(id == 4){
        chan = 3;
    }
    else{
        return ERROR;
    }
    /*gpio_init*/
    pinMode(gpioIn1, OUTPUT);
    pinMode(gpioIn2, OUTPUT);

    digitalWrite(gpioIn1, LOW);
    digitalWrite(gpioIn2, LOW);

    /*pwm_config*/
    ledcSetup(chan, freq, resolution);
    ledcAttachPin(gpioPwm, chan);
    ledcWrite(chan, 0);

    return OK;
}

/**
 * @brief       电机停止
 * @param       电机id
 * @retval      无
 */
void Esp32_DCMotor::stopMotor(int8_t motorId)
{
    switch (motorId)
    {
    case 1:digitalWrite(MOTOR_L_IN1 || MOTOR_L_IN2, LOW);break;
    case 2:digitalWrite(MOTOR_R_IN1 || MOTOR_R_IN2, LOW);break;
    
    default:
        break;
    }
}

/**
 * @brief       电机速度更新
 * @param       电机id
 * @param       速度：0～1024
 * @retval      无
 */
void Esp32_DCMotor::updateMotorSpeed(int8_t id, int16_t pwmValue)
{
    /* 电机转动方向 */
    int dir = pwmValue;
    /*chan_computer 选择对应的通道*/
    uint8_t chan;
    if(id == 1){
        chan = 0;
    }
    else if(id == 2){
        chan = 1;
    }
    else if(id == 3){
        chan = 2;
    }
    else if(id == 4){
        chan = 3;
    }

    if(dir >= 0)
    {
        switch (id)
        {
        case 1:digitalWrite(MOTOR_L_IN1, HIGH);digitalWrite(MOTOR_L_IN2, LOW);break;
        case 2:digitalWrite(MOTOR_R_IN1, HIGH);digitalWrite(MOTOR_R_IN2, LOW);break;

        default:
            break;
        }

        /* 限制占空比 */
        if(pwmValue >= pow(2, resolution))
        {
            pwmValue = pow(2, resolution);
        }
        ledcWrite(chan, pwmValue);
    }
    else
    {
        switch (id)
        {
        case 1:digitalWrite(MOTOR_L_IN1, LOW);digitalWrite(MOTOR_L_IN2, HIGH);break;
        case 2:digitalWrite(MOTOR_R_IN1, LOW);digitalWrite(MOTOR_R_IN2, HIGH);break;

        default:
            break;
        }

        /* 限制占空比 */
        if(-pwmValue >= pow(2, resolution))
        {
            pwmValue = pow(2, resolution);
        }
        ledcWrite(chan, -pwmValue);
    }
}