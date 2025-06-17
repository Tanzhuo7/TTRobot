/**
 ****************************************************************************************************
 * @file        led.h
 * @author      TANZHUO
 * @version     V1.0
 * @date        2024-5-7
 * @brief       LED 驱动代码
 ****************************************************************************************************
 */

#ifndef __LED_H
#define __LED_H

#include "Arduino.h"

/* 引脚定义 */
#define LED_PIN       1   /* 开发板上LED连接到GPIO1引脚 */

/* 宏函数定义 */
#define LED(x)        digitalWrite(LED_PIN, x)
#define LED_TOGGLE()  digitalWrite(LED_PIN, !digitalRead(LED_PIN))

/* 函数声明 */
void led_init(void);      /* led引脚初始化函数 */

#endif



