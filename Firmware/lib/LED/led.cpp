/**
 ****************************************************************************************************
 * @file        led.h
 * @author      TANZHUO
 * @version     V1.0
 * @date        2024-5-7
 * @brief       LED 驱动代码
 ****************************************************************************************************
 */

#include "led.h"

/**
* @brief       初始化LED相关IO口
* @param       无
* @retval      无
*/
void led_init(void) 
{
    pinMode(LED_PIN, OUTPUT);     /* 设置led引脚为输出模式 */
    digitalWrite(LED_PIN, LOW);  /* 结合原理图设计,实物LED获得高电平会熄灭 */
}
