/**
 ****************************************************************************************************
 * @file        key_scan.h
 * @author      TANZHUO
 * @version     V1.0
 * @date        2024-5-7
 * @brief       按键输入 驱动代码
 ****************************************************************************************************
 */

#ifndef __KEY_SCAN_H
#define __KEY_SCAN_H

#include "Arduino.h"

/******************************************************************************************/
/* 引脚 定义 */

#define KEY1_GPIO_PIN                   7
#define KEY2_GPIO_PIN                   6
#define KEY_BOOT_GPIO_PIN               0

/******************************************************************************************/

#define KEY1        digitalRead(KEY1_GPIO_PIN)              /* 读取KEY1引脚 */
#define KEY2        digitalRead(KEY2_GPIO_PIN)              /* 读取KEY2引脚 */
#define KEY_BOOT    digitalRead(KEY_BOOT_GPIO_PIN)          /* 读取KEY_BOOT引脚 */


#define KEY1_PRES        1              /* KEY0按下 */
#define KEY2_PRES        2              /* KEY1按下 */
#define KEY_BOOT_PRES    3              /* KEY2按下 */

void key_init(void);                    /* 按键初始化函数 */
uint8_t key_scan(uint8_t mode);         /* 按键扫描函数 */

#endif


