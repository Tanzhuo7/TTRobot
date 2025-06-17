/**
 ****************************************************************************************************
 * @file        key_scan.c
 * @author      TANZHUO
 * @version     V1.0
 * @date        2024-5-7
 * @brief       按键输入 驱动代码
 ****************************************************************************************************
 */

#include "key_scan.h"

/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    pinMode(KEY1_GPIO_PIN, INPUT_PULLUP);
    pinMode(KEY2_GPIO_PIN, INPUT_PULLUP);
    pinMode(KEY_BOOT_GPIO_PIN, INPUT_PULLUP);
}

/**
 * @brief       按键扫描函数
 * @note        该函数有响应优先级(同时按下多个按键): WK_UP > KEY2 > KEY1 > KEY0!!
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              KEY1_PRES, 2, KEY1按下
 *              KEY2_PRES, 3, KEY2按下
 *              WKUP_PRES, 4, WKUP按下
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* 按键按松开标志 */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* 支持连按 */

    if (key_up && (KEY1 == 0 || KEY2 == 0 || KEY_BOOT == 0))  /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
        delay(10);           /* 去抖动 */
        key_up = 0;

        if (KEY1 == 0)      keyval = KEY1_PRES;

        if (KEY2 == 0)      keyval = KEY2_PRES;

        if (KEY_BOOT == 0)  keyval = KEY_BOOT_PRES;
    }
    else if (KEY1 == 1 && KEY2 == 1 && KEY_BOOT == 1)         /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;              /* 返回键值 */
}

