#include "hc_sr04.h"
#include "Arduino.h"

/**
 * @brief       超声波模块初始化
 * @param       无
 * @retval      无
 */
void HC_SR04_init(void)
{
    pinMode(Trig, OUTPUT); // 初始化舵机和超声波
    pinMode(Echo, INPUT);  // 要检测引脚上输入的脉冲宽度，需要先设置为输入状态
}

/**
 * @brief       超声波测距
 * @param       无
 * @retval      距离（cm）
 */
float HC_SR04__loop(void)
{
    static double mtime;
    digitalWrite(Trig, LOW); // 测量距离
    delayMicroseconds(2);    // 延时2us
    digitalWrite(Trig, HIGH); 
    delayMicroseconds(10); // 产生一个10us的高脉冲去触发SR04
    digitalWrite(Trig, LOW);
    mtime = pulseIn(Echo, HIGH);                  // 检测脉冲宽度，注意返回值是微秒us
    float detect_distance = mtime / 58.0 / 100.0 * 100; // 计算出距离,输出的距离的单位是厘米cm
    return detect_distance;
}