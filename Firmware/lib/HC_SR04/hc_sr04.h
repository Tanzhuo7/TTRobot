#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#include <Arduino.h>

/* 引脚定义 */
#define Trig 42 // 设定SR04连接的Arduino引脚
#define Echo 40

/* 函数声明 */
void HC_SR04_init(void);             /* HC_SR04模块初始化 */
float HC_SR04__loop(void);           /* HC_SR04测距 */

#endif //HC_SR04