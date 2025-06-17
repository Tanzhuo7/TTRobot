#ifndef __DISPLAY_H__
#define __DISPLAT_H__

#include <Arduino.h>
#include "PidController.h"
#include <Wire.h>             // 加载Wire库
#include <Adafruit_GFX.h>     // 加载Adafruit_GFX库
#include <Adafruit_SSD1306.h> // 加载Adafruit_SSD1306库

/* 引脚定义 */
#define OLED_SDA_PIN      38
#define OLED_SCL_PIN      39
#define OLED_BATTERY_PIN  4

#define OLED_ADDR         0X3C  /* 7位器件地址 */

class MyRoBotDisplay
{
private:
    Adafruit_SSD1306 _display;

    float battery_info_;
    float ultrasound_distance_;
    float bot_angular_;
    uint32_t baudrate_;
    String mode_;
    String ip_="wait connect!";
    uint8_t display_mode_;

    int64_t current_time ;
    uint64_t last_update_time;
    uint64_t update_interval{100};

    String wifi_ssid_;
    String wifi_pswd_;

    float kp_;
    float ki_;
    float kd_;

    float wheel_speed1_,wheel_speed2_;
    float PID_out1_,PID_out2_;
    float target_speed1_,target_speed2_;

    float distance_oled_;
    
public:
    float bot_linear_;
    void init();
    void updateDisplayMode(uint8_t display_mode);
    void updateDisplay();
    void updateStartupInfo();
    void updateBatteryInfo(void);
    void updateUltrasoundDist(float &ultrasound_distance);
    void updateBotAngular(float &bot_angular);
    void updateBotLinear(float &bot_linear);
    void updateTransMode(String mode);
    void updateCurrentTime(int64_t current_time_);
    void updateBaudRate(uint32_t baudrate);
    void updateWIFIIp(String ip);
    void updateWIFISSID(String ssid);
    void updateWIFIPSWD(String pswd);
    String twoDigits(int digits);

    void updataDebugPID(float &kp, float &ki, float &kd);
    void updataDebug_wheel_speed(float wheel_speed1, float wheel_speed2);
    void updataDebug_PID_OUT(float pid_out1, float pid_out2);
    void updateDebug_target_speed(float target_speed1, float target_speed2);
    void updateDebug_distance(float distance);
};

#endif