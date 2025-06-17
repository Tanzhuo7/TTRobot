/**
 * @file fishbot_display.cpp
 * @author fishros (fishros@foxmail.com)
 * @brief FishBotOLED显示控制类
 * @version V1.0.0
 * @date 2023-01-05
 *
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 *
 */
#include "display.h"

void MyRoBotDisplay::init()
{
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN, 400000UL);
    _display = Adafruit_SSD1306(128, 64, &Wire);
    _display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR); // 设置OLED的I2C地址
    _display.clearDisplay();                    // 清空屏幕
    _display.setTextSize(1);                    // 设置字体大小
    _display.setTextColor(SSD1306_WHITE);       // 设置字体颜色
    _display.setCursor(0, 0);                   // 设置开始显示文字的坐标
    _display.println("  [MyRobot-v1.0.0]");     // 输出的字符
    _display.println("...");
    _display.println("");
    _display.println("connect agent...");
    _display.println("");
    _display.println("...");
    _display.display();
    
    /* Battery_Init */
    pinMode(OLED_BATTERY_PIN, INPUT);
    analogSetAttenuation(ADC_11db);
}

void MyRoBotDisplay::updateDisplay()
{
    if (millis() - last_update_time > update_interval)
    {
        last_update_time = millis();
        _display.clearDisplay();
        _display.setCursor(0, 0);
        _display.println("   -MyRobot-v1.0.0-");
        _display.print("microros:");
        _display.println(" OK ");
        _display.print("voltage :");
        updateBatteryInfo();
        _display.println(battery_info_);
        _display.print("distance:");
        _display.println(distance_oled_, 2);
        /* Debug_PID */
        _display.print("V:");
        _display.println(wheel_speed2_, 1);
        _display.print("OUT:");
        _display.println(PID_out2_, 1);
        _display.print("v:");
        _display.println(target_speed2_, 1);
        /* MOTOR_Speed */
        _display.setCursor(64, 32);
        _display.print("V:");
        _display.println(wheel_speed1_, 1);
        _display.setCursor(64, 48);
        _display.print("v:");
        _display.println(target_speed1_, 1);
        _display.setCursor(64, 40);
        _display.print("OUT: ");
        _display.println(PID_out1_, 1);

        _display.display();
    }
}
void MyRoBotDisplay::updateBatteryInfo(void)
{
    int analogVolts = analogReadMilliVolts(OLED_BATTERY_PIN);           // 读取模拟电压，单位毫伏
    float realVolts = 11.989 * ((float)analogVolts * 1e-3);             // 计算实际电压值
    battery_info_ = realVolts;
}
void MyRoBotDisplay::updateUltrasoundDist(float &ultrasound_distance)
{
    ultrasound_distance_ = ultrasound_distance;
}
void MyRoBotDisplay::updateBotAngular(float &bot_angular)
{
    bot_angular_ = bot_angular;
}
void MyRoBotDisplay::updateBotLinear(float &bot_linear)
{
    bot_linear_ = bot_linear;
}
void MyRoBotDisplay::updateTransMode(String mode)
{
    mode_ = mode;
}
void MyRoBotDisplay::updateWIFIIp(String ip)
{
    if (ip != ip_)
    {
        ip_ = ip;
    }
}
void MyRoBotDisplay::updateCurrentTime(int64_t current_time_)
{
    current_time = current_time_;
}
String MyRoBotDisplay::twoDigits(int digits)
{
    if (digits < 10)
    {
        String i = '0' + String(digits);
        return i;
    }
    else
    {
        return String(digits);
    }
}

void MyRoBotDisplay::updateBaudRate(uint32_t baudrate)
{
    baudrate_ = baudrate;
}

void MyRoBotDisplay::updateStartupInfo()
{
    // String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
    // last_update_time = millis();
    // _display.clearDisplay();
    // _display.setCursor(0, 0);
    // _display.println("   -fishbot-v1.0.0-");
    // _display.print("microros:");
    // _display.println(mode_);
    // _display.print("voltage :");
    // _display.println(battery_info_);
    // _display.display();
}

void MyRoBotDisplay::updateDisplayMode(uint8_t display_mode)
{
    display_mode_ = display_mode;
}

void MyRoBotDisplay::updateWIFISSID(String ssid)
{
    wifi_ssid_ = ssid;
}
void MyRoBotDisplay::updateWIFIPSWD(String pswd)
{
    wifi_pswd_ = pswd;
}

void MyRoBotDisplay::updataDebugPID(float &kp, float &ki, float &kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void MyRoBotDisplay::updataDebug_wheel_speed(float wheel_speed1, float wheel_speed2)
{
    wheel_speed1_ = wheel_speed1;
    wheel_speed2_ = wheel_speed2;
}

void MyRoBotDisplay::updataDebug_PID_OUT(float pid_out1, float pid_out2)
{
    PID_out1_ = pid_out1;
    PID_out2_ = pid_out2;
}

void MyRoBotDisplay::updateDebug_target_speed(float target_speed1, float target_speed2)
{
    target_speed1_ = target_speed1;
    target_speed2_ = target_speed2;
}

void MyRoBotDisplay::updateDebug_distance(float distance)
{
    distance_oled_ = distance;
}