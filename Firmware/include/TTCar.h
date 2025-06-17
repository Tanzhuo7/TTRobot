/**
 ****************************************************************************************************
 * @file        TTCar.h
 * @author      TANZHUO
 * @version     V1.0
 * @date        2024-5-16
 * @brief       TTCar头文件
 ****************************************************************************************************
 */

#ifndef __TTCAR_H
#define __TTCAR_H
/******************************************** 头文件导入 ********************************************/

/* my Library include */
#include <Arduino.h>
#include "display.h"
#include "IMU.h"
#include "hc_sr04.h"
#include <esp32_motor_basic.h>
#include "key_scan.h"
#include "led.h"
#include "encoder.h"
#include "PidController.h"
#include <Kinematics.h>          
/* MICROROS Library include */
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
/* MICROROS_msg Library include */
#include <micro_ros_utilities/string_utilities.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <example_interfaces/msg/float64.h>

/******************************************** 宏定义 ************************************************/

/* Serial_Buad */
#define Serial_Speed        115200

/* PID */
#define KP                  1.500
#define KI                  0.350
#define KD                  0.000
#define PID_CONTROLLER_L    0
#define PID_CONTROLLER_R    1
#define PID_OUT_MIN         -1024
#define PID_OUT_MAX         1024

/* 电机ID */
#define MOTOR1              1//L
#define MOTOR2              2//R

/* L_MOTOR_PIN */    
#define MOTOR_L_ID          1
#define MOTOR_L_IN1         9
#define MOTOR_L_IN2         46
#define MOTOR_L_PWM         3

#define PCNTUNIT_L          0
#define ENCODE_L_A          16
#define ENCODE_L_B          15

/* R_MOTOR_PIN */  
#define MOTOR_R_ID          2
#define MOTOR_R_IN1         14
#define MOTOR_R_IN2         21
#define MOTOR_R_PWM         13

#define PCNTUNIT_R          1
#define ENCODE_R_A          36
#define ENCODE_R_B          41

/* Kinematics_Param */
#define KINEMATICS_L_ID     0
#define KINEMATICS_R_ID     1
#define REDUCATION_RATIO    30
#define PULSE_RATIO         44
#define WHEEL_DIAMETER      65
#define WHEEL_DISTANCE      180

#endif //__TTCAR_H


