/******************************************************************************
 * Project: SerialPort
 * File: mian.cpp
 * Description: NULL
 * Author: Tan Zhuo
 * Date Created: Dec 18 2024
 * Version: V1.0
 * Dependencies: NULL
 * License: NULL
 *****************************************************************************/
#include "TTCar.h"
#define DEBUG 1 

#if DEBUG
/* MICOROS_Config */
rclc_executor_t executor;                 // 创建一个 RCLC 执行程序对象，用于处理订阅和发布
rclc_support_t support;                   // 创建一个 RCLC 支持对象，用于管理 ROS2 上下文和节点
rcl_allocator_t allocator;                // 创建一个 RCL 分配器对象，用于分配内存
rcl_node_t node;                          // 创建一个 RCL 节点对象，用于此基于 ESP32 的机器人小车
rcl_subscription_t subscriber;            // 创建一个 RCL 订阅对象，用于订阅 ROS2 消息
geometry_msgs__msg__Twist sub_msg;        // 创建一个 ROS2 geometry_msgs/Twist 消息对象
rcl_publisher_t current_speed_publisher;
rcl_publisher_t target_speed_publisher;
rcl_publisher_t odom_publisher;
example_interfaces__msg__Float64 current_speed_msg;
example_interfaces__msg__Float64 target_speed_msg;
nav_msgs__msg__Odometry odom_msg;
#endif

/* Class_Init */
Esp32_DCMotor motor;
Esp32PcntEncoder encoders[2];      
PidController pid_controller[2]; 
Kinematics kinematics;     
MyRoBotDisplay OLED;       

/* variable_Init */
float out_motor_speed[2];          // 创建一个长度为 2 的浮点数数组，用于保存输出电机速度
float target_speed;
unsigned long previousMillis;
unsigned long interval = 50;
float distance;

bool state;

#if DEBUG
/* 回调函数，当接收到新的 Twist 消息时会被调用 */
void twist_callback(const void *msg_in)
{
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
  static float target_motor_speed1, target_motor_speed2; 
  float linear_x = twist_msg->linear.x;  
  float angular_z = twist_msg->angular.z;       
  kinematics.kinematic_inverse(linear_x * 1000, angular_z, target_motor_speed1, target_motor_speed2);

  pid_controller[PID_CONTROLLER_L].update_target(target_motor_speed1); 
  pid_controller[PID_CONTROLLER_R].update_target(target_motor_speed2);

  OLED.updateDebug_target_speed(target_motor_speed1, target_motor_speed2);
}
#endif

#if DEBUG
/* micro-ROS Agent */
void microros_task(void *param)
{
  /* BSP_Init */
  OLED.init();
  led_init();

  /* micro-ROS Agent_IP_config */
  IPAddress agent_ip;
  agent_ip.fromString("192.168.43.42");
  
  /* WiFi config */
  set_microros_wifi_transports("TT", "00998877", agent_ip, 8888);

  /* odom_msg */
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");

  /* wait_Init... */
  delay(2000);
  
  /* 设置 micro-ROS 支持结构、节点和订阅 */
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "TTCar", "", &support);
  
  rclc_publisher_init_best_effort(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom");

  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");
  
  /* 设置 micro-ROS 执行器，并将订阅添加到其中 */ 
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);
  
  while (true)
  {
    if (!rmw_uros_epoch_synchronized())
    {
      // 如果时间同步成功，则将当前时间设置为MicroROS代理的时间，并输出调试信息。
      rmw_uros_sync_session(1000);
      delay(10);
    }

    /* OLED_loop*/
    OLED.updataDebug_wheel_speed(kinematics.motor_speed(KINEMATICS_L_ID), kinematics.motor_speed(KINEMATICS_R_ID));
    OLED.updataDebug_PID_OUT(out_motor_speed[0], out_motor_speed[1]);
    distance = HC_SR04__loop();
    OLED.updateDebug_distance(distance);
    OLED.updateDisplay();
    LED_TOGGLE();

    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  }
}
#endif

void setup() {
  led_init();
  /* Serial_Init */
  Serial.begin(Serial_Speed); 
  Serial2.begin(115200, SERIAL_8N1, 18, 17);

  /* Bsp_Init */
  //HC_SR04_init();

  /* MOTOR_Init：MOTOR_L,MOTOR_R */
  motor.attachMotor(MOTOR_L_ID, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_PWM);
  motor.attachMotor(MOTOR_R_ID, MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_PWM);

  motor.stopMotor(MOTOR_L_ID);
  motor.stopMotor(MOTOR_R_ID);

  /* Encoder_Init */
  encoders[PCNTUNIT_L].init(PCNTUNIT_L, ENCODE_L_A, ENCODE_L_B);
  encoders[PCNTUNIT_R].init(PCNTUNIT_R, ENCODE_R_A, ENCODE_R_B);

  /* PID_Controller_Init */
  pid_controller[PID_CONTROLLER_L].update_pid(KP, KI, KD);
  pid_controller[PID_CONTROLLER_R].update_pid(KP, KI, KD);
  pid_controller[PID_CONTROLLER_L].out_limit(PID_OUT_MIN, PID_OUT_MAX);
  pid_controller[PID_CONTROLLER_R].out_limit(PID_OUT_MIN, PID_OUT_MAX);

  /* kinematics_param_Init */
  kinematics.set_motor_param(KINEMATICS_L_ID, REDUCATION_RATIO, PULSE_RATIO, WHEEL_DIAMETER);
  kinematics.set_motor_param(KINEMATICS_R_ID, REDUCATION_RATIO, PULSE_RATIO, WHEEL_DIAMETER);
  kinematics.set_kinematic_param(WHEEL_DISTANCE);

  // Core 0 Task_Init
  //xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}

void loop() 
{
#if DEBUG
  static uint64_t last_update_info_time = millis();
  kinematics.update_motor_ticks(micros(), encoders[PCNTUNIT_L].getTicks(), encoders[PCNTUNIT_R].getTicks());
  out_motor_speed[0] = pid_controller[PID_CONTROLLER_L].update(kinematics.motor_speed(KINEMATICS_L_ID));
  out_motor_speed[1] = pid_controller[PID_CONTROLLER_R].update(kinematics.motor_speed(KINEMATICS_R_ID));
  if(distance <= 18.00 && distance > 1)//超声波避障
  {
    motor.updateMotorSpeed(MOTOR_L_ID, 300);
    motor.updateMotorSpeed(MOTOR_R_ID, 0);
  }
  else
  {
    motor.updateMotorSpeed(MOTOR_L_ID, out_motor_speed[0]);
    motor.updateMotorSpeed(MOTOR_R_ID, out_motor_speed[1]);
  }
  // motor.updateMotorSpeed(MOTOR_L_ID, out_motor_speed[0]);
  // motor.updateMotorSpeed(MOTOR_R_ID, out_motor_speed[1]);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {                                 
    previousMillis = currentMillis; // 记录上一次打印的时间
    float linear_speed, angle_speed;
    kinematics.kinematic_forward(kinematics.motor_speed(0), kinematics.motor_speed(1), linear_speed, angle_speed);

    Serial.printf("[%ld] linear:%f angle:%f\n", currentMillis, linear_speed, angle_speed);                       // 打印当前时间
    Serial.printf("[%ld] x:%f y:%f yaml:%f\n", currentMillis,kinematics.odom().x, kinematics.odom().y, kinematics.odom().yaw); // 打印当前时间
    int64_t stamp = rmw_uros_epoch_millis();
    odom_t odom = kinematics.odom();
    odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 秒部分
    odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分
    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;
    odom_msg.pose.pose.orientation.w = odom.quaternion.w;
    odom_msg.pose.pose.orientation.x = odom.quaternion.x;
    odom_msg.pose.pose.orientation.y = odom.quaternion.y;
    odom_msg.pose.pose.orientation.z = odom.quaternion.z;

    odom_msg.twist.twist.angular.z = odom.angular_speed;
    odom_msg.twist.twist.linear.x = odom.linear_speed;

    rcl_publish(&odom_publisher, &odom_msg, NULL);
  }
#else
  /* begin */
  // static uint64_t last_update_info_time = millis();
  // kinematics.update_motor_ticks(micros(), encoders[PCNTUNIT_L].getTicks(), encoders[PCNTUNIT_R].getTicks());
  // out_motor_speed[0] = pid_controller[PID_CONTROLLER_L].update(kinematics.motor_speed(KINEMATICS_L_ID));
  // out_motor_speed[1] = pid_controller[PID_CONTROLLER_R].update(kinematics.motor_speed(KINEMATICS_R_ID));

  // motor.updateMotorSpeed(MOTOR_L_ID, out_motor_speed[0]);
  // motor.updateMotorSpeed(MOTOR_R_ID, out_motor_speed[1]);

  //Serial.println("I am Serial");
  // if (Serial2.available()) {
  //   int data = Serial2.read();  // 读取数据
  //   Serial2.println("翻转LED");
  //   // Serial2.print("接收数据: ");
  //   // Serial2.println(data);

  //   if(data == 49)//1
  //     LED_TOGGLE();
  //   else if (data == 50 && state == 0){//2 false
  //     motor.updateMotorSpeed(MOTOR_L_ID, -500);
  //     motor.updateMotorSpeed(MOTOR_R_ID, -500);
  //     state = 1;
  //     Serial2.println("前进");
  //   }
  //   else if (data == 50 && state == 1){//2 true
  //     motor.updateMotorSpeed(MOTOR_L_ID, 0);
  //     motor.updateMotorSpeed(MOTOR_R_ID, 0);
  //     state = 0;
  //     Serial2.println("停车");
  //   }
  // }

  // if(state == 1)
  //   LED_TOGGLE();
  LED_TOGGLE();

#endif
  delay(200);
}






