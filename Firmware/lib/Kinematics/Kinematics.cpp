#include "Kinematics.h"

void Kinematics::set_motor_param(uint8_t id, uint16_t reducation_ratio, uint16_t pulse_ration, float wheel_diameter)
{
    motor_param_[id].id = id;   // 设置电机ID
    motor_param_[id].reducation_ratio = reducation_ratio;   // 设置减速比
    motor_param_[id].pulse_ration = pulse_ration;   // 设置脉冲比
    motor_param_[id].wheel_diameter = wheel_diameter;   // 设置车轮直径
    motor_param_[id].per_pulse_distance = (wheel_diameter * PI) / (reducation_ratio * pulse_ration);   // 每个脉冲对应行驶距离
    motor_param_[id].speed_factor = (1000 * 1000) * (wheel_diameter * PI) / (reducation_ratio * pulse_ration);   // 计算速度因子
    // Serial.printf("init motor param %d: %f=%f*PI/(%d*%d) speed_factor=%d\n", id, motor_param_[id].per_pulse_distance, wheel_diameter, reducation_ratio, pulse_ration, motor_param_[id].speed_factor);   // 打印调试信息
}

void Kinematics::set_kinematic_param(float wheel_distance)
{
    wheel_distance_ = wheel_distance;   // 设置轮间距离
}

void Kinematics::update_motor_ticks(uint64_t current_time, int32_t motor_tick1, int32_t motor_tick2)
{

    uint32_t dt = current_time - motor_param_[0].last_update_time;   // 计算时间差
    int32_t dtick1 = motor_tick1 - motor_param_[0].last_encoder_tick;   // 计算电机1脉冲差
    int32_t dtick2 = motor_tick2 - motor_param_[1].last_encoder_tick;   // 计算电机2脉冲差
    // 轮子速度计算
    motor_param_[0].motor_speed = dtick1 * (motor_param_[0].speed_factor / dt);   // 计算电机1轮子速度
    motor_param_[1].motor_speed = dtick2 * (motor_param_[1].speed_factor / dt);   // 计算电机2轮子速度

    motor_param_[0].last_encoder_tick = motor_tick1;   // 更新电机1上一次的脉冲计数
    motor_param_[1].last_encoder_tick = motor_tick2;   // 更新电机2上一次的脉冲计数
    motor_param_[0].last_update_time = current_time;   // 更新电机1上一次更新时间
    motor_param_[1].last_update_time = current_time;   // 更新电机2上一次更新时间

    // 更新机器人里程计
    this->update_bot_odom(dt);
}

void Kinematics::kinematic_inverse(float linear_speed, float angular_speed, float &out_wheel1_speed, float &out_wheel2_speed)
{
    out_wheel1_speed =
        linear_speed - (angular_speed * wheel_distance_) / 2.0;
    out_wheel2_speed =
        linear_speed + (angular_speed * wheel_distance_) / 2.0;
}


void Kinematics::kinematic_forward(float wheel1_speed, float wheel2_speed, float &linear_speed, float &angular_speed)
{
    linear_speed = (wheel1_speed + wheel2_speed) / 2.0;                 // 计算线速度
    angular_speed = (wheel2_speed - wheel1_speed) / wheel_distance_;    // 计算角速度
}

float Kinematics::motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed; // 返回指定id的轮子速度
} 

void Kinematics::update_bot_odom(uint32_t dt)
{
    static float linear_speed, angular_speed;
    float dt_s = (float)(dt / 1000) / 1000;

    this->kinematic_forward(motor_param_[0].motor_speed, motor_param_[1].motor_speed, linear_speed, angular_speed);

    odom_.angular_speed = angular_speed;
    odom_.linear_speed = linear_speed / 1000; // /1000（mm/s 转 m/s）

    odom_.yaw += odom_.angular_speed * dt_s;

    Kinematics::TransAngleInPI(odom_.yaw, odom_.yaw);
    

    /*更新x和y轴上移动的距离*/
    float delta_distance = odom_.linear_speed * dt_s; // 单位m
    odom_.x += delta_distance * std::cos(odom_.yaw);
    odom_.y += delta_distance * std::sin(odom_.yaw);

}

void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    if (angle > PI)
    {
        out_angle -= 2 * PI;
    }
    else if (angle < -PI)
    {
        out_angle += 2 * PI;
    }
}

void Kinematics::Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q)
{
    // 传入机器人的欧拉角 roll、pitch 和 yaw。
    // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中    
    // https://blog.csdn.net/xiaoma_bk/article/details/79082629
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

odom_t &Kinematics::odom()
{
    // 调用 Euler2Quaternion 函数，将机器人的欧拉角 yaw 转换为四元数 quaternion。
    Kinematics::Euler2Quaternion(0, 0, odom_.yaw, odom_.quaternion);
    return odom_;
}
