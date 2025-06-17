  // key_num = key_scan(0);
  // if(key_num == KEY1_PRES)
  // {
  //   LED(1);
  //   motor_L.updateMotorSpeed(MOTOR1, 1000);
  //   motor_R.updateMotorSpeed(MOTOR2, 1000);
  // }
  // else if(key_num == KEY2_PRES)
  // {
  //   LED(0);
  //   motor_L.updateMotorSpeed(MOTOR1, -1000);
  //   motor_R.updateMotorSpeed(MOTOR2, -1000);
  // }
  // else if(key_num == KEY_BOOT_PRES)
  // {
  //   motor_L.updateMotorSpeed(MOTOR1, 0);
  //   motor_R.updateMotorSpeed(MOTOR2, 0);
  // }

  /* PID_Debug */
  /*
    key_num = key_scan(0);
  if(key_num == KEY1_PRES)
  {
    if(mode_pid == 0)
    {
      kp += 0.5;
    }
    else if(mode_pid == 1)
    {
      ki += 0.05;
    }
    else if (mode_pid == 2)
    {
      kd += 0.05; 
    }
  }
  else if(key_num == KEY2_PRES)
  {
    if(mode_pid == 0)
    {
      kp -= 0.5;
    }
    else if(mode_pid == 1)
    {
      ki -= 0.05;
    }
    else if (mode_pid == 2)
    {
      kd -= 0.05; 
    }
  }
  else if(key_num == KEY_BOOT_PRES)
  {
    mode_pid++;
    if(mode_pid >= 4)
    {
      mode_pid = 0;
    }
    else if(mode_pid == 3)
    {
      pid_controller[0].update_pid(kp, ki, kd);
      pid_controller[1].update_pid(kp, ki, kd);
      pid_controller[0].out_limit(-1000, 1000);
      pid_controller[1].out_limit(-1000, 1000);   
      LED_TOGGLE();
    }
  }
  */