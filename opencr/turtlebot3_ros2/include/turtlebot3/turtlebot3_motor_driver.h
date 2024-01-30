/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include <Dynamixel2Arduino.h>

#define TORQUE_ENABLE ControlTableItem::TORQUE_ENABLE

enum MortorLocation{
  FL = 0,
  FR,
  RL,
  RR,
  MOTOR_NUM_MAX
};

enum VelocityType{
  LINEAR_X = 0,
  LINEAR_Y,
  ANGULAR,
  TYPE_NUM_MAX
};


class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  
  bool init(void);
  void close(void);

  bool is_connected();

  bool set_torque(bool onoff);
  bool get_torque();
  
  bool read_present_position(int32_t &fleft_value, int32_t &fright_value , int32_t &rleft_value, int32_t &rright_value);
  bool read_present_velocity(int32_t &fleft_value, int32_t &fright_value , int32_t &rleft_value, int32_t &rright_value);
  bool read_present_current(int16_t &fleft_value, int16_t &fright_value , int16_t &rleft_value, int16_t &rright_value);
  bool read_profile_acceleration(uint32_t &fleft_value, uint32_t &fright_value, uint32_t &rleft_value, uint32_t &rright_value);
  
  bool write_velocity(int32_t fleft_value, int32_t fright_value, int32_t rleft_value, int32_t rright_value);
  bool write_profile_acceleration(uint32_t fleft_value, uint32_t fright_value, uint32_t rleft_value, uint32_t rright_value);

  bool control_motors(const float wheel_separation,const float wheel_radius, float linear_value_x,float linear_value_y, float angular_value);

  Dynamixel2Arduino& getDxl();
  
 private:
  uint8_t fl_wheel_id_;
  uint8_t fr_wheel_id_;
  uint8_t rl_wheel_id_;
  uint8_t rr_wheel_id_;
  bool torque_;
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
