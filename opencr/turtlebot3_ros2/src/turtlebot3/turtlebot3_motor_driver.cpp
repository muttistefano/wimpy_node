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

#include "../../include/turtlebot3/turtlebot3_motor_driver.h"

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 150; 
const float VELOCITY_CONSTANT_VALUE = 41.7; 

/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_FL = 1; // ID of left motor
const uint8_t DXL_MOTOR_ID_FR = 2; // ID of right motor
const uint8_t DXL_MOTOR_ID_RL = 3; // ID of right motor
const uint8_t DXL_MOTOR_ID_RR = 4; // ID of right motor
const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);



Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: fl_wheel_id_(DXL_MOTOR_ID_FL),
  fr_wheel_id_(DXL_MOTOR_ID_FR),
  rl_wheel_id_(DXL_MOTOR_ID_RL),
  rr_wheel_id_(DXL_MOTOR_ID_RR),
  torque_(false)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool Turtlebot3MotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  sync_write_param.id_count = 4;
  sync_write_param.xel[0].id = fl_wheel_id_;
  sync_write_param.xel[1].id = fr_wheel_id_;
  sync_write_param.xel[2].id = rl_wheel_id_;
  sync_write_param.xel[3].id = rr_wheel_id_;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = 4;
  sync_read_param.xel[0].id = fl_wheel_id_;
  sync_read_param.xel[1].id = fr_wheel_id_;
  sync_read_param.xel[2].id = rl_wheel_id_;
  sync_read_param.xel[3].id = rr_wheel_id_;

  // Enable Dynamixel Torque
  set_torque(true);

  return true;
}

Dynamixel2Arduino& Turtlebot3MotorDriver::getDxl()
{
  return dxl;
}

bool Turtlebot3MotorDriver::is_connected()
{
  return (dxl.ping(DXL_MOTOR_ID_FL) == true && dxl.ping(DXL_MOTOR_ID_FR) == true && dxl.ping(DXL_MOTOR_ID_RL) == true && dxl.ping(DXL_MOTOR_ID_RR) == true);
}

bool Turtlebot3MotorDriver::set_torque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.xel[0].data[0] = onoff;
  sync_write_param.xel[1].data[0] = onoff;
  sync_write_param.xel[2].data[0] = onoff;
  sync_write_param.xel[3].data[0] = onoff;

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

bool Turtlebot3MotorDriver::get_torque()
{
  if(dxl.readControlTableItem(TORQUE_ENABLE, fl_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, fr_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, rl_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, rr_wheel_id_) == true){
    torque_ = true;
  }else{
    torque_ = false;
  }

  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool Turtlebot3MotorDriver::read_present_position(int32_t &fleft_value, int32_t &fright_value , int32_t &rleft_value, int32_t &rright_value)
{
  bool ret = false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&fleft_value, read_result.xel[0].data, read_result.xel[0].length);
    memcpy(&fright_value, read_result.xel[1].data, read_result.xel[1].length);
    memcpy(&rleft_value, read_result.xel[2].data, read_result.xel[2].length);
    memcpy(&rright_value, read_result.xel[3].data, read_result.xel[3].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_velocity(int32_t &fleft_value, int32_t &fright_value , int32_t &rleft_value, int32_t &rright_value)
{
  bool ret = false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&fleft_value , read_result.xel[0].data, read_result.xel[0].length);
    memcpy(&fright_value, read_result.xel[1].data, read_result.xel[1].length);
    memcpy(&rleft_value , read_result.xel[2].data, read_result.xel[2].length);
    memcpy(&rright_value, read_result.xel[3].data, read_result.xel[3].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_current(int16_t &fleft_value, int16_t &fright_value , int16_t &rleft_value, int16_t &rright_value)
{
  bool ret = false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&fleft_value, read_result.xel[0].data, read_result.xel[0].length);
    memcpy(&fright_value, read_result.xel[1].data, read_result.xel[1].length);
    memcpy(&rleft_value, read_result.xel[2].data, read_result.xel[2].length);
    memcpy(&rright_value, read_result.xel[3].data, read_result.xel[3].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_profile_acceleration(uint32_t &fleft_value, uint32_t &fright_value, uint32_t &rleft_value, uint32_t &rright_value)
{
  bool ret = false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&fleft_value, read_result.xel[0].data, read_result.xel[0].length);
    memcpy(&fright_value, read_result.xel[1].data, read_result.xel[1].length);
    memcpy(&rleft_value, read_result.xel[2].data, read_result.xel[2].length);
    memcpy(&rright_value, read_result.xel[3].data, read_result.xel[3].length);
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::write_velocity(int32_t fleft_value, int32_t fright_value, int32_t rleft_value, int32_t rright_value)
{
  bool ret = false;

  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[0].data, &fleft_value, sync_write_param.length);
  memcpy(sync_write_param.xel[1].data, &fright_value, sync_write_param.length);
  memcpy(sync_write_param.xel[2].data, &rleft_value, sync_write_param.length);
  memcpy(sync_write_param.xel[3].data, &rright_value, sync_write_param.length);
  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::write_profile_acceleration(uint32_t fleft_value, uint32_t fright_value, uint32_t rleft_value, uint32_t rright_value)
{
  bool ret = false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[0].data, &fleft_value, sync_write_param.length);
  memcpy(sync_write_param.xel[1].data, &fright_value, sync_write_param.length);
  memcpy(sync_write_param.xel[2].data, &rleft_value, sync_write_param.length);
  memcpy(sync_write_param.xel[3].data, &rright_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::control_motors(const float wheel_separation,const float wheel_radius, float linear_value_x,float linear_value_y, float angular_value)
{
  bool dxl_comm_result = false;
  
  const double angularLength = 0.5 * (wheel_separation);
  const double invWheelRadius = 1 / wheel_radius;



  float wheel_velocity[MortorLocation::MOTOR_NUM_MAX];

  wheel_velocity[0] = (linear_value_x - linear_value_y - angular_value * angularLength) * invWheelRadius;
  wheel_velocity[1] = -1 * (linear_value_x + linear_value_y + angular_value * angularLength) * invWheelRadius;
  wheel_velocity[2] = -1 * (linear_value_x + linear_value_y - angular_value * angularLength) * invWheelRadius;
  wheel_velocity[3] = (linear_value_x - linear_value_y + angular_value * angularLength) * invWheelRadius;

  wheel_velocity[0] = constrain(wheel_velocity[0] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[1] = constrain(wheel_velocity[1] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[2] = constrain(wheel_velocity[2] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[3] = constrain(wheel_velocity[3] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  dxl_comm_result = write_velocity((int32_t)wheel_velocity[0], (int32_t)wheel_velocity[1], (int32_t)wheel_velocity[2], (int32_t)wheel_velocity[3]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
