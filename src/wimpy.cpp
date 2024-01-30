// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "wimpy_node/wimpy.hpp"

#include <memory>
#include <string>

using robotis::wimpy::Wimpy;
using namespace std::chrono_literals;

Wimpy::Wimpy(const std::string & usb_port)
: Node("wimpy_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init Wimpy Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
  odom_pub_    = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


  init_dynamixel_sdk_wrapper(usb_port);
  check_device_status();

  add_motors();
  add_wheels();
  add_devices();

  run();
}

Wimpy::Wheels * Wimpy::get_wheels()
{
  return &wheels_;
}

Wimpy::Motors * Wimpy::get_motors()
{
  return &motors_;
}

void Wimpy::init_dynamixel_sdk_wrapper(const std::string & usb_port)
{
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

  this->declare_parameter<uint8_t>("opencr.id");
  this->declare_parameter<int>("opencr.baud_rate");
  this->declare_parameter<float>("opencr.protocol_version");

  this->get_parameter_or<uint8_t>("opencr.id", opencr.id, 200);
  this->get_parameter_or<int>("opencr.baud_rate", opencr.baud_rate, 1000000);
  this->get_parameter_or<float>("opencr.protocol_version", opencr.protocol_version, 2.0f);

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

  dxl_sdk_wrapper_->init_read_memory(
    extern_control_table.millis.addr,
    (extern_control_table.profile_acceleration_rright.addr - extern_control_table.millis.addr) +
    extern_control_table.profile_acceleration_rright.length
  );
}

void Wimpy::check_device_status()
{
  if (dxl_sdk_wrapper_->is_connected_to_device()) {
    std::string sdk_msg;
    uint8_t reset = 1;

    dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.imu_re_calibration.addr,
      extern_control_table.imu_re_calibration.length,
      &reset,
      &sdk_msg);

    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
    rclcpp::shutdown();
    return;
  }

  const int8_t NOT_CONNECTED_MOTOR = -1;

  int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
    extern_control_table.device_status.addr,
    extern_control_table.device_status.length);

  switch (device_status) {
    case NOT_CONNECTED_MOTOR:
      RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
      break;

    default:
      break;
  }
}

void Wimpy::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter<float>("motors.profile_acceleration_constant");
  this->declare_parameter<float>("motors.profile_acceleration");

  this->get_parameter_or<float>(
    "motors.profile_acceleration_constant",
    motors_.profile_acceleration_constant,
    214.577);

  this->get_parameter_or<float>(
    "motors.profile_acceleration",
    motors_.profile_acceleration,
    0.0);
}

void Wimpy::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.lxy");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.lxy", wheels_.lxy, 0.495);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.076);
}

void Wimpy::add_devices()
{
  RCLCPP_INFO(this->get_logger(), "Add Devices");
  devices_["motor_power"] =
    new devices::MotorPower(node_handle_, dxl_sdk_wrapper_, "motor_power");
  devices_["reset"] =
    new devices::Reset(node_handle_, dxl_sdk_wrapper_, "reset");
}

void Wimpy::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  battery_ = std::make_unique<robotis::wimpy::sensors::BatteryState>(node_handle_,"battery_state");
  imu_     = std::make_unique<robotis::wimpy::sensors::Imu>(node_handle_,"imu","mag");

  sync_timer_call(std::chrono::milliseconds(10));
  publish_timer_battery(std::chrono::milliseconds(1000));
  publish_timer_imu(std::chrono::milliseconds(20));
  heartbeat_timer(std::chrono::milliseconds(100));
  Odom(std::chrono::milliseconds(20));
  cmd_vel_callback();
}

void Wimpy::sync_timer_call(const std::chrono::milliseconds timeout)
{
  sync_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      dxl_sdk_wrapper_->read_data_set();
    }
  );
}

void Wimpy::publish_timer_battery(const std::chrono::milliseconds timeout)
{
  battery_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();
      battery_->publish(now, dxl_sdk_wrapper_);
    }
  );
}


void Wimpy::publish_timer_imu(const std::chrono::milliseconds timeout)
{
  imu_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();
      imu_->publish(now, dxl_sdk_wrapper_);
    }
  );
}

void Wimpy::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      static uint8_t count = 0;
      std::string msg;

      dxl_sdk_wrapper_->set_data_to_device(
        extern_control_table.heartbeat.addr,
        extern_control_table.heartbeat.length,
        &count,
        &msg);

      RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

      count++;
    }
  );
}

void Wimpy::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      std::string sdk_msg;

      union Data {
        int32_t dword[6];
        uint8_t byte[4 * 6];
      } data;

      data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
      data.dword[1] = static_cast<int32_t>(msg->linear.y * 100);
      data.dword[2] = 0;
      data.dword[3] = 0;
      data.dword[4] = 0;
      data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

      uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
      uint16_t addr_length =
      (extern_control_table.cmd_velocity_angular_z.addr -
      extern_control_table.cmd_velocity_linear_x.addr) +
      extern_control_table.cmd_velocity_angular_z.length;

      uint8_t * p_data = &data.byte[0];

      dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

      RCLCPP_DEBUG(
        this->get_logger(),
        "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
    }
  );
}

void Wimpy::Odom(const std::chrono::milliseconds timeout)
{
  RCLCPP_INFO(this->get_logger(), "Odom pub started");
  //TODO use imu in orientation
  current_time = std::chrono::high_resolution_clock::now();
  last_time    = std::chrono::high_resolution_clock::now();

  odom_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {

      std::array<double, 4> velocity =
      {
      dxl_sdk_wrapper_->get_data_from_device<int32_t>(
          extern_control_table.present_velocity_fleft.addr,
          extern_control_table.present_velocity_fleft.length) * 0.229 * 0.1047 ,
      -1 * dxl_sdk_wrapper_->get_data_from_device<int32_t>(
          extern_control_table.present_velocity_fright.addr,
          extern_control_table.present_velocity_fright.length) * 0.229 * 0.1047 ,
      -1 * dxl_sdk_wrapper_->get_data_from_device<int32_t>(
          extern_control_table.present_velocity_rleft.addr,
          extern_control_table.present_velocity_rleft.length) * 0.229 * 0.1047 ,
      dxl_sdk_wrapper_->get_data_from_device<int32_t>(
          extern_control_table.present_velocity_rright.addr,
          extern_control_table.present_velocity_rright.length) * 0.229 * 0.1047 
      };

      current_time = std::chrono::high_resolution_clock::now();
      double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time-last_time).count() / 1e9;
      {
          this->velx_odom = (      velocity[0] + velocity[1] + velocity[2] + velocity[3] ) * (wheels_.radius * 0.25);
          this->vely_odom = ( -1 * velocity[0] + velocity[1] + velocity[2] - velocity[3] ) * (wheels_.radius * 0.25);
          this->velw_odom = ( -1 * velocity[0] + velocity[1] - velocity[2] + velocity[3] ) * (wheels_.radius / ( 4 * wheels_.lxy));
      }


      this->posx_odom += (this->velx_odom * std::cos(this->posw_odom) - this->vely_odom * std::sin(this->posw_odom)) * dt;
      this->posy_odom += (this->velx_odom * std::sin(this->posw_odom) + this->vely_odom * std::cos(this->posw_odom)) * dt;
      this->posw_odom += this->velw_odom * dt;

      this->last_time = this->current_time;

      message_odom.header.stamp =  this->get_clock()->now();
      message_odom.child_frame_id  = "azrael/base_footprint";
      message_odom.header.frame_id = "azrael/odom";
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, this->posw_odom);

      message_odom.pose.pose.orientation.x = q.x();
      message_odom.pose.pose.orientation.y = q.y();
      message_odom.pose.pose.orientation.z = q.z();
      message_odom.pose.pose.orientation.w = q.w();

      message_odom.pose.pose.position.x = this->posx_odom;
      message_odom.pose.pose.position.y = this->posy_odom;

      message_odom.twist.twist.linear.x  = this->velx_odom;
      message_odom.twist.twist.linear.y  = this->vely_odom;
      message_odom.twist.twist.angular.z = this->velw_odom;

      odom_pub_->publish(message_odom);

      geometry_msgs::msg::TransformStamped t;
      
      t.header.stamp = this->get_clock()->now();
      t.child_frame_id = "base_footprint";
      t.header.frame_id = "odom";
      

      t.transform.translation.x = this->posx_odom;
      t.transform.translation.y = this->posy_odom;

      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(t);

    }   
  );
}

// Wimpy::Battery()