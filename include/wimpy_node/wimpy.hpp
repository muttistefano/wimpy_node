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

#ifndef WIMPY_NODE__WIMPY_HPP_
#define WIMPY_NODE__WIMPY_HPP_

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlebot3_msgs/msg/sensor_state.hpp>

#include "wimpy_node/control_table.hpp"
#include "wimpy_node/devices/devices.hpp"
#include "wimpy_node/devices/motor_power.hpp"
#include "wimpy_node/devices/reset.hpp"
#include "wimpy_node/devices/sound.hpp"
#include "wimpy_node/dynamixel_sdk_wrapper.hpp"
#include "wimpy_node/sensors/battery_state.hpp"
#include "wimpy_node/sensors/imu.hpp"
#include "wimpy_node/sensors/sensor_state.hpp"
#include "wimpy_node/sensors/sensors.hpp"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"

namespace robotis
{
namespace wimpy
{
extern const ControlTable extern_control_table;
class Wimpy : public rclcpp::Node
{
public:
  typedef struct
  {
    float lxy;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  explicit Wimpy(const std::string & usb_port);
  virtual ~Wimpy() {}

  Wheels * get_wheels();
  Motors * get_motors();

private:
  void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  void check_device_status();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();

  void run();
  void Odom(const std::chrono::milliseconds timeout);

  void sync_timer_call(const std::chrono::milliseconds timeout);
  void publish_timer_battery(const std::chrono::milliseconds timeout);
  void publish_timer_imu(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;

  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
  std::shared_ptr<robotis::wimpy::sensors::BatteryState> battery_;
  std::shared_ptr<robotis::wimpy::sensors::Imu> imu_;

  std::list<sensors::Sensors *> sensors_;
  std::map<std::string, devices::Devices *> devices_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::chrono::time_point<std::chrono::high_resolution_clock> current_time ;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_time    ;
  nav_msgs::msg::Odometry message_odom ;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;


  double velx_odom = 0.0;
  double vely_odom = 0.0;
  double velw_odom = 0.0;

  double posx_odom = 0.0;
  double posy_odom = 0.0;
  double posw_odom = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
}  // namespace wimpy
}  // namespace robotis
#endif  // WIMPY_NODE__WIMPY_HPP_
