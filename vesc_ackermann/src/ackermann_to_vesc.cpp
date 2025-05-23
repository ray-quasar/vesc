// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // declare parameters
  declare_parameter("speed_to_erpm_gain", 0.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 0.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);
  declare_parameter("vel_diff_thresh", 0.1); 

  // get conversion parameters
  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
  steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
  steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();


  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // get vel diff parameter
  vel_diff_thresh_ = get_parameter("vel_diff_thresh").get_value<double>();

  // create brake publisher
  brake_pub_ = create_publisher<Float64>("commands/motor/brake", 10);
  current_vel_ = 0.0;

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  // subscribe to odom topic
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&AckermannToVesc::odomCallback, this, _1));

}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  double commanded_vel = cmd->drive.speed;

  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * commanded_vel + speed_to_erpm_offset_;

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // brake msg
  Float64 brake_msg;
  // brake_msg.data = 20000;
  //brake_msg.data = 10000;

  // We wanto linearly increase the brake value from 0 to 20000
  // if the commanded velocity is less than the current velocity
  // The brake at at 0.5 difference between the commanded and current velocity should be 10000
  // The brake at at 1.0 difference between the commanded and current velocity should be 20000

  brake_msg.data = (current_vel_ - commanded_vel) * 20000;
  if (brake_msg.data < 0) {
    brake_msg.data = 0;
  } else if (brake_msg.data > 20000) {
    brake_msg.data = 20000;
  }

  // publish (original code)
  // if (rclcpp::ok()) {
  //   erpm_pub_->publish(erpm_msg);
  //   servo_pub_->publish(servo_msg);
  // }

  // publish (new code)
  if (rclcpp::ok()) {
    if (commanded_vel > 0 && current_vel_ > commanded_vel + vel_diff_thresh_) {
      brake_pub_->publish(brake_msg);
      servo_pub_->publish(servo_msg);
    } else if (commanded_vel < 0 && current_vel_ < commanded_vel - vel_diff_thresh_) {
      brake_pub_->publish(brake_msg);
      servo_pub_->publish(servo_msg);
    } else {
      erpm_pub_->publish(erpm_msg);
      servo_pub_->publish(servo_msg);
    }
  }

}

void AckermannToVesc::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  current_vel_ = odom_msg->twist.twist.linear.x;
  // RCLCPP_INFO(get_logger(), "Current velocity: %f", current_vel_);
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
