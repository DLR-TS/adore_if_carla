/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/vehicle_command.hpp"

#include "dynamics/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace adore
{
namespace carla_bridge
{
class ControlCommandConversionNode : public rclcpp::Node
{
public:

  ControlCommandConversionNode();


private:

  void create_subscribers();
  void create_publishers();
  void timer_callback();


  /******************************* PUBLISHERS ************************************************************/
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr publisher_carla_control_command;

  /******************************* SUBSCRIBERS ************************************************************/
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleCommand>::SharedPtr      subscriber_adore_control_command;
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr subscriber_vehicle_state;

  /******************************* OTHER MEMBERS ************************************************************/

  void                                 callback_adore_control_command( const adore_ros2_msgs::msg::VehicleCommand& msg );
  void                                 vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );
  rclcpp::TimerBase::SharedPtr         main_timer;
  int                                  time_step_ms = 50;
  adore::dynamics::VehicleStateDynamic current_vehicle_state;
};
} // namespace carla_bridge
} // namespace adore