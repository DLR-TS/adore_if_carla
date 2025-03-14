/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Matthias Nichting
 ********************************************************************************/

#pragma once

#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <cmath>

#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/state_monitor.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/traffic_participant.hpp"
#include "dynamics/vehicle_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "GeographicLib/TransverseMercatorExact.hpp"
#include "GeographicLib/Constants.hpp"


namespace adore
{
namespace carla_bridge
{
class SensorDataConversionNode : public rclcpp::Node
{
public:

  SensorDataConversionNode();


private:

  void load_parameters();
  void create_subscribers();
  void create_publishers();
  void timer_callback();

  void publish_vehicle_states();
  void publish_ego_transform();
  void publish_traffic_participants();


  /******************************* PUBLISHERS ************************************************************/
  rclcpp::Publisher<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr   publisher_vehicle_state_dynamic;
  rclcpp::Publisher<adore_ros2_msgs::msg::StateMonitor>::SharedPtr          publisher_state_monitor;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                            tf_transform_broadcaster;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_traffic_participant_set;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr    publisher_traffic_participant;


  /******************************* SUBSCRIBERS ************************************************************/
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_sensor_data_gnss;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            subscriber_sensor_data_imu;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  using StateSubscriber = rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr;
  std::vector<StateSubscriber> other_vehicle_traffic_participant_subscribers;

  /******************************* OTHER MEMBERS ************************************************************/
  void other_vehicle_traffic_participant_callback(const adore_ros2_msgs::msg::TrafficParticipant& msg,
                                                   const std::string&                              vehicle_namespace);
  void sensor_callback_imu(const sensor_msgs::msg::Imu& msg);
  void sensor_callback_gnss(const sensor_msgs::msg::NavSatFix& msg);

  rclcpp::TimerBase::SharedPtr main_timer;


 
  adore::dynamics::VehicleStateDynamic                          current_vehicle_state;
  adore::dynamics::TrafficParticipant                           current_traffic_participant;
  std::unordered_map<std::string, dynamics::TrafficParticipant> other_vehicles;
  std::vector<std::string>                                      other_vehicle_namespaces;

  int time_step_ms           = 50;
  double last_call_sensor_callback_imu = 0.0;
  double last_call_sensor_callback_gnss = 0.0;
  double initialization_time = 0.0;
  std::string sensor_parent_name = "hero";
  std::string sensor_role_name_gnss = "gnss";
  std::string sensor_role_name_imu = "imu";
  std::vector<double> ego_vehicle_shape            = { 0.0, 0.0, 0.0 };
  double sensor_range            = 100;

};
} // namespace carla_bridge
} // namespace adore