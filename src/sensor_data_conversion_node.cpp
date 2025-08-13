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
#include "sensor_data_conversion_node.hpp"
#include <adore_dynamics_conversions.hpp>

namespace adore
{
namespace carla_bridge
{
SensorDataConversionNode::SensorDataConversionNode() :
  Node( "sensor_data_conversion_node" )
{
  load_parameters();
  create_publishers();
  create_subscribers();
  last_call_sensor_callback_gnss = 0.0;
  last_call_sensor_callback_imu = 0.0;
  initialization_time = now().seconds();
}

void
SensorDataConversionNode::load_parameters()
{

  declare_parameter<std::vector<double>>( "set_shape", { 4.5, 2.0, 2.0 } );
  get_parameter( "set_shape", ego_vehicle_shape );

  declare_parameter( "vehicle_id", 0 );
  get_parameter( "vehicle_id", current_traffic_participant.id );

  current_traffic_participant.physical_parameters.body_length = ego_vehicle_shape[0];
  current_traffic_participant.physical_parameters.body_width = ego_vehicle_shape[1];
  current_traffic_participant.physical_parameters.body_height = ego_vehicle_shape[2];

  declare_parameter<std::vector<std::string>>( "other_vehicle_namespaces", std::vector<std::string>{} );
  get_parameter( "other_vehicle_namespaces", other_vehicle_namespaces );
}

void
SensorDataConversionNode::create_publishers()
{
  publisher_vehicle_state_dynamic = create_publisher<adore_ros2_msgs::msg::VehicleStateDynamic>("vehicle_state/dynamic", 10);
  publisher_state_monitor         = create_publisher<adore_ros2_msgs::msg::StateMonitor>("vehicle_state/monitor", 10);
  publisher_traffic_participant_set = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>("traffic_participants", 10);
  publisher_traffic_participant = create_publisher<adore_ros2_msgs::msg::TrafficParticipant>("vehicle_state/traffic_participant", 10);
  tf_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);  
}

void
SensorDataConversionNode::create_subscribers()
{
  main_timer = create_wall_timer( 1 * std::chrono::milliseconds(time_step_ms), std::bind( &SensorDataConversionNode::timer_callback, this ) );
  subscriber_sensor_data_gnss = create_subscription<sensor_msgs::msg::NavSatFix>( "/carla/" + sensor_parent_name + "/" + sensor_role_name_gnss, 10,
                                                                           std::bind( &SensorDataConversionNode::sensor_callback_gnss,
                                                                                      this, std::placeholders::_1 ) );
  subscriber_sensor_data_imu = create_subscription<sensor_msgs::msg::Imu>( "/carla/" + sensor_parent_name + "/" + sensor_role_name_imu, 10,
                                                                           std::bind( &SensorDataConversionNode::sensor_callback_imu,
                                                                                      this, std::placeholders::_1 ) );
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  for( const auto& ns : other_vehicle_namespaces )
  {
    if( ns == get_namespace() )
      continue;

    std::string odom_topic = "/" + ns + "/vehicle_state/traffic_participant";

    auto traffic_participant_subscription = create_subscription<adore_ros2_msgs::msg::TrafficParticipant>(
      odom_topic, 1,
      [this, ns]( const adore_ros2_msgs::msg::TrafficParticipant& msg ) { other_vehicle_traffic_participant_callback( msg, ns ); } );

    other_vehicle_traffic_participant_subscribers.emplace_back( traffic_participant_subscription );
  }
}

void
SensorDataConversionNode::other_vehicle_traffic_participant_callback(const adore_ros2_msgs::msg::TrafficParticipant& msg,
                                                                     const std::string& vehicle_namespace)
{
  other_vehicles[vehicle_namespace] = dynamics::conversions::to_cpp_type(msg);
}



void SensorDataConversionNode::timer_callback()
{
  if (last_call_sensor_callback_gnss == 0.0 || last_call_sensor_callback_imu == 0.0)
  {
    if (now().seconds() - 1.0 > initialization_time)
    {
      std::string not_initialized;
      if (last_call_sensor_callback_gnss == 0.0)
      {
        if(!not_initialized.empty())
        {
          not_initialized.append(", ");
        }
        not_initialized.append("gnss");
      }
      if (last_call_sensor_callback_imu == 0.0)
      {
        if(!not_initialized.empty())
        {
          not_initialized.append(", ");
        }
        not_initialized.append("imu");
      }
      std::cout << "not initialized: " << not_initialized << std::endl;
    }
    return;
  }

  
  if(current_traffic_participant.id == 0)
  {
    publish_traffic_participants();
  }
  
  current_traffic_participant.state = current_vehicle_state;
  publish_vehicle_states();
  
  if(current_traffic_participant.id == 0)
  {
    publish_ego_transform();
  }
}




void
SensorDataConversionNode::publish_ego_transform()
{
  auto vehicle_frame = dynamics::conversions::vehicle_state_to_transform(current_vehicle_state, rclcpp::Time(static_cast<rcl_time_point_value_t>(current_vehicle_state.time * 1e9)), "world");
  tf_transform_broadcaster->sendTransform(vehicle_frame);
}

void
SensorDataConversionNode::publish_vehicle_states()
{
  adore_ros2_msgs::msg::VehicleStateDynamic dynamic_msg = dynamics::conversions::to_ros_msg(current_vehicle_state);
  publisher_vehicle_state_dynamic->publish(dynamic_msg);

  // for consistency with bob interface
  adore_ros2_msgs::msg::StateMonitor state_monitor_msg;
  state_monitor_msg.localization_error = 0.0;
  publisher_state_monitor->publish(state_monitor_msg);

  // Ego vehicle publishing itself as traffic participant
  adore_ros2_msgs::msg::TrafficParticipant ego_as_traffic_participant = dynamics::conversions::to_ros_msg(current_traffic_participant);
  publisher_traffic_participant->publish(ego_as_traffic_participant);
}



void
SensorDataConversionNode::sensor_callback_gnss(const sensor_msgs::msg::NavSatFix& msg)
{
  double x,y,z;
  double k = 1;
  GeographicLib::TransverseMercatorExact tmexact(GeographicLib::Constants::WGS84_a(),GeographicLib::Constants::WGS84_f(),k);//GeographicLib::Constants::UTM_k0());
  tmexact.Forward(0,msg.latitude,msg.longitude,x,y,z,k);

  // int zone;
  // bool north;
  // GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude, zone, north, x, y);
  // GeographicLib::LocalCartesian localcart(0,0,0);  
  // localcart.Forward(msg.latitude,msg.longitude,0,x,y,z);
  
  
  double dt = (last_call_sensor_callback_gnss == 0.0) ? 0.0 : (rclcpp::Time(msg.header.stamp).seconds() - last_call_sensor_callback_gnss);
  last_call_sensor_callback_gnss = rclcpp::Time(msg.header.stamp).seconds();
  current_vehicle_state.time = last_call_sensor_callback_gnss;
  
  double vx_map = (dt > 0.0) ? (x - current_vehicle_state.x) / dt : 0.0;
  double vy_map = (dt > 0.0) ? (y - current_vehicle_state.y) / dt : 0.0;
  current_vehicle_state.vx = vx_map * cos(current_vehicle_state.yaw_angle) + vy_map * sin(current_vehicle_state.yaw_angle);
  current_vehicle_state.vy = -vx_map * sin(current_vehicle_state.yaw_angle) + vy_map * cos(current_vehicle_state.yaw_angle);
  std::cout << "vx is " << current_vehicle_state.vx << " vy is " << current_vehicle_state.vy << std::endl;
  std::cout << "dt " << dt << " vxmap " << vx_map << " vymap "<< vy_map << std::endl;
  
  current_vehicle_state.x = x;
  current_vehicle_state.y = y;
  current_vehicle_state.z = msg.altitude;
  
}

void
SensorDataConversionNode::sensor_callback_imu(const sensor_msgs::msg::Imu& msg)
{
  tf2::Quaternion quat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  current_vehicle_state.yaw_angle = +M_PI/4.0 + yaw;
  current_vehicle_state.yaw_rate = msg.angular_velocity.z;
  // steering_angle not available
  // steering_rate not available
  current_vehicle_state.ax = msg.linear_acceleration.x;
  current_vehicle_state.ay = msg.linear_acceleration.y;
  last_call_sensor_callback_imu = rclcpp::Time(msg.header.stamp).seconds();
}
  

void
SensorDataConversionNode::publish_traffic_participants()
{
// Clear previous data
  dynamics::TrafficParticipantSet traffic_participants;

  for( const auto& [vehicle_namespace, other_vehicle] : other_vehicles )
  {
    double distance = adore::math::distance_2d(other_vehicle.state, current_vehicle_state);
    if( distance > sensor_range )
      continue;

    traffic_participants.participants[other_vehicle.id] = other_vehicle;
  }
  publisher_traffic_participant_set->publish(dynamics::conversions::to_ros_msg(traffic_participants));
}
} // namespace carla_bridge
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::carla_bridge::SensorDataConversionNode>() );
  rclcpp::shutdown();
}


