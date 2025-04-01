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
#include "control_command_conversion_node.hpp"

namespace adore
{
namespace carla_bridge
{
ControlCommandConversionNode::ControlCommandConversionNode() :
  Node( "control_command_conversion_node" )
{
  create_publishers();
  create_subscribers();
}

void
ControlCommandConversionNode::create_publishers()
{
  publisher_carla_control_command = create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 10);  
}

void
ControlCommandConversionNode::create_subscribers()
{
  main_timer = create_wall_timer( 1 * std::chrono::milliseconds(time_step_ms), std::bind( &ControlCommandConversionNode::timer_callback, this ) );
  subscriber_adore_control_command = create_subscription<adore_ros2_msgs::msg::VehicleCommand>( "next_vehicle_command", 10,
                                                                            std::bind( &ControlCommandConversionNode::callback_adore_control_command,
                                                                                     this, std::placeholders::_1 ) );
  subscriber_vehicle_state = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>("vehicle_state/dynamic", 1, 
                                                                            std::bind( &ControlCommandConversionNode::vehicle_state_callback, this, std::placeholders::_1 ) );
}



void ControlCommandConversionNode::timer_callback()
{
  // timer_callback is not invoked 
  return;
}

void
ControlCommandConversionNode::callback_adore_control_command(const adore_ros2_msgs::msg::VehicleCommand& msg)
{
  carla_msgs::msg::CarlaEgoVehicleControl output;
  output.throttle = msg.acceleration > 0. ? std::min(1.,msg.acceleration/2):0.;
  output.brake = msg.acceleration < 0. ? std::min(1.,-msg.acceleration/6):0.;
  output.steer = -msg.steering_angle/0.7;
  output.hand_brake = false;
  output.reverse = false;
  output.manual_gear_shift = false;
  output.header.stamp = now();
  publisher_carla_control_command->publish(output);
}

void
ControlCommandConversionNode::vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  current_vehicle_state = dynamics::conversions::to_cpp_type( msg );
}


  
} // namespace carla_bridge
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::carla_bridge::ControlCommandConversionNode>() );
  rclcpp::shutdown();
}
