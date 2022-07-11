/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#include <iostream>
#include <thread>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

/**
 * This node translates control messages from ADORe-Automation to Carla topic
 * /carla/vehicle_namespace_in_carla/vehicle_control_cmd
 */

namespace adore
{
    namespace adore_if_carla
    {
        class ControlCommand2Carla
        {
          public:
            ControlCommand2Carla()
            {
            }
            void init(int argc, char** argv, double rate, std::string nodename)
            {
                ros::init(argc, argv, nodename);
                ros::NodeHandle* n = new ros::NodeHandle();
                n_ = n;
                initSim();
                bool carla_namespace_specified = n_->getParam("carla_namespace", namespace_carla_);
                std::cout << "ControlCommand2Carla: namespace of the carla vehicle is: "
                          << (carla_namespace_specified ? namespace_carla_ : "NOT SPECIFIED") << std::endl;
                initROSConnections();

                // timer callbacks
                transmitControlCommandTimer_ = n_->createTimer(
                    ros::Duration(1 / rate), std::bind(&ControlCommand2Carla::transmitControlCommand, this, std::placeholders::_1));

                integrator_error_ax_ = 0.0;
                integrator_error_delta_ = 0.0;
                last_error_ax_ = 0.0;
                last_error_delta_ = 0.0;
            }
            void run()
            {
                while (n_->ok())
                {
                    ros::spin();
                }
            }

          private:
            std::string namespace_carla_;
            ros::Subscriber subscriber_acceleration_command_;
            ros::Subscriber subscriber_steering_command_;
            ros::Subscriber subscriber_current_acceleration_;
            ros::Subscriber subscriber_current_steeringAngle_;
            ros::Subscriber subscriber_odom_;
            ros::Publisher publisher_control_command_;
            double acceleration_command_;
            double steering_command_;
            double current_acceleration_;
            double current_steeringAngle_;
            double integrator_error_ax_;
            double integrator_error_delta_;
            double last_error_ax_;
            double last_error_delta_;
            double current_vx_;
            ros::NodeHandle* n_;
            ros::Timer transmitControlCommandTimer_;

            void initSim()
            {
            }
            ros::NodeHandle* getRosNodeHandle()
            {
                return n_;
            }

            void initROSConnections()
            {
                subscriber_acceleration_command_ = getRosNodeHandle()->subscribe<std_msgs::Float32>(
                    "FUN/MotionCommand/acceleration", 1, &ControlCommand2Carla::receiveAccelerationCommand, this);

                subscriber_steering_command_ = getRosNodeHandle()->subscribe<std_msgs::Float32>(
                    "FUN/MotionCommand/steeringAngle", 1, &ControlCommand2Carla::receiveSteeringCommand, this);

                subscriber_current_acceleration_ = getRosNodeHandle()->subscribe<std_msgs::Float32>(
                    "VEH/ax", 1, &ControlCommand2Carla::receiveAx, this);

                subscriber_current_steeringAngle_ = getRosNodeHandle()->subscribe<std_msgs::Float32>(
                    "VEH/steering_angle_measured", 1, &ControlCommand2Carla::receiveDelta, this);

                subscriber_odom_ = getRosNodeHandle()->subscribe<nav_msgs::Odometry>(
                    "odom", 1, &ControlCommand2Carla::receiveOdom, this);

                publisher_control_command_ = getRosNodeHandle()->advertise<carla_msgs::CarlaEgoVehicleControl>(
                    "/carla/" + namespace_carla_ + "/vehicle_control_cmd", 1);
            }

            void receiveAccelerationCommand(const std_msgs::Float32ConstPtr& in_msg)
            {
                acceleration_command_ = in_msg->data;
            }

            void receiveSteeringCommand(const std_msgs::Float32ConstPtr& in_msg)
            {
                steering_command_ = in_msg->data;
            }

            void receiveAx(const std_msgs::Float32ConstPtr& in_msg)
            {
                current_acceleration_ = in_msg->data;
            }

            void receiveDelta(const std_msgs::Float32ConstPtr& in_msg)
            {
                current_steeringAngle_ = in_msg->data;
            }

            void receiveOdom(const nav_msgs::OdometryConstPtr& in_msg)
            {
                current_vx_ = in_msg->twist.twist.linear.x;
            }

            void transmitControlCommand(const ros::TimerEvent& te)
            {
                carla_msgs::CarlaEgoVehicleControl control_cmd;

                // Throttle/Brake
                if (current_vx_ > 0)
                {
                    integrator_error_ax_ += (acceleration_command_ - current_acceleration_) * 0.01;
                }
                else
                {
                    integrator_error_ax_ = 0.0;
                }

                double ax = (acceleration_command_ - current_acceleration_) * 0.1 + integrator_error_ax_ * 0.25 +
                            (last_error_ax_ - (acceleration_command_ - current_acceleration_)) * 0.03;

                if (ax >= 0)
                {
                    control_cmd.throttle = std::max(0.0, std::min(ax, 1.0));
                }
                else
                {
                    control_cmd.brake = std::max(0.0, std::min(-ax, 1.0));
                }

                last_error_ax_ = acceleration_command_ - current_acceleration_;

                // Steer

                control_cmd.steer = -steering_command_ / 17;

                control_cmd.hand_brake = false;
                control_cmd.reverse = false;
                control_cmd.gear = 0;
                control_cmd.manual_gear_shift = false;

                publisher_control_command_.publish(control_cmd);
            }
        };
    }  // namespace adore_if_carla
}  // namespace adore

int main(int argc, char** argv)
{
    adore::adore_if_carla::ControlCommand2Carla controlcommand2carla;
    controlcommand2carla.init(argc, argv, 100.0, "controlcommand2carla");
    controlcommand2carla.run();
    return 0;
}