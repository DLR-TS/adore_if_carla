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
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
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
            void init(int argc, char **argv, double rate, std::string nodename)
            {
                ros::init(argc, argv, nodename);
                ros::NodeHandle *n = new ros::NodeHandle();
                n_ = n;
                initSim();
                bool carla_namespace_specified = n_->getParam("PARAMS/adore_if_carla/carla_namespace", namespace_carla_);
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
                last_throttle_ = 0.0;
                carla_ego_vehicle_max_steer_angle_ = 1.0;
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
            ros::Subscriber subscriber_vehicle_info_;
            ros::Subscriber subscriber_vehicle_status_;
            ros::Publisher publisher_control_command_;
            double acceleration_command_;
            double steering_command_;
            double current_acceleration_;
            double current_steeringAngle_;
            double integrator_error_ax_;
            double integrator_error_delta_;
            double last_error_ax_;
            double last_error_delta_;
            double last_throttle_;
            double current_vx_;
            carla_msgs::CarlaEgoVehicleInfo carla_ego_vehicle_info_;
            carla_msgs::CarlaEgoVehicleStatus carla_ego_vehicle_status_;
            double carla_ego_vehicle_max_steer_angle_;
            ros::NodeHandle *n_;
            ros::Timer transmitControlCommandTimer_;

            void initSim()
            {
            }
            ros::NodeHandle *getRosNodeHandle()
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

                subscriber_vehicle_info_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaEgoVehicleInfo>(
                    "/carla/" + namespace_carla_ + "/vehicle_info", 1, &ControlCommand2Carla::receiveVehicleInfo, this);

                subscriber_vehicle_status_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaEgoVehicleStatus>(
                    "/carla/" + namespace_carla_ + "/vehicle_status", 1, &ControlCommand2Carla::receiveVehicleStatus, this);

                publisher_control_command_ = getRosNodeHandle()->advertise<carla_msgs::CarlaEgoVehicleControl>(
                    "/carla/" + namespace_carla_ + "/vehicle_control_cmd", 1);
            }

            void receiveAccelerationCommand(const std_msgs::Float32ConstPtr &in_msg)
            {
                acceleration_command_ = in_msg->data;
            }

            void receiveSteeringCommand(const std_msgs::Float32ConstPtr &in_msg)
            {
                steering_command_ = in_msg->data;
            }

            void receiveAx(const std_msgs::Float32ConstPtr &in_msg)
            {
                current_acceleration_ = in_msg->data;
            }

            void receiveDelta(const std_msgs::Float32ConstPtr &in_msg)
            {
                current_steeringAngle_ = in_msg->data;
            }

            void receiveOdom(const nav_msgs::OdometryConstPtr &in_msg)
            {
                current_vx_ = in_msg->twist.twist.linear.x;
            }
            void receiveVehicleInfo(const carla_msgs::CarlaEgoVehicleInfoConstPtr &in_msg)
            {
                carla_ego_vehicle_info_.mass = in_msg->mass;
                carla_ego_vehicle_info_.wheels = in_msg->wheels;
                carla_ego_vehicle_max_steer_angle_ = carla_ego_vehicle_info_.wheels[0].max_steer_angle;
            }

            void receiveVehicleStatus(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_msg)
            {
                carla_ego_vehicle_status_.orientation.w = in_msg->orientation.w;
                carla_ego_vehicle_status_.orientation.x = in_msg->orientation.x;
                carla_ego_vehicle_status_.orientation.y = in_msg->orientation.y;
                carla_ego_vehicle_status_.orientation.z = in_msg->orientation.z;
            }
            void transmitControlCommand(const ros::TimerEvent &te)
            {
                carla_msgs::CarlaEgoVehicleControl control_cmd;

                // Throttle/Brake

                // calcuate throttle value by force
                // set parameters
                double w, x, y, z;
                double pitch; // pitch (y rotation)
                x = carla_ego_vehicle_status_.orientation.x;
                y = carla_ego_vehicle_status_.orientation.y;
                z = carla_ego_vehicle_status_.orientation.z;
                w = carla_ego_vehicle_status_.orientation.w;

                double sinp = 2 * (w * y - z * x);
                if (std::abs(sinp) >= 1)
                    pitch = std::copysign(M_PI / 2, sinp);
                else
                    pitch = std::asin(sinp);

                double aerodynamic_drag_coefficient = 0.3; // currently not provided in carla_vehicle_info
                double drag_reference_area = 2.37;         // currently not provided in carla_vehicle_info
                double roh_air = 1.184;
                double rolling_resistance_coefficient = 0.01; // currently not provided in carla_vehicle_info
                double acceleration_of_gravity = 9.81;
                double max_acceleration = 6.5; // maximum acceleration of used carla vehicle, determined by experiment

                // calculate forces
                double areodynamic_drag_force = 0.5 * aerodynamic_drag_coefficient * drag_reference_area * roh_air * current_vx_ * current_vx_;
                double rolling_resistance_force = rolling_resistance_coefficient * acceleration_of_gravity * carla_ego_vehicle_info_.mass;
                double inertial_force = carla_ego_vehicle_info_.mass * acceleration_command_;
                double slope_force = acceleration_of_gravity * carla_ego_vehicle_info_.mass * std::asin(-pitch);
                double max_force = carla_ego_vehicle_info_.mass * max_acceleration;

                // calcuate throttle value
                double throttle_by_force = (areodynamic_drag_force + rolling_resistance_force + inertial_force) / max_force;

                // closed-loop control

                // rostopic pubs for controller tuning
                // rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 0.0}" -r 10
                // rostopic pub /vehicle0/FUN/MotionCommand/acceleration std_msgs/Float32 2 -r 20
                // rostopic pub /vehicle0/FUN/MotionCommand/steeringAngle std_msgs/Float32 19.52 -r 20

                if (current_vx_ > 0.0 || acceleration_command_ > 0.0)
                {
                    integrator_error_ax_ += (acceleration_command_ - current_acceleration_) * 0.01;
                }
                else
                {
                    integrator_error_ax_ = 0.0;
                }

                double throttle = throttle_by_force + integrator_error_ax_ * getAccelerationErrorIGain();

                // limit throttle value
                double throttle_max = last_throttle_ + 0.0015;
                double throttle_min = last_throttle_ - 0.0015;
                throttle = std::max(throttle_min, std::min(throttle_max, throttle));

                if (current_vx_ < 0.03 && acceleration_command_ < 0.01)
                {
                    throttle = 0.0;
                }

                // command throttle or brake
                if (throttle >= 0.0)
                {
                    control_cmd.throttle = std::max(0.0, std::min(1.0, throttle));
                }
                else
                {
                    control_cmd.brake = std::max(0.0, std::min(1.0, -throttle));
                }

                // save current values
                last_error_ax_ = acceleration_command_ - current_acceleration_;
                last_throttle_ = throttle;

                // Steer
                control_cmd.steer = (-steering_command_ / getSteeringGain()) / carla_ego_vehicle_max_steer_angle_;

                control_cmd.hand_brake = false;
                control_cmd.reverse = false;
                control_cmd.gear = 0;
                control_cmd.manual_gear_shift = false;

                publisher_control_command_.publish(control_cmd);
            }
            double getSteeringGain()
            {
                double val = 1.0;
                const std::string name = "PARAMS/adore_if_carla/steering_gain";
                if (!n_->getParamCached(name, val))
                {
                    ROS_INFO_STREAM("No parameter named " << name << " in launch-file. Hardcoded default value used.");
                }
                return val;
            }
            double getAccelerationErrorIGain()
            {
                double val = 0.1;
                const std::string name = "PARAMS/adore_if_carla/a_error_I_gain";
                if (!n_->getParamCached(name, val))
                {
                    ROS_INFO_STREAM("No parameter named " << name << " in launch-file. Hardcoded default value used.");
                }
                return val;
            }
        };
    } // namespace adore_if_carla
} // namespace adore

int main(int argc, char **argv)
{
    adore::adore_if_carla::ControlCommand2Carla controlcommand2carla;
    controlcommand2carla.init(argc, argv, 100.0, "controlcommand2carla");
    controlcommand2carla.run();
    return 0;
}