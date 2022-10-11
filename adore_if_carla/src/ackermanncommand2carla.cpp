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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/

#include <iostream>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <adore_if_ros_msg/SetPointRequest.h>

/**
 * This node translates control messages from ADORe-Automation to ackermann_msgs/AckermannDrive
 * /carla/vehicle_namespace_in_carla/ackermann_cmd
 */

namespace adore
{
    namespace adore_if_carla
    {
        class AckermannCommand2Carla
        {
        public:
            AckermannCommand2Carla()
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
            ros::Subscriber subscriber_spr_;
            ros::Publisher publisher_ackermann_command_;
            double acceleration_command_;
            double steering_command_;
            double current_acceleration_;
            double current_steeringAngle_;
            double current_vx_;
            double current_time_;
            bool b_received_acc_;
            bool b_received_steer_;
            adore_if_ros_msg::SetPointRequest last_spr_;
            ros::NodeHandle *n_;

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
                    "FUN/MotionCommand/acceleration", 1, &AckermannCommand2Carla::receiveAccelerationCommand, this);

                subscriber_steering_command_ = getRosNodeHandle()->subscribe<std_msgs::Float32>(
                    "FUN/MotionCommand/steeringAngle", 1, &AckermannCommand2Carla::receiveSteeringCommand, this);

                subscriber_current_acceleration_ = getRosNodeHandle()->subscribe<std_msgs::Float32>(
                    "VEH/ax", 1, &AckermannCommand2Carla::receiveAx, this);

                subscriber_current_steeringAngle_ = getRosNodeHandle()->subscribe<std_msgs::Float32>(
                    "VEH/steering_angle_measured", 1, &AckermannCommand2Carla::receiveDelta, this);

                subscriber_odom_ = getRosNodeHandle()->subscribe<nav_msgs::Odometry>(
                    "odom", 1, &AckermannCommand2Carla::receiveOdom, this);

                //subscriber_vehicle_info_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaEgoVehicleInfo>(
                //    "/carla/" + namespace_carla_ + "/vehicle_info", 1, &AckermannCommand2Carla::receiveVehicleInfo, this);

                //subscriber_vehicle_status_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaEgoVehicleStatus>(
                //    "/carla/" + namespace_carla_ + "/vehicle_status", 1, &AckermannCommand2Carla::receiveVehicleStatus, this);

                subscriber_spr_ = getRosNodeHandle()->subscribe<adore_if_ros_msg::SetPointRequest>(
                    "FUN/SetPointRequest_odom", 1, &AckermannCommand2Carla::receiveSPR, this);

                publisher_ackermann_command_ = getRosNodeHandle()->advertise<ackermann_msgs::AckermannDrive>(
                    "/carla/" + namespace_carla_ + "/ackermann_cmd", 1);
            }

            void receiveAccelerationCommand(const std_msgs::Float32ConstPtr &in_msg)
            {
                acceleration_command_ = in_msg->data;
                b_received_acc_ = true;
                transmitControlCommand();
            }
            void receiveSteeringCommand(const std_msgs::Float32ConstPtr &in_msg)
            {
                double steering_ratio = 1.0;
                n_->getParamCached("PARAMS/Vehicle/steeringRatio",steering_ratio);
                steering_command_ = in_msg->data / steering_ratio;
                b_received_steer_ = true;
                transmitControlCommand();
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
            void receiveSPR(const adore_if_ros_msg::SetPointRequestConstPtr &in_msg)
            {
                last_spr_.setPoints = in_msg->setPoints;
            }
            void transmitControlCommand() // const ros::TimerEvent &te)
            {
                if (b_received_acc_ && b_received_steer_)
                {
                    b_received_acc_ = false;
                    b_received_steer_ = false;
                }
                else
                {
                    return;
                }
                ackermann_msgs::AckermannDrive ackermann_cmd;
                ackermann_cmd.steering_angle = steering_command_;
                ackermann_cmd.steering_angle_velocity = 0.0;
                ackermann_cmd.speed = std::max(0.5, current_vx_);
                ackermann_cmd.acceleration = acceleration_command_;
                ackermann_cmd.jerk = 0.0;

                // for (int i = 0; i < last_spr_.setPoints.size(); ++i)
                // {
                //     if (last_spr_.setPoints.at(i).tStart <= current_time_ && last_spr_.setPoints.at(i).tEnd > current_time_)
                //     {
                //         ackermann_cmd.steering_angle_velocity = last_spr_.setPoints.at(i).ddelta ;
                //         ackermann_cmd.speed = std::max(last_spr_.setPoints.at(i).vx,0.5);
                //         //ackermann_cmd.acceleration = last_spr_.setPoints.at(i).ax;
                //         //ackermann_cmd.jerk = last_spr_.setPoints.at(i).dax;
                //         break;
                //     }
                // }

                publisher_ackermann_command_.publish(ackermann_cmd);
            }
        };
    } // namespace adore_if_carla
} // namespace adore

int main(int argc, char **argv)
{
    adore::adore_if_carla::AckermannCommand2Carla controlcommand2carla;
    controlcommand2carla.init(argc, argv, 100.0, "controlcommand2carla");
    controlcommand2carla.run();
    return 0;
}