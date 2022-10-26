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
#include <cstdlib>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

/**
 * This node translates vehicle state messages from Carla to ADORe-Automation
 */

namespace adore
{
    namespace adore_if_carla
    {
        class VehicleState2Adore
        {
        public:
            VehicleState2Adore()
            {
            }
            void init(int argc, char **argv, double rate, std::string nodename)
            {
                // Although the application has no periodically called functions, the rate is required for scheduling
                ros::init(argc, argv, nodename);
                ros::NodeHandle *n = new ros::NodeHandle();
                n_ = n;
                initSim();
                bool carla_namespace_specified = getRosNodeHandle()->getParam("PARAMS/adore_if_carla/carla_namespace", namespace_carla_);
                std::cout << "VehicleState2Adore: namespace of the carla vehicle is: "
                          << (carla_namespace_specified ? namespace_carla_ : "NOT SPECIFIED") << std::endl;
                use_imu_ = true;
                initROSConnections();
                carla_autopilot_enabled_ = false;
                adore_automation_enabled_ = false;
                publish_checkpoint_clearance_dummy_ = true;
                carla_ego_vehicle_max_steer_angle_ = 1.0;
                

                for (int i = 0; i < argc; ++i)
                {
                    if (std::string(argv[i]) == "no_cp_clearance_dummy")
                    {
                        publish_checkpoint_clearance_dummy_ = false;
                    }
                    if (std::string(argv[i]) == "adore_automation_enabled")
                    {
                        adore_automation_enabled_ = true;
                    }
                }

                // timer callbacks
                timer_ = n_->createTimer(ros::Duration(1 / rate), std::bind(&VehicleState2Adore::periodic_run, this, std::placeholders::_1));
            }
            void run()
            {
                while (n_->ok())
                {
                    ros::spin();
                }
            }
            bool getCarlaAutopilotStatus()
            {
                return carla_autopilot_enabled_;
            }
            bool getAdoreAutomationStatus()
            {
                return adore_automation_enabled_;
            }
            void setAdoreAutomationStatus(bool value)
            {
                adore_automation_enabled_ = value;
            }

        private:
            std::string namespace_carla_;
            ros::Subscriber subscriber_vehicle_state_;
            ros::Subscriber subscriber_vehicle_info_;
            ros::Subscriber subscriber_vehicle_status_;
            ros::Subscriber subscriber_vehicle_imu_;
            ros::Subscriber subscriber_autopilot_;
            ros::Publisher publisher_vehicle_state_odom_;
            ros::Publisher publisher_vehicle_state_localization_;
            ros::Publisher publisher_longitudinal_acceleration_;
            ros::Publisher publisher_steering_angle_;
            ros::Publisher publisher_control_state_acceleration_;
            ros::Publisher publisher_control_state_acceleration_active_;
            ros::Publisher publisher_control_state_steering_;
            ros::Publisher publisher_gear_state_;
            ros::Publisher publisher_checkpoints_clearnace_;
            ros::Publisher publisher_right_indicator_;
            ros::Publisher publisher_left_indicator_;
            bool carla_autopilot_enabled_;
            bool adore_automation_enabled_;
            bool publish_checkpoint_clearance_dummy_;
            carla_msgs::CarlaEgoVehicleInfo carla_ego_vehicle_info_;
            double carla_ego_vehicle_max_steer_angle_;
            ros::Timer timer_;
            ros::NodeHandle *n_;
            bool use_imu_;

            void initSim()
            {
            }
            ros::NodeHandle *getRosNodeHandle()
            {
                return n_;
            }
            void initROSConnections()
            {
                subscriber_vehicle_state_ = getRosNodeHandle()->subscribe<nav_msgs::Odometry>(
                    "/carla/" + namespace_carla_ + "/odometry", 1, &VehicleState2Adore::receiveOdometry, this);
                publisher_vehicle_state_odom_ = getRosNodeHandle()->advertise<nav_msgs::Odometry>("odom", 1);
                publisher_vehicle_state_localization_ =
                    getRosNodeHandle()->advertise<nav_msgs::Odometry>("localization", 1);
                subscriber_vehicle_info_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaEgoVehicleInfo>(
                    "/carla/" + namespace_carla_ + "/vehicle_info", 1, &VehicleState2Adore::receiveVehicleInfo, this);
                subscriber_vehicle_status_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaEgoVehicleStatus>(
                    "/carla/" + namespace_carla_ + "/vehicle_status", 1, &VehicleState2Adore::receiveVehicleStatus,
                    this);
                if (use_imu_)
                {
                    subscriber_vehicle_imu_ = getRosNodeHandle()->subscribe<sensor_msgs::Imu>(
                        "/carla/" + namespace_carla_ + "/imu", 1, &VehicleState2Adore::receiveVehicleImu,
                        this);
                }
                publisher_longitudinal_acceleration_ = getRosNodeHandle()->advertise<std_msgs::Float32>("VEH/ax", 1);
                publisher_steering_angle_ =
                    getRosNodeHandle()->advertise<std_msgs::Float32>("VEH/steering_angle_measured", 1);
                publisher_gear_state_ = getRosNodeHandle()->advertise<std_msgs::Int8>("VEH/gear_state", 1);

                subscriber_autopilot_ =
                    getRosNodeHandle()->subscribe<std_msgs::Bool>("/carla/" + namespace_carla_ + "/enable_autopilot", 1,
                                                                  &VehicleState2Adore::receiveCarlaAutopilot, this);
                publisher_control_state_acceleration_ =
                    getRosNodeHandle()->advertise<std_msgs::Bool>("VEH/AutomaticControlState/acceleration", 1);
                publisher_control_state_acceleration_active_ =
                    getRosNodeHandle()->advertise<std_msgs::Bool>("VEH/AutomaticControlState/accelerationActive", 1);
                publisher_control_state_steering_ =
                    getRosNodeHandle()->advertise<std_msgs::Bool>("VEH/AutomaticControlState/steering", 1);

                publisher_checkpoints_clearnace_ =
                    getRosNodeHandle()->advertise<std_msgs::Bool>("VEH/Checkpoints/clearance", 1);
                publisher_left_indicator_ = getRosNodeHandle()->advertise<std_msgs::Bool>("VEH/IndicatorState/left", 1);
                publisher_right_indicator_ =
                    getRosNodeHandle()->advertise<std_msgs::Bool>("VEH/IndicatorState/right", 1);
            }
            void receiveOdometry(const nav_msgs::OdometryConstPtr &in_msg)
            {
                double w, x, y, z;
                double yaw;
                x = in_msg->pose.pose.orientation.x;
                y = in_msg->pose.pose.orientation.y;
                z = in_msg->pose.pose.orientation.z;
                w = in_msg->pose.pose.orientation.w;

                // yaw (z rotation)
                double siny_cosp = 2 * (w * z + x * y);
                double cosy_cosp = 1 - 2 * (y * y + z * z);
                yaw = std::atan2(siny_cosp, cosy_cosp);

                double x_pos, y_pos;
                double distance_to_center = -0.9;

                x_pos = in_msg->pose.pose.position.x + std::cos(yaw) * distance_to_center;
                y_pos = in_msg->pose.pose.position.y + std::sin(yaw) * distance_to_center;

                nav_msgs::Odometry out_msg;

                out_msg.header.seq = in_msg->header.seq;
                out_msg.header.stamp.sec = in_msg->header.stamp.sec;
                out_msg.header.stamp.nsec = in_msg->header.stamp.nsec;
                out_msg.header.frame_id = in_msg->header.frame_id;
                out_msg.child_frame_id = in_msg->child_frame_id;
                out_msg.pose.pose.position.x = x_pos;
                out_msg.pose.pose.position.y = y_pos;
                out_msg.pose.pose.position.z = in_msg->pose.pose.position.z;
                out_msg.pose.pose.orientation.x = in_msg->pose.pose.orientation.x;
                out_msg.pose.pose.orientation.y = in_msg->pose.pose.orientation.y;
                out_msg.pose.pose.orientation.z = in_msg->pose.pose.orientation.z;
                out_msg.pose.pose.orientation.w = in_msg->pose.pose.orientation.w;
                out_msg.pose.covariance = in_msg->pose.covariance;
                out_msg.twist.twist.linear.x = in_msg->twist.twist.linear.x;
                out_msg.twist.twist.linear.y = in_msg->twist.twist.linear.y;
                out_msg.twist.twist.linear.z = in_msg->twist.twist.linear.z;
                out_msg.twist.twist.angular.x = in_msg->twist.twist.angular.x;
                out_msg.twist.twist.angular.y = in_msg->twist.twist.angular.y;
                out_msg.twist.twist.angular.z = in_msg->twist.twist.angular.z;
                out_msg.twist.covariance = in_msg->twist.covariance;

                publisher_vehicle_state_odom_.publish(out_msg);
                publisher_vehicle_state_localization_.publish(out_msg);
            }
            void receiveVehicleInfo(const carla_msgs::CarlaEgoVehicleInfoConstPtr &in_msg)
            {
                carla_ego_vehicle_info_.wheels = in_msg->wheels;
                carla_ego_vehicle_max_steer_angle_ = 5.0;
                bool use_default_max_steer_angle = true;
                for (auto wheel:carla_ego_vehicle_info_.wheels)
                {
                    if (wheel.max_steer_angle>0.0 && wheel.max_steer_angle < carla_ego_vehicle_max_steer_angle_)
                    {
                        use_default_max_steer_angle = false;
                        carla_ego_vehicle_max_steer_angle_ = wheel.max_steer_angle;
                    }
                }
                if (use_default_max_steer_angle)
                {
                    std::cout << "vehiclestate2adore: default max steer angle used!"<<std::endl;
                    carla_ego_vehicle_max_steer_angle_ = 1.22;
                }
            }
            void receiveVehicleStatus(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_msg)
            {
                double w, x, y, z;
                double roll, pitch, yaw;
                x = in_msg->orientation.x;
                y = in_msg->orientation.y;
                z = in_msg->orientation.z;
                w = in_msg->orientation.w;

                // roll (x rotation)
                double sinr_cosp = 2 * (w * x + y * z);
                double cosr_cosp = 1 - 2 * (x * x + y * y);
                roll = std::atan2(sinr_cosp, cosr_cosp);

                // pitch (y rotation)
                double sinp = 2 * (w * y - z * x);
                if (std::abs(sinp) >= 1)
                    pitch = std::copysign(M_PI / 2, sinp);
                else
                    pitch = std::asin(sinp);

                // yaw (z rotation)
                double siny_cosp = 2 * (w * z + x * y);
                double cosy_cosp = 1 - 2 * (y * y + z * z);
                yaw = std::atan2(siny_cosp, cosy_cosp);

                yaw = -yaw;
                double lon_accel =
                    std::cos(yaw) * in_msg->acceleration.linear.x - std::sin(yaw) * in_msg->acceleration.linear.y;
                double lat_accel =
                    std::sin(yaw) * in_msg->acceleration.linear.x + std::cos(yaw) * in_msg->acceleration.linear.y;
                if (!use_imu_)
                {
                    std_msgs::Float32 ax_msg;
                    ax_msg.data = lon_accel;
                    publisher_longitudinal_acceleration_.publish(ax_msg);
                }
                std_msgs::Float32 steering_msg;
                steering_msg.data = -carla_ego_vehicle_max_steer_angle_ * in_msg->control.steer;
                publisher_steering_angle_.publish(steering_msg);
                std_msgs::Int8 gear_state_msg;
                int gear = in_msg->control.gear;
                // encoding of gear states:
                // gear state (0:Park, 1: Drive, 2: Reverse, 3: Neutral)
                if (gear == 0)
                {
                    gear_state_msg.data = 3;
                }
                else if (gear > 0)
                {
                    gear_state_msg.data = 1;
                }
                else if (gear < 0)
                {
                    gear_state_msg.data = 2;
                }
                publisher_gear_state_.publish(gear_state_msg);
            }
            void receiveVehicleImu(const sensor_msgs::Imu::ConstPtr &in_msg)
            {
                double lon_accel = in_msg->linear_acceleration.x;
                std_msgs::Float32 ax_msg;
                ax_msg.data = lon_accel;
                publisher_longitudinal_acceleration_.publish(ax_msg);
            }
            void receiveCarlaAutopilot(const std_msgs::BoolConstPtr &in_msg)
            {
                carla_autopilot_enabled_ = in_msg->data;
                if (carla_autopilot_enabled_)
                {
                    adore_automation_enabled_ = false;
                }
                publish_adore_automation_state();
                std::cout << std::endl;
                std::cout << "Carla-Autopilot is now turned" << (carla_autopilot_enabled_ ? " ON" : " OFF")
                          << std::endl;
                std::cout << "Adore-Automation is turned" << (adore_automation_enabled_ ? " ON" : " OFF") << std::endl;
            }
            void publish_adore_automation_state()
            {
                std_msgs::Bool out_msg;
                out_msg.data = adore_automation_enabled_;
                publisher_control_state_acceleration_.publish(out_msg);
                publisher_control_state_acceleration_active_.publish(out_msg);
                publisher_control_state_steering_.publish(out_msg);
            }
            void publish_dummy_checkpoints_clearance_and_indicators()
            {
                std_msgs::Bool out_msg;
                out_msg.data = false;
                if (publish_checkpoint_clearance_dummy_ == true)
                {
                    publisher_checkpoints_clearnace_.publish(out_msg);
                }
                publisher_left_indicator_.publish(out_msg);
                publisher_right_indicator_.publish(out_msg);
            }
            void periodic_run(const ros::TimerEvent &te)
            {
                publish_adore_automation_state();
                publish_dummy_checkpoints_clearance_and_indicators();
            }
        };
    } // namespace adore_if_carla
} // namespace adore

adore::adore_if_carla::VehicleState2Adore vehiclestate2adore;
bool terminated = false;

void kbinput()
{
    while (!terminated)
    {
        int c = std::cin.get();
        if (c == 10)
        {
            if (vehiclestate2adore.getCarlaAutopilotStatus())
            {
                std::cout << "Carla-Autopilot is still turned ON. Please deactivate first." << std::endl;
            }
            else
            {
                vehiclestate2adore.setAdoreAutomationStatus(!vehiclestate2adore.getAdoreAutomationStatus());
                std::cout << std::endl;
                std::cout << "Adore-Automation is now turned"
                          << (vehiclestate2adore.getAdoreAutomationStatus() ? " ON" : " OFF") << std::endl;
                std::cout << "Carla-Autopilot is turned"
                          << (vehiclestate2adore.getCarlaAutopilotStatus() ? " ON" : " OFF") << std::endl;
            }
        }
    }
}

int main(int argc, char **argv)
{
    std::thread kbinput_thread(kbinput);
    vehiclestate2adore.init(argc, argv, 10.0, "vehiclestate2adore");

    if (!vehiclestate2adore.getAdoreAutomationStatus())
    {
        std::cout << std::endl;
        std::cout << "Carla-Autopilot and Adore-Automation are turned off by default." << std::endl;
        std::cout << "To (de-)activate Carla-Autopilot press p in Carla-Ros-Bridge-Window." << std::endl;
        std::cout << "To (de-)activate Adore-Automation press ENTER in this window." << std::endl;
    }
    else
    {
        std::cout << std::endl;
        std::cout << "Adore-Automation is turned" << (vehiclestate2adore.getAdoreAutomationStatus() ? " ON " : " OFF ")
                  << "by argument." << std::endl;
        std::cout << "Carla-Autopilot is turned" << (vehiclestate2adore.getCarlaAutopilotStatus() ? " ON" : " OFF")
                  << " by default." << std::endl;
        std::cout << "To (de-)activate Carla-Autopilot press p in Carla-Ros-Bridge-Window." << std::endl;
        std::cout << "To (de-)activate Adore-Automation press ENTER in this window." << std::endl;
    }

    vehiclestate2adore.run();
    terminated = true;
    return 0;
}
