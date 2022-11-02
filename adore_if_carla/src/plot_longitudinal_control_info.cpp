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
 *  Matthias Nichting
 ********************************************************************************/

#include <plotlablib/figurestubfactory.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

class PlotLongControlInfo
{
private:
    double ax_ackermann_control_in_;
    std::string style_ax_ackermann_control_in_;
    double ax_command_;
    std::string style_ax_command_;
    double ax_actual_;
    std::string style_ax_actual_;
    double throttle_command_;
    std::string style_throttle_command_;
    double t_;
    double t0_;
    bool t0_set_;
    std::string namespace_carla_;
    ros::NodeHandle *n_;

    ros::Subscriber subscriber_ax_ackermann_control_in_;
    ros::Subscriber subscriber_ax_command_;
    ros::Subscriber subscriber_ax_actual_;
    ros::Subscriber subscriber_throttle_command_;
    ros::Subscriber subscriber_odom_;

    DLR_TS::PlotLab::AFigureStub *figure_;

    void receive_ax_ackermann_control_in(ackermann_msgs::AckermannDriveConstPtr msg)
    {
        ax_ackermann_control_in_ = msg->acceleration;
    }
    void receive_ax_command(std_msgs::Float32ConstPtr msg)
    {
        // output by adore fb controller
        ax_command_ = msg->data;
    }
    void receive_ax_actual(std_msgs::Float32ConstPtr msg)
    {
        ax_actual_ = msg->data;
    }
    void receive_throttle_command(carla_msgs::CarlaEgoVehicleControlConstPtr msg)
    {
        throttle_command_ = msg->throttle;
        if (msg->brake > msg->throttle)
        {
            throttle_command_ = -1.0 * msg->brake;
        }
    }
    void receive_odom(nav_msgs::OdometryConstPtr msg)
    {
        if (!t0_set_)
        {
            t0_ = msg->header.stamp.toSec();
            t0_set_ = true;
        }
        t_ = msg->header.stamp.toSec() - t0_;
        do_plot();
    }

    void do_plot()
    {
        figure_->append("ax_command", &t_, &ax_command_, 1, style_ax_command_);
        // figure_->append("ax_ackermann_control_in",&t_,&ax_ackermann_control_in_,1,style_ax_ackermann_control_in_);
        figure_->append("ax_actual", &t_, &ax_actual_, 1, style_ax_actual_);
        figure_->append("throttle_command", &t_, &throttle_command_, 1, style_throttle_command_);
    }

public:
    PlotLongControlInfo()
        : ax_ackermann_control_in_(0.0),
          style_ax_ackermann_control_in_("LineColor=0,0,1;LineWidth=2"),
          ax_command_(0.0),
          style_ax_command_("LineColor=0.5,0,0.5;LineWidth=2"),
          ax_actual_(0.0),
          style_ax_actual_("LineColor=1,0,0;LineWidth=2"),
          throttle_command_(0.0),
          style_throttle_command_("LineColor=0.5,0.5,0;LineWidth=2"),
          t_(0.0),
          t0_(0.0),
          t0_set_(false)
    {
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        figure_ = fig_factory.createFigureStub(9);
        figure_->setTitle("acc_command (pink), actual acceleration (red), throttle (yellow)");
        figure_->setXLabel("t (s)");
        figure_->setYLabel("m/s or 1");
        figure_->showAxis();
        figure_->showGrid();
        figure_->show();

        n_ = new ros::NodeHandle();
        bool carla_namespace_specified = n_->getParam("PARAMS/adore_if_carla/carla_namespace", namespace_carla_);
        std::cout << "PlotLongitudinalControlInfo: namespace of the carla vehicle is: "
                  << (carla_namespace_specified ? namespace_carla_ : "NOT SPECIFIED") << std::endl;

        subscriber_ax_ackermann_control_in_ = n_->subscribe("/carla/" + namespace_carla_ + "/ackermann_cmd", 1, &PlotLongControlInfo::receive_ax_ackermann_control_in, this);
        subscriber_ax_command_ = n_->subscribe("FUN/MotionCommand/acceleration", 1, &PlotLongControlInfo::receive_ax_command, this);
        subscriber_ax_actual_ = n_->subscribe("VEH/ax", 1, &PlotLongControlInfo::receive_ax_actual, this);
        subscriber_throttle_command_ = n_->subscribe("/carla/" + namespace_carla_ + "/vehicle_control_cmd", 1, &PlotLongControlInfo::receive_throttle_command, this);
        subscriber_odom_ = n_->subscribe("odom", 1, &PlotLongControlInfo::receive_odom, this);
    }
    void run()
    {
        while (n_->ok())
        {
            ros::spin();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_longitudinal_control_info");
    PlotLongControlInfo plci;
    plci.run();
}