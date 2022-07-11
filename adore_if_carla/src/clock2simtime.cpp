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
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <rosgraph_msgs/Clock.h>

/**
 * This nodes translates messages of topic /clock to topic /SIM/UTC.
 */

namespace adore
{
    namespace adore_if_carla
    {
        class Clock2Simtime
        {
          public:
            Clock2Simtime()
            {
            }
            void init(int argc, char** argv, double rate, std::string nodename)
            {
                // Although the application has no periodically called functions, the rate is required for scheduling
                ros::init(argc, argv, nodename);
                ros::NodeHandle* n = new ros::NodeHandle();
                n_ = n;
                initSim();
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
            ros::Subscriber subscriber_;
            ros::Publisher publisher_;
            ros::NodeHandle* n_;

            void initSim()
            {
            }
            ros::NodeHandle* getRosNodeHandle()
            {
                return n_;
            }
            void initROSConnections()
            {
                subscriber_ = getRosNodeHandle()->subscribe<rosgraph_msgs::Clock>(
                    "/clock", 1, &Clock2Simtime::receiveClockTime, this);
                publisher_ = getRosNodeHandle()->advertise<std_msgs::Float64>("/SIM/utc", 1);
            }
            void receiveClockTime(const rosgraph_msgs::ClockConstPtr& in_msg)
            {
                std_msgs::Float64 out_msg;
                out_msg.data = in_msg->clock.sec + in_msg->clock.nsec * 1e-9;
                publisher_.publish(out_msg);
            }
        };
    }  // namespace adore_if_carla
}  // namespace adore

int main(int argc, char** argv)
{
    adore::adore_if_carla::Clock2Simtime clock2simtime;
    clock2simtime.init(argc, argv, 10.0, "clock2simtime");
    clock2simtime.run();
    return 0;
}
