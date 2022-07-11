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
#include <derived_object_msgs/ObjectArray.h>
#include <adore_if_ros_msg/TrafficParticipantSet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <rosgraph_msgs/Clock.h>

/**
 * This node translates messages of topic /carla/vehicle_namespace_in_carla/objects to topic
 * /vehicle_namespace_in_adore/traffic.
 */

namespace adore
{
    namespace adore_if_carla
    {
        class Objects2Adore
        {
          public:
            Objects2Adore()
            {
            }
            void init(int argc, char** argv, double rate, std::string nodename)
            {
                // Although the application has no periodically called functions, the rate is required for scheduling
                ros::init(argc, argv, nodename);
                ros::NodeHandle* n = new ros::NodeHandle();
                n_ = n;
                initSim();
                bool carla_namespace_specified = n_->getParam("carla_namespace", namespace_carla_);
                std::cout << "VehicleState2Adore: namespace of the carla vehicle is: "
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
            ros::Subscriber subscriber_;
            ros::Publisher publisher_;
            std::string namespace_carla_;
            ros::NodeHandle* n_;

            void initSim()
            {
            }
            ros::NodeHandle* getRosNodeHandle()
            {
                return n_;
            }
            void receiveClockTime(const rosgraph_msgs::ClockConstPtr& in_msg)
            {
                std_msgs::Float64 out_msg;
                out_msg.data = in_msg->clock.sec + in_msg->clock.nsec * 1e-9;
                publisher_.publish(out_msg);
            }
            void initROSConnections()
            {
                subscriber_ = getRosNodeHandle()->subscribe<derived_object_msgs::ObjectArray>(
                    "/carla/" + namespace_carla_ + "/objects", 1, &Objects2Adore::receiveObjects, this);
                publisher_ = getRosNodeHandle()->advertise<adore_if_ros_msg::TrafficParticipantSet>("traffic", 1);
            }
            void receiveObjects(const derived_object_msgs::ObjectArrayConstPtr& in_msg)
            {
                adore_if_ros_msg::TrafficParticipantSet out_msg;

                for (auto object_it = in_msg->objects.begin(); object_it != in_msg->objects.end(); ++object_it)
                {
                    adore_if_ros_msg::TrafficParticipantDetection tp;
                    tp.trackingID = object_it->id;
                    tp.detection_by_sensor = tp.UNDEFINED;
                    // todo: header time stamp may be not same as detection time
                    tp.data.time = object_it->header.stamp.sec + object_it->header.stamp.nsec * 1e-9;
                    // todo: no covariance information for pose and twist
                    tp.data.motion_state.pose.pose = object_it->pose;
                    tp.data.motion_state.twist.twist = object_it->twist;
                    // todo: tp.data.motion_state.frame_id
                    tp.data.shape = object_it->shape;
                    // todo: no actual value for dimensions_variance
                    tp.data.dimensions_variance = 0.0;
                    if (object_it->classification == object_it->CLASSIFICATION_UNKNOWN)
                        tp.data.classification.type_id = tp.data.classification.UNCLASSIFIED;
                    else if (object_it->classification == object_it->CLASSIFICATION_UNKNOWN_SMALL)
                        tp.data.classification.type_id = tp.data.classification.UNKNOWN_SMALL;
                    else if (object_it->classification == object_it->CLASSIFICATION_UNKNOWN_MEDIUM)
                        tp.data.classification.type_id = tp.data.classification.UNKNOWN_SMALL;
                    else if (object_it->classification == object_it->CLASSIFICATION_UNKNOWN_BIG)
                        tp.data.classification.type_id = tp.data.classification.UNKNOWN_BIG;
                    else if (object_it->classification == object_it->CLASSIFICATION_PEDESTRIAN)
                        tp.data.classification.type_id = tp.data.classification.PEDESTRIAN;
                    else if (object_it->classification == object_it->CLASSIFICATION_BIKE)
                        tp.data.classification.type_id = tp.data.classification.BIKE;
                    else if (object_it->classification == object_it->CLASSIFICATION_CAR)
                        tp.data.classification.type_id = tp.data.classification.CAR;
                    else if (object_it->classification == object_it->CLASSIFICATION_TRUCK)
                        tp.data.classification.type_id = tp.data.classification.TRUCK;
                    else if (object_it->classification == object_it->CLASSIFICATION_MOTORCYCLE)
                        tp.data.classification.type_id = tp.data.classification.MOTORCYCLE;
                    else if (object_it->classification == object_it->CLASSIFICATION_OTHER_VEHICLE)
                        tp.data.classification.type_id = tp.data.classification.UNKNOWN_BIG;
                    else if (object_it->classification == object_it->CLASSIFICATION_BARRIER)
                        tp.data.classification.type_id = tp.data.classification.UNCLASSIFIED;
                    else if (object_it->classification == object_it->CLASSIFICATION_SIGN)
                        tp.data.classification.type_id = tp.data.classification.UNCLASSIFIED;
                    else
                        std::cout << "unknown classification" << std::endl;

                    tp.data.classification_certainty =
                        (u_int8_t)object_it->classification_certainty / 255 * tp.data.certainty_max;
                    tp.data.existance_certainty = tp.data.certainty_max;
                    out_msg.data.push_back(tp);
                }
                publisher_.publish(out_msg);
            }
        };
    }  // namespace adore_if_carla
}  // namespace adore

int main(int argc, char** argv)
{
    adore::adore_if_carla::Objects2Adore objects2adore;
    objects2adore.init(argc, argv, 10.0, "objects2adore");
    objects2adore.run();
    return 0;
}
