// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "sensor_msgs/Imu.h"

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher rpy_publisher;
ros::Subscriber quat_subscriber;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // this Vector is then published:
    rpy_publisher.publish(rpy);
    ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    rpy_publisher = n.advertise<geometry_msgs::Vector3>("/imu_transform/euler", 1);
    quat_subscriber = n.subscribe("/imu/data", 1, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}