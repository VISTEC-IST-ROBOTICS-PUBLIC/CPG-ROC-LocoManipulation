
#ifndef DUNG_BEETLE_CONTROLLER_REALROS2CLASS_H
#define DUNG_BEETLE_CONTROLLER_REALROS2CLASS_H

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>

// core ros2 library
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

// std msgs
#include <std_msgs/msg/string.hpp>
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include <std_msgs/msg/int32.hpp>
// #include "std_msgs/MultiArrayLayout.h"
// #include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/msg/float32_multi_array.hpp"
// #include "std_msgs/Int32MultiArray.h"

// Dynamixel msgs
//#include <dynamixel_workbench_msgs/JointCommand.h>

// Rosgraph msgs
// #include <rosgraph_msgs/Clock.h>

// sensor msgs
// // Messages
#include <sensor_msgs/msg/joint_state.hpp>

// joy msgs
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"

// data transformation
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

#define PI 3.14159265359

class realRos2Class : public rclcpp::Node
{
    private:
        using JointState = sensor_msgs::msg::JointState;
        using IMU_data = sensor_msgs::msg::Imu;
        using FloatArray = std_msgs::msg::Float32MultiArray;

        // Subscribers
        // ros::Subscriber joint_states;
        // ros::Subscriber joySub;
        // ros::Subscriber imuSub;
        // ros::Subscriber imuEulerSub;

        // Publishers
        // ros::Publisher jointControlPub;
        // ros::Publisher jointNewStatesPub;
        // ros::Publisher plotPub;
        
        // Private Global Variables
        // rclcpp::Rate * loop_rate;

        // Callbacks
        void jointstates_subscribe_callback(const sensor_msgs::msg::JointState &msg);

        // Msg Types

        // Publisher
        rclcpp::Publisher<JointState>::SharedPtr joint_newstates_publisher;
        rclcpp::Publisher<FloatArray>::SharedPtr data_plot_publisher;
        // Subscriber
        rclcpp::Subscription<JointState>::SharedPtr joint_states_subscriber;
        rclcpp::Subscription<IMU_data>::SharedPtr imu_subscriber;

        rclcpp::TimerBase::SharedPtr timer_;

        JointState joint_feedback_state_;
        JointState update_state_;

        float degtoRad = PI/180.0;
        
    public:

        // Public Methods

        // Constructor
        realRos2Class(int argc,char* argv[]);

        //Destructor
        virtual ~realRos2Class();

        // Read robot feedback from sensor_msg
        JointState getJointFeedback();
        std::vector<float> getImuFeedback();
        // void joy_CB(const sensor_msgs::Joy::ConstPtr& joy); // joy callback
        void imu_callback(const sensor_msgs::msg::Imu& imu); // imu callback
        // void imuEuler_CB(const geometry_msgs::Vector3::ConstPtr& imuEuler); // imu callback


        // Publishing methods
        void setLegMotorPosition(vector<float> positions);
        void setLegMotorTorques(vector<float> torques);
        void updateMotorState(vector<string> names, vector<float> positions, vector<float> velocities, vector<float> torques);
        void dataPlot(vector<float> data);
        
        // // Other
        void ros2_spin_some();

        // Public Global Variables
        std::vector<int> jointIDs           = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> jointPositions   = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> jointVelocities  = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> jointTorques     = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> jointErrorStates = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> axes             ={0,0,0,0,0,0,0,0};
        std::vector<int> buttons            ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        std::vector<float> imu              ={0,0,0, 0,0,0, 0,0,0,0, 0,0,0};
        // tf2::Quaternion q;
        tf2::Matrix3x3 m;
        // std::vector<float> accel            ={0,0,0};
        std::vector<float> omega = {0,0,0};
        std::vector<float> angle = {0,0,0};
        std::vector<float> old_omega = {0,0,0};
        std::vector<float> old_angle = {0,0,0};
		float angle_input_w = 1.0;
		float angle_self_w = 0.0;

        rclcpp::Rate * loop_rate;
};


#endif //DUNG_BEETLE_CONTROLLER_REALROS2CLASS_H
