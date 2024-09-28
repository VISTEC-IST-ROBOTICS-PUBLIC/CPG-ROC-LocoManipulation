//
// Created by Carlos on 05/2019.
//

#ifndef DUNG_BEETLE_CONTROLLER_REALROSCLASS_H
#define DUNG_BEETLE_CONTROLLER_REALROSCLASS_H

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <vector>
#include <string>

// std msgs
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

// Dynamixel msgs
//#include <dynamixel_workbench_msgs/JointCommand.h>

// Rosgraph msgs
#include <rosgraph_msgs/Clock.h>

// sensor msgs
#include <sensor_msgs/JointState.h>

// joy msgs
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"

// data transformation
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

#define PI 3.14159265359

class realRosClass 
{
    private:

        // Subscribers
        ros::Subscriber joint_states;
        ros::Subscriber joySub;
        ros::Subscriber imuSub;
        ros::Subscriber imuEulerSub;

        // Publishers
        ros::Publisher jointControlPub;
        ros::Publisher jointNewStatesPub;
        ros::Publisher plotPub;
        
        // Private Global Variables
        ros::Rate* rate;

        // Callbacks
        void jointStatesCallback(const sensor_msgs::JointState& msg);

        sensor_msgs::JointState _state_;
        sensor_msgs::JointState update_state_;

        float degtoRad = PI/180.0;
        
    public:

        // Public Methods

        // Constructor
        realRosClass(int argc, char *argv[]);

        //Destructor
        ~realRosClass();

        // Read robot feedback from sensor_msg
        void getRobotFeedback();
        void joy_CB(const sensor_msgs::Joy::ConstPtr& joy); // joy callback
        void imu_CB(const sensor_msgs::Imu::ConstPtr& imu); // imu callback
        void imuEuler_CB(const geometry_msgs::Vector3::ConstPtr& imuEuler); // imu callback


        // Publishing methods
        void setLegMotorPosition(vector<float> positions);
        void setLegMotorTorques(vector<float> torques);
        void updateMotorState(vector<string> names, vector<float> positions, vector<float> velocities, vector<float> torques);
        void dataplot(vector<float> data);
        
        // Other
        void rosSpinOnce();

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
};


#endif //DUNG_BEETLE_CONTROLLER_REALROSCLASS_H
