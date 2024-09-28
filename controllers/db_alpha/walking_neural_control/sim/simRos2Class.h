//
// Created by mat on 8/2/17.
//

#ifndef DUNG_BEETLE_CONTROLLER_SIMROS2CLASS_H
#define DUNG_BEETLE_CONTROLLER_SIMROS2CLASS_H

#include <cstdio>
#include <cstdlib>
// core ros2 library
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

// std msgs
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/int32.hpp>
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
// sensor msgs
#include "sensor_msgs/msg/joy.hpp"

#include <chrono>

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class simRos2Class : public rclcpp::Node
{
private:
    // Topics Name
    // coppeliaSim related topics
    using Bool = std_msgs::msg::Bool;
    // using IMU_data = sensor_msgs::msg::Imu;
    using FloatArray = std_msgs::msg::Float32MultiArray;

    std::string startSimTopic;
    std::string pauseSimTopic;
    std::string stopSimTopic;
    std::string simulationTimeTopic;
    std::string terminateNodeTopic;
    std::string enableSyncModeTopic;
    std::string triggerNextStepTopic;
    std::string simulationStepDoneTopic;
    std::string simulationStateTopic;
    // controller Topics
    std::string motorCommTopic;
    std::string motorMaxTorqueCommTopic;
    std::string jointPositionTopic;
    std::string jointVelocityTopic;
    std::string jointTorqueTopic;
    std::string imuTopic;
    std::string footContactTopic;
    std::string testParametersTopic;
    std::string dataPlotTopic;
    std::string joyTopic;
    // neuroVis Topics
    std::string neuron_activityTopic;
    std::string C_matrixTopic;
    std::string norm_biasTopic;
    std::string norm_gainTopic;
    std::string N_nameTopic;

    // CoppeliaSim
    // Publishers
    rclcpp::Publisher<Bool>::SharedPtr startSimPub;
    rclcpp::Publisher<Bool>::SharedPtr pauseSimPub;
    rclcpp::Publisher<Bool>::SharedPtr stopSimPub;
    rclcpp::Publisher<Bool>::SharedPtr enableSyncModePub;
    rclcpp::Publisher<Bool>::SharedPtr triggerNextStepPub;
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSimulationTimeSub;
    rclcpp::Subscription<Bool>::SharedPtr subTerminateNodeSub;
    rclcpp::Subscription<Bool>::SharedPtr simulationStepDoneSub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr simulationStateSub;
    
    // Controller
    // Publisher
    rclcpp::Publisher<FloatArray>::SharedPtr motorCommPub;
    rclcpp::Publisher<FloatArray>::SharedPtr motorMaxTorqueCommPub;
    rclcpp::Publisher<FloatArray>::SharedPtr dataPlotPub;
    // Subscribers
    rclcpp::Subscription<FloatArray>::SharedPtr jointPositionSub;
    rclcpp::Subscription<FloatArray>::SharedPtr jointVelocitySub;
    rclcpp::Subscription<FloatArray>::SharedPtr jointTorqueSub;
    rclcpp::Subscription<FloatArray>::SharedPtr testParametersSub;
    rclcpp::Subscription<FloatArray>::SharedPtr footContactSub;
    rclcpp::Subscription<FloatArray>::SharedPtr imuSub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub;

    //neuroVis
    rclcpp::Publisher<FloatArray>::SharedPtr neuron_activityPub;
    rclcpp::Publisher<FloatArray>::SharedPtr C_matrixPub;
    rclcpp::Publisher<FloatArray>::SharedPtr norm_biasPub;
    rclcpp::Publisher<FloatArray>::SharedPtr norm_gainPub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr N_namePub;

    // Private Global Variables
    struct timeval tv;
    __time_t currentTime_updatedByTopicSubscriber=0;
    bool simStepDone=true;
    rclcpp::Rate * loop_rate;

    // Private Methods
    // CoppeliaSim
    void simulationTimeCallback(const std_msgs::msg::Float32& simTime);
    void simulationStepDoneCallback(const std_msgs::msg::Bool& simStepDoneAns);
    void simulationStateCallback(const std_msgs::msg::Int32& stateAns);
    void terminateNodeCallback(const std_msgs::msg::Bool& termNode);
    // Controller
    void jointPositionCallback(const std_msgs::msg::Float32MultiArray& jointPositions);
    void jointVelocityCallback(const std_msgs::msg::Float32MultiArray& jointVelocities);
    void jointTorqueCallback(const std_msgs::msg::Float32MultiArray& jointTorques);
    void testParametersCallback(const std_msgs::msg::Float32MultiArray& testParameters);
    void footContactCallback(const std_msgs::msg::Float32MultiArray& forceSensors);
    void imuCallback(const std_msgs::msg::Float32MultiArray& inclinationSensors);
    void joyCallback(const sensor_msgs::msg::Joy::ConstPtr& joy);

public:
    // Public Methods
    int simState=0;
    simRos2Class(int argc, char *argv[]);
    ~simRos2Class();
    void plot(std::vector<float> data);
    void setLegMotorPosition(std::vector<float> positions);
    void setMotorMaxTorque(std::vector<float> jointMaxTorque);
    void rosSpinOnce();
    void synchronousSimulation(std::string option);

    //neuroVis
    void publish_neuron_activity(std::vector<float> neuron_activity);
    void publish_C_matrix(std::vector<float> C_matrix);
    void publish_norm_bias(std::vector<float> norm_bias);
    void publish_norm_gain(std::vector<float> norm_gain);
    void publish_N_name(std::string N_name);

    std::vector<float> getJointFeedback();
    std::vector<float> getTorqueFeedback();
    std::vector<float> getFootContactFeedback();
    std::vector<float> getTestParam();
    std::vector<float> getImuFeedback();
    std::vector<float> getJoyAxesFeedback();
    std::vector<int> getJoyButtonFeedback();
    float getSimTime();
    bool getSimTerminateState();
    int getSimState();

    // Public Global Variables
    std::vector<float> testParameters		={0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
    std::vector<float> jointPositions		={0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0};
    std::vector<float> jointVelocities		={0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
    std::vector<float> jointTorques			={0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
    std::vector<float> footContactSensors	={0,0,0, 0,0,0};
    std::vector<float> imuSensors	        ={0,0,0, 0,0,0, 0,0,0};
    std::vector<float> axes             	={0,0,0,0,0,0,0,0};
    std::vector<int> buttons            	={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float simulationTime=0.0;
    bool terminateSimulation = false;
};


#endif //ROS_HEXAPOD_CONTROLLER_SIMROS2CLASS_H
