//
// Created by mat on 8/2/17.
//

#ifndef ROS_HEXAPOD_CONTROLLER_SIMROSCLASS_H
#define ROS_HEXAPOD_CONTROLLER_SIMROSCLASS_H

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <rosgraph_msgs/Clock.h>
#include "sensor_msgs/Joy.h"


class simRosClass {
private:
    // Topics Name
    // coppeliaSim related topics
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
    ros::Publisher startSimPub;
    ros::Publisher pauseSimPub;
    ros::Publisher stopSimPub;
    ros::Publisher enableSyncModePub;
    ros::Publisher triggerNextStepPub;
    // Subscribers
    ros::Subscriber subSimulationTimeSub;
    ros::Subscriber subTerminateNodeSub;
    ros::Subscriber simulationStepDoneSub;
    ros::Subscriber simulationStateSub;
    
    // Controller
    // Publisher
    ros::Publisher motorCommPub;
    ros::Publisher motorMaxTorqueCommPub;
    ros::Publisher dataPlotPub;
    // Subscribers
    ros::Subscriber jointPositionSub;
    ros::Subscriber jointVelocitySub;
    ros::Subscriber jointTorqueSub;
    ros::Subscriber testParametersSub;
    ros::Subscriber footContactSub;
    ros::Subscriber imuSub;
    ros::Subscriber joySub;

    //neuroVis
    ros::Publisher neuron_activityPub;
    ros::Publisher C_matrixPub;
    ros::Publisher norm_biasPub;
    ros::Publisher norm_gainPub;
    ros::Publisher N_namePub;

    // Private Global Variables
    struct timeval tv;
    __time_t currentTime_updatedByTopicSubscriber=0;
    bool simStepDone=true;
    ros::Rate* rate;

    // Private Methods
    // CoppeliaSim
    void simulationTimeCallback(const std_msgs::Float32& simTime);
    void simulationStepDoneCallback(const std_msgs::Bool& simStepDoneAns);
    void simulationStateCallback(const std_msgs::Int32& stateAns);
    void terminateNodeCallback(const std_msgs::Bool& termNode);
    // Controller
    void jointPositionCallback(const std_msgs::Float32MultiArray& jointPositions);
    void jointVelocityCallback(const std_msgs::Float32MultiArray& jointVelocities);
    void jointTorqueCallback(const std_msgs::Float32MultiArray& jointTorques);
    void testParametersCallback(const std_msgs::Float32MultiArray& testParameters);
    void footContactCallback(const std_msgs::Float32MultiArray& forceSensors);
    void imuCallback(const std_msgs::Float32MultiArray& inclinationSensors);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

public:
    // Public Methods
    int simState=0;
    simRosClass(int argc, char *argv[]);
    ~simRosClass();
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


#endif //ROS_HEXAPOD_CONTROLLER_SIMROSCLASS_H
