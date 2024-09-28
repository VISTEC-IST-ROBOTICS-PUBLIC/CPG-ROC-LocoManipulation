//
// Created by mat on 8/2/17.
//

#include "simRosClass.h"

simRosClass::simRosClass(int argc, char **argv) {

    // CoppeliaSim
    startSimTopic="/startSimulation";
    pauseSimTopic="/pauseSimulation";
    stopSimTopic="/stopSimulation";
    simulationTimeTopic="/simulationTime";
    terminateNodeTopic="/terminateController";
    enableSyncModeTopic="/enableSyncMode";
    triggerNextStepTopic="/triggerNextStep";
    simulationStepDoneTopic="/simulationStepDone";
    simulationStateTopic="/simulationState";
    // Controller Topics
    motorCommTopic="/motorCommand";
    motorMaxTorqueCommTopic="/motorMaxTorqueCommand";
    jointPositionTopic="/jointPosition";
    jointVelocityTopic="/jointVelocity";
    testParametersTopic="/tuneParameter";
    footContactTopic="/footContact";
    imuTopic="/imu";
    jointTorqueTopic="/jointTorque";
    dataPlotTopic="/dataPlot";
    joyTopic="/joy";

    neuron_activityTopic="/neuron_activity";
    C_matrixTopic="/C_matrix";
    norm_biasTopic="/norm_bias";
    norm_gainTopic="/norm_gain";
    N_nameTopic="/N_name";

    // Create a ROS nodes (The name has a random component)
    int _argc = 0;
    char** _argv = NULL;
    if (gettimeofday(&tv, NULL)==0)
        currentTime_updatedByTopicSubscriber=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
    std::string nodeName("simROS");
    std::string randId(boost::lexical_cast<std::string>(currentTime_updatedByTopicSubscriber+int(999999.0f*(rand()/(float)RAND_MAX))));
    nodeName+=randId;
    ros::init(_argc,_argv,nodeName);

    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");

    ros::NodeHandle node("~");
    ROS_INFO("simROS just started!");

    // Subscribe to topics and specify callback functions
    subSimulationTimeSub=node.subscribe(simulationTimeTopic, 1, &simRosClass::simulationTimeCallback, this);
    subTerminateNodeSub=node.subscribe(terminateNodeTopic, 1, &simRosClass::terminateNodeCallback, this);
    simulationStepDoneSub=node.subscribe(simulationStepDoneTopic, 1, &simRosClass::simulationStepDoneCallback, this);
    simulationStateSub=node.subscribe(simulationStateTopic, 1, &simRosClass::simulationStateCallback, this);
    jointPositionSub=node.subscribe(jointPositionTopic, 1, &simRosClass::jointPositionCallback, this);
    jointVelocitySub=node.subscribe(jointVelocityTopic, 1, &simRosClass::jointVelocityCallback, this);
    jointTorqueSub=node.subscribe(jointTorqueTopic, 1, &simRosClass::jointTorqueCallback, this);
    testParametersSub=node.subscribe(testParametersTopic, 1, &simRosClass::testParametersCallback, this);
    footContactSub=node.subscribe(footContactTopic, 1, &simRosClass::footContactCallback, this);
    imuSub=node.subscribe(imuTopic, 1, &simRosClass::imuCallback, this);
    joySub=node.subscribe(joyTopic, 1, &simRosClass::joyCallback, this);

    // Initialize publishers
    motorCommPub=node.advertise<std_msgs::Float32MultiArray>(motorCommTopic,1);
    motorMaxTorqueCommPub=node.advertise<std_msgs::Float32MultiArray>(motorMaxTorqueCommTopic,1);
    startSimPub=node.advertise<std_msgs::Bool>(startSimTopic,1);
    pauseSimPub=node.advertise<std_msgs::Bool>(pauseSimTopic,1);
    stopSimPub=node.advertise<std_msgs::Bool>(stopSimTopic,1);
    enableSyncModePub=node.advertise<std_msgs::Bool>(enableSyncModeTopic,1);
    triggerNextStepPub=node.advertise<std_msgs::Bool>(triggerNextStepTopic,1);
    dataPlotPub=node.advertise<std_msgs::Float32MultiArray>(dataPlotTopic,1);

    // neuroVis publishers
    // neuron_activityPub=node.advertise<std_msgs::Float32MultiArray>(neuron_activityTopic,1);
    // C_matrixPub=node.advertise<std_msgs::Float32MultiArray>(C_matrixTopic,1);
    // norm_biasPub=node.advertise<std_msgs::Float32MultiArray>(norm_biasTopic,1);
    // norm_gainPub=node.advertise<std_msgs::Float32MultiArray>(norm_gainTopic,1);
    // N_namePub=node.advertise<std_msgs::String>(N_nameTopic,1);

    // TODO: set /use_sim_time = true --> Currently not working due to lag?
    rate = new ros::Rate(17*4); // was 25
}


void simRosClass::simulationTimeCallback(const std_msgs::Float32& simTime)
{
    simulationTime=simTime.data;
}

void simRosClass::terminateNodeCallback(const std_msgs::Bool& termNode)
{
    terminateSimulation=termNode.data;
}

void simRosClass::simulationStepDoneCallback(const std_msgs::Bool& _simStepDone)
{
    simStepDone=_simStepDone.data;
}

void simRosClass::simulationStateCallback(const std_msgs::Int32& _state)
{
    simState=_state.data;
}

void simRosClass::jointPositionCallback(const std_msgs::Float32MultiArray& _jointPositions)
{
    // ROS_INFO("The BC position is: %f", _jointPositions.data[6]);
    jointPositions = _jointPositions.data;
}

void simRosClass::jointVelocityCallback(const std_msgs::Float32MultiArray& _jointVelocities)
{
    //ROS_INFO("The BC position is: %f", _jointPositions.data[0]);
    jointVelocities = _jointVelocities.data;
}

void simRosClass::jointTorqueCallback(const std_msgs::Float32MultiArray& _jointTorques)
{
    //ROS_INFO("The BC position is: %f", _jointPositions.data[0]);
    jointTorques = _jointTorques.data;
}

void simRosClass::testParametersCallback(const std_msgs::Float32MultiArray& _testParameters) {
    testParameters = _testParameters.data;
}

void simRosClass::footContactCallback(const std_msgs::Float32MultiArray& _forceSensors) {
    footContactSensors = _forceSensors.data;
}

void simRosClass::imuCallback(const std_msgs::Float32MultiArray& _inclinationSensors) {
    imuSensors = _inclinationSensors.data;
}

void simRosClass::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    axes = joy->axes;
    buttons = joy->buttons;
}

void simRosClass::setLegMotorPosition(std::vector<float> positions) {
    // publish the motor positions:
    std_msgs::Float32MultiArray array;
    array.data.clear();
    for (int i = 0; i < 19; ++i) {
        array.data.push_back(positions[i]);
    }
    motorCommPub.publish(array);
}

void simRosClass::setMotorMaxTorque(std::vector<float> jointMaxTorque) {
    // publish the motor positions:
    std_msgs::Float32MultiArray array;
    array.data.clear();
    for (int i = 0; i < 18; ++i) {
        array.data.push_back(jointMaxTorque[i]);
    }
    motorMaxTorqueCommPub.publish(array);
}

void simRosClass::plot(std::vector<float> data) {
    // publish the motor positions:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i <= data.size(); ++i){
        array.data.push_back(data[i]);
    }
    dataPlotPub.publish(array);
}

// publish neurovis
void simRosClass::publish_neuron_activity(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    neuron_activityPub.publish(array);
}
void simRosClass::publish_C_matrix(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    C_matrixPub.publish(array);
    std::cout << array << std::endl;
}
void simRosClass::publish_norm_bias(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    norm_biasPub.publish(array);
}
void simRosClass::publish_norm_gain(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    norm_gainPub.publish(array);
}
void simRosClass::publish_N_name(std::string data) {
    // publish neuron activity:
    std_msgs::String msg;
    msg.data = data;
    N_namePub.publish(msg);
}



void simRosClass::rosSpinOnce(){
    ros::spinOnce();
    bool rateMet = rate->sleep();
    while(!rateMet)
    {
        rateMet = rate->sleep();
//        ROS_ERROR("Sleep rate not met");
    }
}

void simRosClass::synchronousSimulation(std::string option){
    std_msgs::Bool _bool;
    _bool.data = true;

    if(option == "enable")
    {
        enableSyncModePub.publish(_bool);
        startSimPub.publish(_bool);
    }
    else if(option == "trigger")
    {
        triggerNextStepPub.publish(_bool);
    }
}

simRosClass::~simRosClass() {
    ROS_INFO("simROS just terminated!");
    ros::shutdown();
}
