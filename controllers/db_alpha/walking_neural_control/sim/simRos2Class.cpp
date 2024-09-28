//
// Created by mat on 8/2/17.
//

#include "simRos2Class.h"

simRos2Class::simRos2Class(int argc, char *argv[]) 
: Node("db_beta_controller_sim_node")
{

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
    // if (gettimeofday(&tv, NULL)==0)
    //     currentTime_updatedByTopicSubscriber=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
    // std::string nodeName("simROS");
    // std::string randId(boost::lexical_cast<std::string>(currentTime_updatedByTopicSubscriber+int(999999.0f*(rand()/(float)RAND_MAX))));
    // nodeName+=randId;

    // CoppeliaSim
	std::cout << "Run db_beta_controller_sim" << std::endl;

    int rate = 60; //Hz
    loop_rate = new rclcpp::Rate(rate);

    RCLCPP_INFO(this->get_logger(), "init simRos2Class_node");

    // Subscribe to topics and specify callback functions
    subSimulationTimeSub = this->create_subscription<std_msgs::msg::Float32>(simulationTimeTopic, 1, std::bind(&simRos2Class::simulationTimeCallback, this, _1));
    subTerminateNodeSub = this->create_subscription<Bool>(terminateNodeTopic, 1, std::bind(&simRos2Class::terminateNodeCallback, this, _1));
    simulationStepDoneSub = this->create_subscription<Bool>(simulationStepDoneTopic, 1, std::bind(&simRos2Class::simulationStepDoneCallback, this, _1));
    simulationStateSub = this->create_subscription<std_msgs::msg::Int32>(simulationStateTopic, 1, std::bind(&simRos2Class::simulationStateCallback, this, _1));
    jointPositionSub = this->create_subscription<FloatArray>(jointPositionTopic, 1, std::bind(&simRos2Class::jointPositionCallback, this, _1));
    jointVelocitySub = this->create_subscription<FloatArray>(jointVelocityTopic, 1, std::bind(&simRos2Class::jointVelocityCallback, this, _1));
    jointTorqueSub = this->create_subscription<FloatArray>(jointTorqueTopic, 1, std::bind(&simRos2Class::jointTorqueCallback, this, _1));
    testParametersSub = this->create_subscription<FloatArray>(testParametersTopic, 1, std::bind(&simRos2Class::testParametersCallback, this, _1));
    footContactSub = this->create_subscription<FloatArray>(footContactTopic, 1, std::bind(&simRos2Class::footContactCallback, this, _1));
    imuSub = this->create_subscription<FloatArray>(imuTopic, 1, std::bind(&simRos2Class::imuCallback, this, _1));
    joySub = this->create_subscription<sensor_msgs::msg::Joy>(joyTopic, 1, std::bind(&simRos2Class::joyCallback, this, _1));

    // Initialize publishers
    motorCommPub = this->create_publisher<FloatArray>(motorCommTopic,1);
    motorMaxTorqueCommPub = this->create_publisher<FloatArray>(motorMaxTorqueCommTopic,1);
    startSimPub = this->create_publisher<Bool>(startSimTopic,1);
    pauseSimPub = this->create_publisher<Bool>(pauseSimTopic,1);
    stopSimPub = this->create_publisher<Bool>(stopSimTopic,1);
    enableSyncModePub = this->create_publisher<Bool>(enableSyncModeTopic,1);
    triggerNextStepPub = this->create_publisher<Bool>(triggerNextStepTopic,1);
    dataPlotPub = this->create_publisher<FloatArray>(dataPlotTopic,1);

    // neuroVis publishers
    // neuron_activityPub=node.advertise<std_msgs::Float32MultiArray>(neuron_activityTopic,1);
    // C_matrixPub=node.advertise<std_msgs::Float32MultiArray>(C_matrixTopic,1);
    // norm_biasPub=node.advertise<std_msgs::Float32MultiArray>(norm_biasTopic,1);
    // norm_gainPub=node.advertise<std_msgs::Float32MultiArray>(norm_gainTopic,1);
    // N_namePub=node.advertise<std_msgs::String>(N_nameTopic,1);

    // TODO: set /use_sim_time = true --> Currently not working due to lag?
    // rate = new ros::Rate(17*4); // was 25

    RCLCPP_INFO(this->get_logger(), "finish setup simRos2Class_node");
}


void simRos2Class::simulationTimeCallback(const std_msgs::msg::Float32& simTime)
{
    simulationTime=simTime.data;
}

void simRos2Class::terminateNodeCallback(const std_msgs::msg::Bool& termNode)
{
    terminateSimulation=termNode.data;
}

void simRos2Class::simulationStepDoneCallback(const std_msgs::msg::Bool& _simStepDone)
{
    simStepDone=_simStepDone.data;
}

void simRos2Class::simulationStateCallback(const std_msgs::msg::Int32& _state)
{
    simState=_state.data;
}

void simRos2Class::jointPositionCallback(const std_msgs::msg::Float32MultiArray& _jointPositions)
{
    // ROS_INFO("The BC position is: %f", _jointPositions.data[6]);
    jointPositions = _jointPositions.data;
}

void simRos2Class::jointVelocityCallback(const std_msgs::msg::Float32MultiArray& _jointVelocities)
{
    //ROS_INFO("The BC position is: %f", _jointPositions.data[0]);
    jointVelocities = _jointVelocities.data;
}

void simRos2Class::jointTorqueCallback(const std_msgs::msg::Float32MultiArray& _jointTorques)
{
    //ROS_INFO("The BC position is: %f", _jointPositions.data[0]);
    jointTorques = _jointTorques.data;
}

void simRos2Class::testParametersCallback(const std_msgs::msg::Float32MultiArray& _testParameters) {
    testParameters = _testParameters.data;
}

void simRos2Class::footContactCallback(const std_msgs::msg::Float32MultiArray& _forceSensors) {
    footContactSensors = _forceSensors.data;
}

void simRos2Class::imuCallback(const std_msgs::msg::Float32MultiArray& _inclinationSensors) {
    imuSensors = _inclinationSensors.data;
}

void simRos2Class::joyCallback(const sensor_msgs::msg::Joy::ConstPtr& joy){
    axes = joy->axes;
    buttons = joy->buttons;
}

void simRos2Class::setLegMotorPosition(std::vector<float> positions) {
    // publish the motor positions:
    std_msgs::msg::Float32MultiArray array;
    array.data.clear();
    for (int i = 0; i < 19; ++i) {
        array.data.push_back(positions[i]);
    }
    motorCommPub->publish(array);
}

void simRos2Class::setMotorMaxTorque(std::vector<float> jointMaxTorque) {
    // publish the motor positions:
    std_msgs::msg::Float32MultiArray array;
    array.data.clear();
    for (int i = 0; i < 18; ++i) {
        array.data.push_back(jointMaxTorque[i]);
    }
    motorMaxTorqueCommPub->publish(array);
}

void simRos2Class::plot(std::vector<float> data) {
    // publish the motor positions:
    std_msgs::msg::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i <= data.size(); ++i){
        array.data.push_back(data[i]);
    }
    dataPlotPub->publish(array);
}

// publish neurovis
void simRos2Class::publish_neuron_activity(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::msg::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    neuron_activityPub->publish(array);
}
void simRos2Class::publish_C_matrix(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::msg::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    C_matrixPub->publish(array);
    // std::cout << array << std::endl;
}
void simRos2Class::publish_norm_bias(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::msg::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    norm_biasPub->publish(array);
}
void simRos2Class::publish_norm_gain(std::vector<float> data) {
    // publish neuron activity:
    std_msgs::msg::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); ++i)
        array.data.push_back(data[i]);

    norm_gainPub->publish(array);
}
void simRos2Class::publish_N_name(std::string data) {
    // publish neuron activity:
    std_msgs::msg::String msg;
    msg.data = data;
    N_namePub->publish(msg);
}



void simRos2Class::rosSpinOnce(){
    bool rateMet = loop_rate->sleep();

    if(!rateMet)
    {
        RCLCPP_ERROR(rclcpp::get_logger("db_beta_controller"), "Sleep rate not met");
    }
}

void simRos2Class::synchronousSimulation(std::string option){
    std_msgs::msg::Bool _bool;
    _bool.data = true;

    if(option == "enable")
    {
        enableSyncModePub->publish(_bool);
        startSimPub->publish(_bool);
    }
    else if(option == "trigger")
    {
        triggerNextStepPub->publish(_bool);
    }
}

simRos2Class::~simRos2Class() {
    RCLCPP_INFO(this->get_logger(), "simROS just terminated!");
    rclcpp::shutdown();
}

std::vector<float> simRos2Class::getJointFeedback(){
	return jointPositions;
}

std::vector<float> simRos2Class::getTorqueFeedback(){
	return jointTorques;
}

std::vector<float> simRos2Class::getFootContactFeedback(){
	return footContactSensors;
}


std::vector<float> simRos2Class::getTestParam(){
	return testParameters;
}

std::vector<float> simRos2Class::getImuFeedback(){
	return imuSensors;
}

std::vector<float> simRos2Class::getJoyAxesFeedback(){
	return axes;
}

std::vector<int> simRos2Class::getJoyButtonFeedback(){
	return buttons;
}

float simRos2Class::getSimTime(){
	return simulationTime;
}

bool simRos2Class::getSimTerminateState(){
	return terminateSimulation;
}

int simRos2Class::getSimState(){
	return simState;
}