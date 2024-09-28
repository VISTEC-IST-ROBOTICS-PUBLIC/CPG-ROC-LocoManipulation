
#include "realRos2Class.h"

float degtoRad = PI/180.0;
// using namespace std;

// Constructor
realRos2Class::realRos2Class(int argc,char* argv[])
: Node("db_beta_controller_node")
{
    // const char * argc;
    // char * argv[];
    // rclcpp::init(argc, argv);
    // std::cout << "Run db_beta_controller" << std::endl;

    // auto db_beta_controller = std::make_shared<realRos2Class>();
    // std::cout << "init real ros2 class node" << std::endl;
	// rclcpp::init(argc, argv);
	std::cout << "Run db_beta_controller" << std::endl;

    int rate = 60; //Hz
    loop_rate = new rclcpp::Rate(rate);

    RCLCPP_INFO(this->get_logger(), "init realRos2Class_node");

    // this->declare_parameter("qos_depth", 10);
    // int8_t qos_depth = 0;
    // this->get_parameter("qos_depth", qos_depth);

    // const auto QOS_RKL10V =
    // rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    // set up Publisher
    joint_newstates_publisher = this->create_publisher<JointState>("motor_command", 10);

    data_plot_publisher = this->create_publisher<FloatArray>("plot", 10);
    // timer_ = this->create_wall_timer(
    // 10ms, std::bind(&realRos2Class::updateMotorState, this));


    // set up subscriber
    joint_states_subscriber = this->create_subscription<JointState>(
    "motor_feedback", 1, std::bind(&realRos2Class::jointstates_subscribe_callback, this, _1));

    imu_subscriber = this->create_subscription<IMU_data>(
    "/imu/data", 1, std::bind(&realRos2Class::imu_callback, this, _1));


    // // Create a ROS nodes
    // int _argc = 0;
    // char** _argv = NULL;
    // ros::init(_argc,_argv,"db_alpha_controller_node");

    // if(!ros::master::check())
    //     ROS_ERROR("ros::master::check() did not pass!");

    // ros::NodeHandle node("~");
    // ROS_INFO("ROS control node just started!");

    // // Initialize Subscribers
    // joint_states=node.subscribe("/db_dynamixel_ROS_driver/hexapod_joint_feedback", 1, &realRos2Class::jointStatesCallback, this);;
    // joySub=node.subscribe("/joy", 1, &realRos2Class::joy_CB, this);
    // imuSub=node.subscribe("/imu/data", 1, &realRos2Class::imu_CB, this);
    // // imuEulerSub=node.subscribe("/imu_transform/euler", 1, &realRos2Class::imuEuler_CB, this);

    // // Initialize Publishers: Change this lines depending on what protocol you are using.
    // //jointControlPub=node.advertise<std_msgs::Float32MultiArray>("/db_dynamixel_ROS_driver/hexapod_multi_joint_commands",1);
    // jointNewStatesPub=node.advertise<sensor_msgs::JointState>("/db_dynamixel_ROS_driver/hexapod_state_commands",1);
    // plotPub=node.advertise<std_msgs::Float32MultiArray>("/plot",1);

    // // Set Rate
    // rate = new ros::Rate(60); // 60hz// 17*4
    // // ros::Rate rate(60);

}

// Destructor
realRos2Class::~realRos2Class() 
{

}

// //--------------------------------------------------------------------------------------------------------------------------------------


// CALLBACKS

// JointState feedback callback
void realRos2Class::jointstates_subscribe_callback(const JointState &msg)
{
    // std::cout << "test joint: " << endl;
    joint_feedback_state_ = msg;
}


// //-------------------------------------------------------------------------------------------------------------------------


// // PUBLISHING METHODS


// // BEFORE SENDING MOTOR COMMANDS, GO TO THE DYNAMIXEL CODE AND CHANGE THE CONTROL MODE

// // * Position Control - Open Loop, No compliance
// // * Torque Control - Closed Loop, Compliant


// // Position Control
// void realRos2Class::setLegMotorPosition(std::vector<float> positions)
// {
//     // publish the motor positions:
//     std_msgs::Float32MultiArray array;
//     array.data.clear();

//     for (float position : positions) 
//     {
//         array.data.push_back(position);
//     }

//     jointControlPub.publish(array);
// }

// // Torque Control
// void realRos2Class::setLegMotorTorques(std::vector<float> torques) 
// {
//     // Publish motor torques
//     std_msgs::Float32MultiArray array;
//     array.data.clear();

//     for (float torque : torques) 
//     {
//         array.data.push_back(torque);
//     }

//     jointControlPub.publish(array);
// }


// Publish JointState updates
void realRos2Class::updateMotorState(vector<string> names, vector<float> positions, vector<float> velocities, vector<float> torques)
{
    std::cout << "updateMotorState and publish joint state" << std::endl;
    // Publish joint state 
    update_state_.header.stamp = this->get_clock()->now();

    update_state_.name.clear();
    update_state_.position.clear();
    update_state_.velocity.clear();
    update_state_.effort.clear();

    for(int i=0; i<names.size(); i++)
    {
        update_state_.name.push_back(names[i]);
        update_state_.position.push_back(positions[i]);
        update_state_.velocity.push_back(velocities[i]);
        update_state_.effort.push_back(torques[i]);
    }

    joint_newstates_publisher->publish(update_state_);
}


// //--------------------------------------------------------------------------------------------------------------------------------------

// Publish Plot Data updates
void realRos2Class::dataPlot(vector<float> data)
{
    // publish Plot Data:
    FloatArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); i++){
        array.data.push_back(data[i]);
    }

    data_plot_publisher->publish(array);
}


//--------------------------------------------------------------------------------------------------------------------------------------

// void realRos2Class::jointNameArray()
// {   
//     jointPositions.clear();
//     jointVelocities.clear();
//     jointTorques.clear();

//     int index = 0;
//     for(std::string name : _state_.name)
//     {
//         //jointIDs.push_back(name); 
//         jointPositions.push_back(_state_.position[index]);
//         jointVelocities.push_back(_state_.velocity[index]);
//         jointTorques.push_back(_state_.effort[index]);
//         index++;
//     }
//     std::cout << "joint feedback Position motor_1 : " << jointPositions[0] << std::endl;
// }

// std::vector<float> realRos2Class::getJointTimeStamp()
// {   
    
// }

sensor_msgs::msg::JointState realRos2Class::getJointFeedback()
{   
    return joint_feedback_state_;
}

std::vector<float> realRos2Class::getImuFeedback()
{   
    return imu;
}

// // joy callback
// void realRos2Class::joy_CB(const sensor_msgs::Joy::ConstPtr& joy){
//     axes = joy->axes;
//     buttons = joy->buttons;
// }

// imu callback
void realRos2Class::imu_callback(const IMU_data& imu_){
    std::cout << "test imu: " << endl;
    // imu[0] = imu_->angular_velocity.x; // simple convention
    // imu[1] = imu_->angular_velocity.y;
    // imu[2] = imu_->angular_velocity.z;
    imu[0] =  imu_.angular_velocity.y/degtoRad; // convert to robot convention
    imu[1] = -imu_.angular_velocity.x/degtoRad;
    imu[2] =  imu_.angular_velocity.z/degtoRad;
    imu[3] =  imu_.linear_acceleration.y;
    imu[4] = -imu_.linear_acceleration.x;
    imu[5] =  imu_.linear_acceleration.z;
    imu[6] =  imu_.orientation.x;
    imu[7] =  imu_.orientation.y;
    imu[8] =  imu_.orientation.z;
    imu[9] =  imu_.orientation.w;

    tf2::Quaternion q( 
        imu_.orientation.x,
        imu_.orientation.y,
        imu_.orientation.z,
        imu_.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // roll, pitch, yaw in phidgets imu attachment orientation
    // convert to roll, pitch, yaw of the robot imu[10]-[12] = roll, pitch, yaw
    imu[10] = pitch/degtoRad;
    imu[11] = -roll/degtoRad;
    imu[12] = yaw/degtoRad;
    
	// omega = std::vector<float> (imu.begin(), imu.begin() + 3);
	// angle = std::vector<float> (imu.begin()+10, imu.end());
}

// // imu_Eluer degree
// void realRos2Class::imuEuler_CB(const geometry_msgs::Vector3::ConstPtr& imuEuler_){
//     imu[10] = imuEuler_->x;
//     imu[11] = imuEuler_->y;
//     imu[12] = imuEuler_->z;
// }

// // ROS spin
void realRos2Class::ros2_spin_some()
{
    bool rateMet = loop_rate->sleep();
    std::cout << "test: " << endl;

    if(!rateMet)
    {
        RCLCPP_ERROR(rclcpp::get_logger("db_beta_controller"), "Sleep rate not met");
    }

}


