//
// Created by Carlos on 05/2019.
//

#include "realRosClass.h"

float degtoRad = PI/180.0;


// Constructor
realRosClass::realRosClass(int argc, char **argv)
{
    // Create a ROS nodes
    int _argc = 0;
    char** _argv = NULL;
    ros::init(_argc,_argv,"db_alpha_controller_node");

    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");

    ros::NodeHandle node("~");
    ROS_INFO("ROS control node just started!");

    // Initialize Subscribers
    joint_states=node.subscribe("/db_dynamixel_ROS_driver/hexapod_joint_feedback", 1, &realRosClass::jointStatesCallback, this);;
    joySub=node.subscribe("/joy", 1, &realRosClass::joy_CB, this);
    imuSub=node.subscribe("/imu/data", 1, &realRosClass::imu_CB, this);
    // imuEulerSub=node.subscribe("/imu_transform/euler", 1, &realRosClass::imuEuler_CB, this);

    // Initialize Publishers: Change this lines depending on what protocol you are using.
    //jointControlPub=node.advertise<std_msgs::Float32MultiArray>("/db_dynamixel_ROS_driver/hexapod_multi_joint_commands",1);
    jointNewStatesPub=node.advertise<sensor_msgs::JointState>("/db_dynamixel_ROS_driver/hexapod_state_commands",1);
    plotPub=node.advertise<std_msgs::Float32MultiArray>("/plot",1);

    // Set Rate
    rate = new ros::Rate(60); // 60hz// 17*4
    // ros::Rate rate(60);

}


//--------------------------------------------------------------------------------------------------------------------------------------


// CALLBACKS

// JointState feedback callback
void realRosClass::jointStatesCallback(const sensor_msgs::JointState& msg)
{
    _state_ = msg;
}


//-------------------------------------------------------------------------------------------------------------------------


// PUBLISHING METHODS


// BEFORE SENDING MOTOR COMMANDS, GO TO THE DYNAMIXEL CODE AND CHANGE THE CONTROL MODE

// * Position Control - Open Loop, No compliance
// * Torque Control - Closed Loop, Compliant


// Position Control
void realRosClass::setLegMotorPosition(std::vector<float> positions)
{
    // publish the motor positions:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (float position : positions) 
    {
        array.data.push_back(position);
    }

    jointControlPub.publish(array);
}

// Torque Control
void realRosClass::setLegMotorTorques(std::vector<float> torques) 
{
    // Publish motor torques
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (float torque : torques) 
    {
        array.data.push_back(torque);
    }

    jointControlPub.publish(array);
}


// Publish JointState updates
void realRosClass::updateMotorState(vector<string> names, vector<float> positions, vector<float> velocities, vector<float> torques)
{
    // Publish joint state 
    update_state_.header.stamp = ros::Time::now();

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

    jointNewStatesPub.publish(update_state_);
}


//--------------------------------------------------------------------------------------------------------------------------------------

// Publish Plot Data updates
void realRosClass::dataplot(vector<float> data)
{
    // publish Plot Data:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < data.size(); i++){
        array.data.push_back(data[i]);
    }

    plotPub.publish(array);
}


//--------------------------------------------------------------------------------------------------------------------------------------

void realRosClass::getRobotFeedback()
{   
    jointPositions.clear();
    jointVelocities.clear();
    jointTorques.clear();

    int index = 0;
    for(std::string name : _state_.name)
    {
        //jointIDs.push_back(name); 
        jointPositions.push_back(_state_.position[index]);
        jointVelocities.push_back(_state_.velocity[index]);
        jointTorques.push_back(_state_.effort[index]);
        index++;
    }
}

// joy callback
void realRosClass::joy_CB(const sensor_msgs::Joy::ConstPtr& joy){
    axes = joy->axes;
    buttons = joy->buttons;
}

// imu callback
void realRosClass::imu_CB(const sensor_msgs::Imu::ConstPtr& imu_){
    // imu[0] = imu_->angular_velocity.x; // simple convention
    // imu[1] = imu_->angular_velocity.y;
    // imu[2] = imu_->angular_velocity.z;
    imu[0] =  imu_->angular_velocity.y/degtoRad; // convert to robot convention
    imu[1] = -imu_->angular_velocity.x/degtoRad;
    imu[2] =  imu_->angular_velocity.z/degtoRad;
    imu[3] =  imu_->linear_acceleration.y;
    imu[4] = -imu_->linear_acceleration.x;
    imu[5] =  imu_->linear_acceleration.z;
    imu[6] =  imu_->orientation.x;
    imu[7] =  imu_->orientation.y;
    imu[8] =  imu_->orientation.z;
    imu[9] =  imu_->orientation.w;

    tf2::Quaternion q( 
        imu_->orientation.x,
        imu_->orientation.y,
        imu_->orientation.z,
        imu_->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // roll, pitch, yaw in phidgets imu attachment orientation
    // convert to toll, pitch, yaw of the robot imu[10]-12 = roll, pitch, yaw
    imu[10] = pitch/degtoRad;
    imu[11] = -roll/degtoRad;
    imu[12] = yaw/degtoRad;
    
	omega = std::vector<float> (imu.begin(), imu.begin() + 3);
	angle = std::vector<float> (imu.begin()+10, imu.end());
 	// apply sensors filters
	// for(int i = 0; i < 3; i++){
	// 	omega[i] = (angle_input_w*omega[i] + angle_self_w*old_omega[i])*0.2; // rad/s
	// 	angle[i] = (angle_input_w*angle[i] + angle_self_w*old_angle[i]); 	 // rad
		// accel[i] = roll_angle_input_w*accel[i] + roll_angle_self_w*accel[i];
	// }
	// old_omega = omega;
	// old_angle = angle;
	// old_accel = accel;
   
    // cout << roll << "  "
    //      << pitch << "  "
    //      << yaw << "  "<< endl;

}

// imu_Eluer degree
void realRosClass::imuEuler_CB(const geometry_msgs::Vector3::ConstPtr& imuEuler_){
    imu[10] = imuEuler_->x;
    imu[11] = imuEuler_->y;
    imu[12] = imuEuler_->z;
}

// ROS spin
void realRosClass::rosSpinOnce()
{
    ros::spinOnce();
    bool rateMet = rate->sleep();

    if(!rateMet)
    {
        ROS_ERROR("Sleep rate not met");
    }

}


// Destructor
realRosClass::~realRosClass() 
{
    ROS_INFO("realROS just terminated!");
    ros::shutdown();
}
