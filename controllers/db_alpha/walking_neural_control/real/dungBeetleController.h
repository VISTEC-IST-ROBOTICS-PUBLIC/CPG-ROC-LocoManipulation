//
// Created by Carlos on 27/04/2019.
// From Neutron Modular Controller by Mathias Thor.
//

#ifndef DUNG_BEETLE_CONTROLLER_DUNGBEETLECONTROLLER_H
#define DUNG_BEETLE_CONTROLLER_DUNGBEETLECONTROLLER_H

// C++
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <tuple>
#include <vector>
#include <math.h>
#include <string>
#include <unistd.h>

// ROS2
#include "realRos2Class.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include <sensor_msgs/msg/joint_state.hpp>

// DB Controller headers
#include "../dbMotorDefinition.h"
#include "delayline.h"
#include "../dualIntegralLearner.h"
#include "../postProcessing.h"
#include "../AMC.h"
#include "../dbModularController.h"
#include "PDcontroller.h"
#include "../joystick.h"
#include <termios.h>
#include <string>
#include "utils/ann-library/so2cpg.h"


class dualIntegralLearner;
class modularController;
class postProcessing;
class realRos2Class;
class AMC;
class PDcontroller;
class Delayline;
class SO2CPG;

#define PI 3.14159265359

class dungBeetleController 
{
    public:
        // class
        SO2CPG * so2;
        float temp;

        bool isDenmark_db_alpha = false;
        // Constructor 
        dungBeetleController(int argc,char* argv[]);
        
        // Methods
        bool runController();

        // Variable
        float degtoRad = PI/180.0;
        float fac = 1.0;
        float fw = 1.0;
        float bw = 1.0;
        int gg = 0;
        float mleg = 0.5;
        float imu_base_x = 0.0;
        float imu_base_y = 0.0;
        float imu_base_z = 0.0;
//        float ballturn = 0.5;
		float default_MI = 0.01; // limitation test paper parameter // cybern revision MI: 0.05
		// float default_MI = 0.01; // new MI for 60 Hz rosRate, walking: 0.2
		// Rolling
		float default_front_MI = 0.15; // fast movement default: 0.15
		float default_back_MI = 0.15; // default: 0.15
		// float default_front_MI = 0.1;
		// float default_back_MI = 0.1;
		float front_MI = 0.0;
		float back_MI = 0.0;

        int data_counter = 0;
        int objective = 0;
        int FC_state = 0;

        int experiment_end_step = 100000; // 9000 grab experiment, 700 GS experiment

    	float eff_vrn3;
    	float eff_vrn4;
    	float eff_vrn3_delay;
    	float eff_vrn4_delay;

		double time_begin = 0;
		double time_end = 0;

		// Ros2 variable
		// int rate; // Hz
  		// rclcpp::WallRate loop_rate;

		// Timer 
		std::chrono::_V2::system_clock::time_point end;
		std::chrono::_V2::system_clock::time_point start;

    private:

        // PRIVATE METHODS

        // Delay
		// internal forward model
        Delayline * tau_openl_eff3;
        Delayline * tau_openl_eff4;
        std::vector<Delayline> tau_reflexChain;
        int reflex_delay;
        std::vector<Delayline> tauCF;
        int tauEff_JointFeedback = 15;

		// Joy stick
	    double joySpeed = 0;
	    double joyTurn = 0;
	    bool useJoy = false;

	    // Keyboard input
	    bool useKeyboard = true; //Set true if you want to use keyboard inputs for walking, this allows for changing gaits
	    char keyboard_input = ' ';
	    double inputCounter = 0;
	    char doMotion = ' ';
	    char olddoMotion = ' ';
	    char getch();

        // Actuation

        // Robot controllers
        void actuateRobot_standTorque();
        void actuateRobot_walkingTorque();
        void actuateRobot_standPosition(int mode);
        void actuateRobot_walkingPosition();
        void standAndWalk();
        void standAndWalkTorque();
        void actuateRobot_Rolling_pose_Position();
        void standAndRoll();
        void knock();
        vector<float> trimJointMinMax(vector<float> array);
        void updateJointFeedback();
        void updateImuFeedback();

        float err_spike = 0, olderr_sp = 0, err_sp = 0;

        // One leg controllers
        void stand_one_leg();

        // Numerical
        float rescale(float oldMax, float oldMin, float newMax, float newMin, float parameter);
        float convertTorque2Current(float torque);
        float convertCurrent2Torque(float current);
        vector<float> limitTorque(vector<float> torque);
		float relu(float input);
		float limitRangeFunction(float input, float lowerLimit, float upperLimit);
		float lowerLimitFunction(float input, float lowerLimit);
		float sigmoid(float x, float expfac, float a, float b);

        // PRIVATE VARIABLES

        // Control configurations
        bool full_robot = true;
        bool activate_muscle_model = false;
        float MAX_TORQUE = 4.0;
        int initial_counter = 0;
        float FREQ = 60;
        double dt = 0;
        int lower_limit = 0;
        int upper_limit = 0;
        int cpg_type;
        double phii = 0.04;
//        double mi = 0.01;
        float MAX_NETWORK_OUTPUT = 1;

		/// decouple CPG system
		float cpg_input_w_f = 0.2;
		float cpg_input_w_b = 0.1;
		std::vector<float> cpg_input_mod = {0,0,0,0,0,0};

		// Robot Feedback signal
		sensor_msgs::msg::JointState joint_state;
        std::vector<int> jointIDs           = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> jointPositions   = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> jointVelocities  = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> jointTorques     = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
        std::vector<float> imuData          ={0,0,0, 0,0,0, 0,0,0,0, 0,0,0};
		// Joy feedback data
        std::vector<float> axes             ={0,0,0,0,0,0,0,0};
        std::vector<int> buttons            ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        // std::vector<float> omega = {0,0,0}; // declare in the roll control
        // std::vector<float> angle = {0,0,0};

        // Home position:
        vector<float> home_position = {-0.035, 0.0568, -0.082, 
                                        0.035, 0.0568, -0.082, 
                                       	-0.233, 0.316, -0.087, 
                                       	0.233, 0.316, -0.087, 
                                       	-0.103, 0.244, -0.373, 
                                       	0.103, 0.244, -0.373, 
                                        -0.3497, -0.367, 0.269};
//        vector<float> dung_beetle_pose = {-0.195, -0.3, 0.34,
//                                          0.195, -0.3, 0.34,
//                                          -0.402, -0.259, 0.36,
//                                          0.402, -0.259, 0.36,
//                                          -0.305, 0.0, 0.394,
//                                          0.305, 0.0, 0.394,
//                                          -0.1381, -0.3595, -0.0966};
//        //rubber
//        vector<float> dung_beetle_pose = {0.2619, -1.2222, -0.5238,
//        								  0.2619, -1.2222, -0.5238,
//                                          0.2794, -1.1476, -0.5238,
//                                          0.2794, -1.1476, -0.5238,
//                                          0.3492, -1.3968, -0.3492,
//                                          0.3492, -1.3968, -0.3492,
//                                          -0.2885, -0.35, 0.2};

        // silicone test V1
//		vector<float> dung_beetle_pose = {0.2619, -1.2222, -0.5238,
//										  0.2619, -1.1222, -0.5238,
//										  0.2794, -1.22, -0.5238,
//										  0.2794, -1.22, -0.5238,
//										  0.3492, -1.3968, -0.3492,
//										  0.3492, -1.4968, -0.3492,
//										  -0.2885, -0.35, 0.0};

		//V3 silicone
//		vector<float> dung_beetle_pose = {-0.1, -1.0, -0.3,
//										  -0.1, -1.0, -0.3,
//										  0.2, -0.80, -0.3,
//										  0.2, -0.80, -0.3,
//										  0.1, -1.5, -0.3,
//										  0.1, -1.5, -0.3,
//										  -0.2885, -0.35, -0.1};
		//V3 silicone TEsting
		vector<float> dung_beetle_pose = {-0.1, -0.5,  0.0,
										  -0.1, -0.5,  0.0,
										  0.2, -0.4,  -0.1, // 0.2, -0.5 bias for locomotion paper
										  0.2, -0.4,  -0.1,
										  0.2, -1.3,  0.0,
										  0.2, -1.3,  0.0,
										  -0.2885, -0.35, -0.1};

		//V3 silicone TEsting
		vector<float> dung_beetle_ball_pose = {-0.1, -0.79, -0.79,
										  	   -0.1, -0.79, -0.79,
											    0.17, -0.96, -0.7,
												0.17, -0.96, -0.7,
											    0.1, -1.31, -0.35,
											    0.1, -1.31, -0.35,
											   -0.1, -0.24, -0.1  };
		// parameters from simulation
		// std::vector<float> BC_Roll_fac = { 0.5 ,  0.4,  0.4 }; // new rolling pose locomotion paper, (base value : {0.5 ,  0.4,  0.4 })
		// std::vector<float> CF_Roll_fac = { 0.55,  0.6,  0.55}; //, (base value : {0.55,  0.6,  0.55})
		// std::vector<float> FT_Roll_fac = { 0.7 ,  0.7,  0.6};  //, (base value : {0.7 , -0.7, -0.6} is for walking)		 
		// Real robot parameters
		std::vector<float> BC_Roll_fac = { 1.0,  0.6,  0.6}; // new rolling pose locomotion paper, (base value : {0.5 ,  0.4,  0.4 })
		std::vector<float> CF_Roll_fac = { 1.0,  0.4,  0.4}; //, (base value : {0.55,  0.6,  0.55})
		std::vector<float> FT_Roll_fac = { 1.0,  0.6,  0.6};  //, (base value : {0.7 , -0.7, -0.6} is for walking)		 

		//V3 silicone Rolling_pose TEsting
//		vector<float> dung_beetle_rolling_pose = {0.0 ,  -0.72,  0.0 ,
//										  	  	  0.0 ,  -0.72,  0.0 ,
//												  0.0 ,  -0.15,  0.2 ,
//												  0.0 ,  -0.15,  0.2 ,
//												  0.35,  -1.57, -0.35,
//												  0.35,  -1.57, -0.35,
//												 -0.07,  -0.12,  0.1};

		// rolling posture test
        // Static rolling posture
		// vector<float> dung_beetle_rolling_pose = {-0.1, -0.4, -0.0,
		// 		  	  	  	  	  	  	  	  	  -0.1, -0.4, -0.0,
		// 										   0.3, -0.15 ,  0.5,
		// 										   0.3, -0.15,   0.5,
		// 										   0.2, -1.3,  0.2 ,
		// 										   0.2, -1.3,  0.2,
		// 										  0.22,  0.2,  0.1};
        // Trial Dynamic Rolling posture
		// vector<float> dung_beetle_rolling_pose = { 0.0,  -0.4,   0.3, // walkpose (-0.1, -0.5,  0.0)
		// 		  	  	  	  	  	  	  	  	   0.0,  -0.4,   0.3,
		// 										   0.2,  -0.1,   0.5, // walkpose (0.2, -0.5,  -0.1)
		// 										   0.2,  -0.1,   0.5,
		// 										   0.1,  -1.1,   0.25, // CF-joint: -1.3 (default)
		// 										   0.1,  -1.1,   0.25,
		// 										   0.15,  0.16,  0.1}; // (1st: 0.0,0.01,0.1; slightly toward front)
		
		// Test white ball hind foam
		vector<float> dung_beetle_rolling_pose = { 0.0,  -0.4,   0.3, // walkpose (-0.1, -0.5,  0.0)
				  	  	  	  	  	  	  	  	   0.0,  -0.4,   0.3,
												   0.2,  -0.1,   0.5, // walkpose (0.2, -0.5,  -0.1)
												   0.2,  -0.1,   0.5,
												   0.1,  -1.1,   0.25, // CF-joint: -1.3 (default)
												   0.1,  -1.1,   0.25,
												   0.15,  0.16,  0.1}; // (1st: 0.0,0.01,0.1; slightly toward front)
        // Trial Dynamic Rolling posture Backup Weights
		// vector<float> dung_beetle_rolling_pose = { 0.2, -0.5,    0.23, // walkpose (-0.1, -0.5,  0.0)
		// 		  	  	  	  	  	  	  	  	   0.2, -0.5,    0.23,
		// 										   0.1, -0.15 ,   0.2, // walkpose (0.2, -0.5,  -0.1)
		// 										   0.1, -0.15,    0.2,
		// 										   0.1, -1.3,    0.2 ,
		// 										   0.1, -1.3,    0.2,
		// 										  -0.05, -0.06,    0.1}; // (1st: 0.0,0.01,0.1; slightly toward front)

		//sitting pose
		vector<float> dung_beetle_sitting_pose = {-0.0, -1.5, -0.0,
										  -0.0, -1.5, -0.0,
										  0.0, -1.5, -0.0,
										  0.0, -1.5, -0.0,
										  1.3, -1.5,  0.0,
										  1.3, -1.5,  0.0,
										  -0.2885, -0.35, 0.1};

//		vector<float> joint_pos_hind_grab =   {-0.8, -0.3, -0.6}; // test hind leg grab & lie on ball
		// vector<float> joint_pos_hind_grab =   {-0.9, -0.2, -0.5};//-.04 // b test G basketball video paper
		// vector<float> joint_pos_hind_grab =   {-0.9, -0.2, -0.45};//-.04 // b test G basketball video paper review comment
		vector<float> joint_pos_hind_grab =   {-0.8,  0.0, -0.4};// new pose for basketball with soft (black sponge) hind legs
		// vector<float> joint_pos_hind_grab =   {-0.2,  0.1, -0.6}; // small paper box
		// vector<float> joint_pos_hind_grab =   {-0.8,  0.1, -0.5}; // big paper box paper
		// vector<float> joint_pos_hind_grab =   {-0.8,    0.0, -0.7}; // big paper box paper test
		// vector<float> joint_pos_hind_grab =   {-0.7,   -0.15, -0.8}; // big paper box paper test revision
		// vector<float> joint_pos_hind_grab =   {-0.8,   -0.1, -0.75}; // big paper box paper test revision slope 5 deg
		// vector<float> joint_pos_hind_grab =   {-0.8,   -0.2, -0.8}; // big paper box paper test revision slope 5 deg, black glove tip
		// vector<float> joint_pos_hind_grab =   {-0.8,   -0.2, -0.4}; // diamond paper box paper test revision
		// vector<float> joint_pos_hind_grab =   {-0.7,  0.1, -0.8}; // b test small fitness ball paper
		// vector<float> joint_pos_hind_grab =   {-0.7,  0.1, -0.75}; // b test small fitness ball paper grab friction test

		vector<float> joint_min = { -1.2860, -1.5561, -1.5561,
									-1.2860, -1.5561, -1.5561,
									-0.2118, -1.5561, -1.5561,
									-0.2118, -1.5561, -1.5561,
									-0.35, -1.8078, -1.5561,
									-0.35, -1.8078, -1.5561,
									-0.4588, -0.5187, -0.6722};

		vector<float> joint_max = { 0.1565, -0.0153,  1.5561,
									0.1565, -0.0153,  1.5561,
								    0.3714, -0.0153,  1.5561,
									0.3714, -0.0153,  1.5561,
								    1.5561, -0.5494,  1.5561,
									1.5561, -0.5494,  1.5561,
								    0.4128,  0.3714,  0.3714};


//      New update home position of Denmark dung beetle
//		std::vector<float> dung_beetle_pose = {-0.123, -0.356, 0.360,
//											   0.123, -0.356, 0.360,
//											   -0.367, -0.213, 0.363,
//					ssh						   0.367, -0.213, 0.363,
//											   -0.279, -0.02, 0.397,
//											   0.279, -0.02, 0.397,
//											   -0.190, -0.227, -0.098};


        vector<float> jointBias = {0.0, -1.0, 0.0,
                                   0.0, -1.0, 0.0,
                                   0.0, -1.0, 0.0,
                                   0.0, -1.0, 0.0,
                                   0.0, -1.0, 0.0,
                                   0.0, -1.0, 0.0,
                                   -0.2885, -0.35, 0.0};

        // walking joint bias for TH db_alpha
		/////// joint target V1 lower body
        std::vector<float> targetBC = { 30.0,   10.0,  -30.0};		// joint bias manual setting
        std::vector<float> targetCF = {-75.0,  -55.0,  -30.0};
        std::vector<float> targetFT = {-21.0,  -30.0,    9.0};
        float targetTA = 20;

		/////// joint target V2 raising body
//		std::vector<float> targetBC = { 15.0,   -5.0,  -20.0};		// joint bias manual setting
//		std::vector<float> targetCF = {-75.0,  -12.0,  -10.0};
//		std::vector<float> targetFT = { -5.0,   -5.0,   9.0 };
//		float targetTA = 0;
		////////////////////////////////////

		/////// joint target V3
//		std::vector<float> targetBC = { 6.0,    12.0,  12.0};		// joint bias manual setting
//		std::vector<float> targetCF = {-85.0,  -51.0,  -68.0};
//		std::vector<float> targetFT = { -12.0,   -12.0,   -12.0 };
//		float targetTA = -6.0;
		////////////////////////////////////

        // estimate walking joint bias for Denmark db_alpha by binggwong
        // uncomment these if using denmark version
        std::vector<float> targetBCr = {  30.0-4,   10.0+5,  -30.0-8};		// joint bias manual setting
        std::vector<float> targetBCl = { -30.0+4,  -10.0-5,   30.0+8};
//
//        std::vector<float> targetCF =  {-75.0+85,  -60.0+50,  -30.0+50};
//        std::vector<float> targetFT =  { 21.0+4,   30.0+4,   -9.0+4};
        /////////////////////////////////////////////////////////////////

        vector<float> home_velocity = {0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0};
        vector<float> home_torques = {0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0};
        vector<int> home_IDs = {11, 12, 13,
                                21, 22, 23,
                                31, 32, 33,
                                41, 42, 43,
                                51, 52, 53,
                                61, 62, 63,
                                71, 72, 73};
        // vector<string> home_names = {"id_101", "id_102", "id_103",
        //                              "id_104", "id_105", "id_106",
        //                              "id_107", "id_108", "id_109",
        //                              "id_110", "id_111", "id_112",
        //                              "id_113", "id_114", "id_115",
        //                              "id_116", "id_117", "id_118",
        //                              "id_119", "id_120", "id_121"};
		// New home_names for ROS2 db_beta_interface
        vector<string> home_names = {"motor_1", "motor_2", "motor_3",
                                     "motor_4", "motor_5", "motor_6",
                                     "motor_7", "motor_8", "motor_9",
                                     "motor_10", "motor_11", "motor_12",
                                     "motor_13", "motor_14", "motor_15",
                                     "motor_16", "motor_17", "motor_18",
                                     "motor_19", "motor_20", "motor_21"};
//        vector<string> one_leg_names = {"id_105", "id_106"};
    //    vector<string> one_leg_names = {"id_116", "id_117", "id_118"}; // Right Front Leg L1
        // vector<string> one_leg_names = {"id_110", "id_111", "id_112"}; // Right Middle Leg L2
    //    vector<string> one_leg_names = {"id_104", "id_105", "id_106"}; // Right Hind Leg L2
        vector<string> one_leg_names = {"id_101", "id_102", "id_103"}; // Left Hind Leg L2
        vector<string> couple_leg_names = {"id_113", "id_114", "id_115",
                							"id_116", "id_117", "id_118",}; // Right Middle Leg L2
        vector<string> one_leg_names_knock = {"id_116", "id_117", "id_118"};
        vector<string> one_leg_names_test = {"id_103"};
        vector<float> one_leg_names_test_pos = {0.0, -1, 0.0};

        // AMC data.
        vector<float> positions;
        vector<float> oneleg_positions;
        vector<float> couple_positions;
        vector<float> oneleg_knock;
        vector<float> torques;
        vector<float> feedback_positions = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};;
        vector<float> positions_target;
        vector<float> feedback_velocities;
        vector<float> feedback_current;
        vector<float> previous_positions;
        vector<float> previous_velocities;
        vector<float> cpg_signal;
        vector<float> pcpg_signal;
        vector<float> vrn_signal;
        vector<float> psn_signal;
        vector<float> legsearch_signal;
        vector<float> imu_signal;
        vector<float> ballDistanceControlData;
        vector<float> rollControlData;
        vector<float> pitchControlData;


        //vector<float> tau_ext;
        float TC_0_previous = 0;
        float TC_3_previous = 0;
        float CF_0_previous = 0;
        float CF_3_previous = 0;

       
        // CPG (this is intended just for data collection and understanding)
        float output_mnn_0 = 0;
        float output_mnn_1 = 0;
        float output_mnn_2 = 0;
        float output_mnn_3 = 0;
        float output_mnn_4 = 0;
        float output_mnn_5 = 0;
        float output_mnn_6 = 0;
        float output_mnn_7 = 0;
        float output_mnn_8 = 0;
        float output_mnn_9 = 0;

        // Constant head and backbone positions
        float m_71 = -0.3497;
        float m_72 = -0.367;
        float m_73 = -0.169;


        // Object declarations
        std::vector<modularController*> MC;
        dualIntegralLearner * learner;
        // modularController * CPG;
        // rclcpp::Node<realRos2Class>::SharedPtr realRos;
        std::shared_ptr<realRos2Class> realRos;
        AMC * complianceController;
        PDcontroller * pd_c;
        Joystick * joystick;

        // Save Data
        ofstream pos_feedback_csv;
        ofstream vel_feedback_csv;
        ofstream torque_csv;
        ofstream current_csv;
        ofstream pos_desired_csv;
        ofstream vel_desired_csv;
        ofstream vel_error_csv;
        ofstream current_feedback_csv;
        ofstream pos_error_csv;
        /// CPG signal
        ofstream cpg_signal_csv;
        ofstream pcpg_signal_csv;
        ofstream vrn_signal_csv;
        ofstream psn_signal_csv;
        /// leg searching signal
        ofstream legsearch_signal_csv;
        ofstream imu_csv;
        ofstream rollControl_csv;
        ofstream pitchControl_csv;
        ofstream ballDistanceControl_csv;

		// Leg state neuron
		float LS_self_w = 1.0;
		float LS_input_w = 1.0;
		float CF_tq_w = 0.9;
		std::vector<float> LS_act 	  = {0,0,0, 0,0,0};
		std::vector<float> LS_sense   = {0,0,0, 0,0,0};
		std::vector<float> LS_act_old = {0,0,0, 0,0,0};
		std::vector<float> LS_out 	  = {0,0,0, 0,0,0}; // state = 1(stand), state = -1 (swing)
		std::vector<float> LS_out_old = {0,0,0, 0,0,0};
        bool sw_activation = false;
        // Plot Data
//        std::vector<float> plotdata;

        ///// Turning System
    	float l = 1;
    	float r = 1;
        /////

        ///// backward leg lifting System
    	float bl = 1.0;
        /////

		// joint Torques
//		std::vector<float> sw_sens_rawbc;
//		std::vector<float> sw_sens_rawcf;
//		std::vector<float> FT_torque;
		std::vector<float> old_sw_sens_raw = {0,0,0, 0,0,0};
		std::vector<float> old_fc_sens_raw = {0,0,0, 0,0,0};
		std::vector<float> old_FT_torque = {0,0,0, 0,0,0};
		float input_w = 0.5;
		float self_w = 0.5;



		//// Foot contact system
        bool activate_fc_closeloop = false;
        std::vector<bool> stanceToSwing = {0,0,0,0,0,0};
        std::vector<bool> swingToStance = {0,0,0,0,0,0};

		std::vector<float> openlsignal = {0,0,0,0,0,0};
		std::vector<float> delay_openlsignal = {0,0,0,0,0,0};
		std::vector<float> old_openlsignal = {0,0,0, 0,0,0};
		std::vector<int> fcphase = {0,0,0, 0,0,0};
		std::vector<float> eff = {0,0,0, 0,0,0};
		float fc_threshold = 0.05;
		std::vector<float> fc_sens_raw = {0,0,0, 0,0,0};
		std::vector<float> fc_sens = {0,0,0, 0,0,0};
		std::vector<float> fc_error_st = {0,0,0,0,0,0};
		std::vector<float> fc_error_sw = {0,0,0,0,0,0};
		std::vector<float> oldfc_error_st = {0,0,0,0,0,0};
		std::vector<float> oldfc_error_sw = {0,0,0,0,0,0};

		std::vector<float> oldfc_error = {0,0,0,0,0,0};
		std::vector<float> max_fc_error = {0,0,0,0,0,0};
		float fc_selfw = 0.9; // locomotion: 0.9, rolling: 0.3
		float fc_err_fac = 0.4; // (0.4 for walking) // rolling: 0.1
		//////

		//// Swing obstacle avoidance system
		bool activate_sw_avoid = false;
		std::vector<float> sw_threshold = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2}; //fc : 200, 1000, 600

		std::vector<float> sw_sens_raw = {0,0,0, 0,0,0};

		std::vector<float> sw_sens = {0,0,0, 0,0,0};
		std::vector<float> sw_error = {0,0,0,0,0,0};
		std::vector<float> max_sw_error = {0,0,0,0,0,0};
		std::vector<float> oldsw_error = {0,0,0,0,0,0};
		std::vector<float> sw_err_fac = {0,0,0,0,0,0};
		std::vector<float> sw_error_mem = {0,0,0,0,0,0};
		float sw_selfw = 0.5;
		float sw_err_threshold = 0.4;
		// float sw_err_fac = 0.5;
        std::vector<float> activate_sw_leg_avoid = {0,0,0,0,0,0};
			///// avoid switching system
			bool activate_sw_switch = false;
			std::vector<float> sw_switch_threshold = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2}; // fc: 500, 400, 400
			float sw_switch_err_threshold = 0.3;
			std::vector<float> sw_switch_sens = {0,0,0,0,0,0};
			std::vector<float> sw_switch_err = {0,0,0,0,0,0};
			std::vector<float> sw_switch_spike = {0,0,0,0,0,0};
			std::vector<float> oldsw_switch_err = {0,0,0,0,0,0};
			std::vector<float> sw_switch_fac = {0,0,0,0,0,0};
	        std::vector<float> FT_torque = {0,0,0,0,0,0};

			//////
		//////		//////		//////



		//// Rolling system
		bool activate_rolling_grab = false;
		bool activate_walking_grab = false;
        bool activate_hingleg_oscillate = false;
		bool activate_roll_control = false;
		bool activate_ball_distance = false;
		bool activate_pitch_control = false;
		bool activate_pitch_learner = false;
        float roll;
        float pitch;
        float yaw;

        ////// Rolling system from Simulation
		// bool activate_rolling_grab = false;
		// Contact Manipulation
		std::vector<float> ball_push_sig 		= {0,0,0, 0,0,0};
		std::vector<float> old_ball_push_sig 	= {0,0,0, 0,0,0};
		std::vector<float> ground_push_sig 		= {0,0,0, 0,0,0};
		std::vector<float> old_ground_push_sig 	= {0,0,0, 0,0,0};
		float ball_push_input = 0.0;
		float ball_push_gain = 1.0;
		float ground_push_gain = 0.1;
		// Ball Control
		std::vector<float> joint_pos_ball_contact_sens = {0,0,0, 0,0,0};
		float represent_joint_pos_ball_contact_sens = 0.0;
		float robot_ball_contact = 0.0;
		float target_robot_ball_distance = -0.50;
		float robot_ball_distance_gain = 5.0;
		float e_ball = 0;

		// Pitch Control
		float pitch_freq_mod_fac = 0.01;
		float pitch_jointBias_mod_fac = 0.005;
		float pitch_bias = 40; // need calibration
		float pitch_freq_mod = 0;
		// float pitch;
		float pitch_error;
		float pitch_range = 5;
		float pitch_upper_bound = pitch_bias+pitch_range ; //sim: -5
		float pitch_lower_bound = pitch_bias-pitch_range ; // sim: -10
		// Pitch Control pitch bias learner
		float pitch_fc_miss_error = 0.0;
		float pitch_fc_miss_mod = 0.0;
		float pitch_fc_miss_gain = 0.01;
		// Integral Learner
		float pitchIntegralError = 0;
		float multi = 4.5;
		float Asp = 0.992;
		float Bsp = 0.0036*multi;
		float Bsi = 0.00018*multi;
		float dil_gamma = 0.0000001; // 0.00001 stable
		float slowLearner = 0;
		float slowLearnerP = 0;
		float dualLearner = 0;
		float controlOutput = 0;
		// Leg Amplitude Modulation
		//// from pitch Angle
		float pitch_front_amp_gain = 1.0;
		float pitch_back_amp_gain  = 1.0;
		float pitch_front_amp_gain_old = 1.0;
		float pitch_back_amp_gain_old  = 1.0;
		float pitch_front_adapt_bias = 1.0;
		float pitch_back_adapt_bias  = 1.0;
		//// from Roll Angle
		float roll_range  = 5.0;
		float roll_left_amp_gain  = 1.0;
		float roll_right_amp_gain = 1.0;
		// Roll Control
		std::vector<float> omega = {0,0,0};
		std::vector<float> angle = {0,0,0};
		std::vector<float> angle_lowpass = {0,0,0};
		std::vector<float> accel = {0,0,0};
		std::vector<float> old_omega = {0,0,0};
		std::vector<float> old_angle = {0,0,0};
		std::vector<float> old_accel = {0,0,0};
		float angle_input_w = 0.05;
		float angle_self_w = 0.95;
		float roll_bias = 0.0;
		float roll_cpg_mod_l = 0.0;
		float roll_cpg_mod_r = 0.0;
		float roll_mod_l_old = 0.0;
		float roll_mod_r_old = 0.0;

		float roll_leg_fac = 0.1;	

		float lowpass_self_w = 0.7;
		float lowpass_input_w = 0.3;


		// Stiffness Control
		float defalut_jointMaxTorque_frontL = 4.0; // 7.0 : 5 deg
		float jointMaxTorque_frontL = 0.0;
		std::vector<float> jointMaxTorqueArray = {0,0,0, 0,0,0};
		float pitch_jointStiff_mod_fac = 0.5; //defalt 0.5
		float pitch_jointStiff_mod = 0.0;
		float jointStiffLowerBound = 4.0;
		vector<float> trackingError  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

        // Info message
        void infoMessage();
        
};
#endif //DUNG_BEETLE_CONTROLLER_DUNGBEETLECONTROLLER_H


