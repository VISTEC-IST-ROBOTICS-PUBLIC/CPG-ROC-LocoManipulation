//
// Created by Binggwong on 15/07/2019.
// From Dungbeetle Controller by Carlos.
//

#ifndef DUNG_BEETLE_CONTROLLER_DUNGBEETLECONTROLLER_H
#define DUNG_BEETLE_CONTROLLER_DUNGBEETLECONTROLLER_H

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <cmath>
// core ros2 library
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "simRos2Class.h"
#include "utils/dln.h"
#include <fstream>
#include "joystick.h"
#include <termios.h>
#include <string>
#include "delayline.h"
#include "utils/AMC.h"

#include "./dbModularController.h"
#include "alphaMotorDefinition.h"

#define PI 3.14159265359

class modularController;
class simRos2Class;
class SO2CPG;
class AMC;
// class DelayN;

class dungBeetleController
{
	public:

		// General
		int countgg = 0;
		int sim_counter = 0;
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
		// Constructor
		dungBeetleController(int argc,char* argv[]);

		// Methods
		bool runController();
		void updateJointFeedback();
		void updateTorqueFeedback();
		void updateFootContactFeedback();
		void updateTestParam();
		void updateImuFeedback();
		void updateJoyAxesFeedback();
		void updateJoyButtonFeedback();
		void updateSimTime();

		// Variable
		float degtoRad = PI/180.0;	//conversion factor
		float fac = 1.0;
		float bb = 1.0;
		float c1, c1h, c2, c2h;
		float l = 1.0, r = 1.0;
		float fw = 1.0, bw = 1.0;
		float oldfw = 0.0;
		float x;
		float default_MI = 0.01;
		float default_front_MI = 0.15;
		float default_back_MI = 0.15;
		float front_MI = 0.15;
		float back_MI = 0.15;

		//neuroVis Param
		std::string neuron_name;
		std::vector<float> neuron_act;
		std::vector<float> c_matrix;
		std::vector<float> norm_gain;
		std::vector<float> norm_bias;
		int neuron_num;
		std::vector<int> IPSN_neuron = {23,24,25,26,27,28};
		std::vector<int> PSN_neuron = {29,32,35,38,41,44};
		std::vector<int> bc_neuron = {47,48,49,50,51,52};
		std::vector<int> cf_neuron = {53,54,55,56,57,58};
		std::vector<int> ft_neuron = {59,60,61,62,63,64};
		std::vector<int> fc_neuron = {65,66,67,68,69,70};
		std::vector<int> sw_neuron = {71,72,73,74,75,76};
		std::vector<float> bc_weight = {0.9,0.4,1.1, 0.9,0.4,1.1};
		std::vector<float> cf_weight = {0.6,-0.7,-0.7, 0.6,-0.7,-0.7};
		std::vector<float> ft_weight = {0.4,-0.7,-0.7, 0.4,-0.7,-0.7};

    private:

		// Leg state neuron
		float LS_self_w = 1;
		float LS_input_w = 1;
		float CF_tq_w = 0.5;
		std::vector<float> LS_act 	  = {0,0,0, 0,0,0};
		std::vector<float> LS_sense   = {0,0,0, 0,0,0};
		std::vector<float> LS_act_old = {0,0,0, 0,0,0};
		std::vector<float> LS_out 	  = {0,0,0, 0,0,0}; // state = 1(stand), state = -1 (swing)
		std::vector<float> LS_out_old 	  = {0,0,0, 0,0,0}; // state = 1(stand), state = -1 (swing)

		//neuroVis
		void setWeights(std::vector<int> i, std::vector<int> j, std::vector<float> array);
		void setWeights1on1(std::vector<int> i, std::vector<int> j, std::vector<float> array);
		void setNeuron_activity(std::vector<int> neurons, std::vector<float> activity);

        // Variables' Log file
		bool savedata = true;
		int savedatacounter = 0;
		int savelimit = 1000;
        std::vector<float> ballControlData;
        std::vector<float> rollControlData;
        std::vector<float> pitchControlData;
        std::vector<float> localLegData;
        ofstream fc_Loss;
        ofstream ballControlLog;
        ofstream rollControlLog;
        ofstream pitchControlLog;
        ofstream localLegLog;
		void saveVector(vector<float> vec, ofstream& file);

        //Delay
        std::vector<Delayline> tauCF;

        void Logdata_loss(double simtime, std::vector<float> fc_error);

		// Joy stick
	    double joySpeed = 0;
	    double joyTurn = 0;
	    bool useJoy = false;

	    // Keyboard input
	    bool useKeyboard = true; //Set true if you want to use keyboard inputs for walking, this allows for changing gaits
	    char keyboard_input = '3';
	    double inputCounter = 0;
	    char doMotion = 'w';
	    char getch();


        //Control Config
        double waiter = 200; // 5 SECONDS

        //Actuate position
		bool actuateJump();
		bool actuateGallop();
		bool actuateBackFlip();
		bool actuateStand();
    	int standcount = 0;


        // Numerical
        double rescale(double oldMax, double oldMin, double newMax, double newMin, double parameter);
        float relu(float input);
		float limitRangeFunction(float input, float lowerLimit, float upperLimit);
		float lowerLimitFunction(float input, float lowerLimit);
		float sigmoid(float x, float expfac, float a, float b);

		//global
		int objective;		//Walking forward = 1    //walking backward = 2		//Rolling = 3	  //Rolling on Inclined Floor = 4
		int inclineSensfb;
		bool walkfirst = true;

		//Joint Bias ////
		float biasBC0; float biasBC1; float biasBC2; float biasBC3; float biasBC4; float biasBC5;	// joint bias
		float biasCF0; float biasCF1; float biasCF2; float biasCF3; float biasCF4; float biasCF5;
		float biasFT0; float biasFT1; float biasFT2; float biasFT3; float biasFT4; float biasFT5;
		float biasTA = 30.0;

		////select one target joint option
		/////// joint target V1 lower body
		// std::vector<float> targetBC = { 30.0,   10.0,  -30.0};		// joint bias manual setting
		// std::vector<float> targetCF = {-70.0,  -65.0,  -30.0};
		// std::vector<float> targetFT = { -21.0,  -30.0,   -9.0};
		// float targetTA = -20;

		std::vector<float> targetBC = { 10.0,   10.0,  -5.0};		// new joint bias same as paper locomotion
		std::vector<float> targetCF = {-75.0,  -30.0,  -30.0};
		std::vector<float> targetFT = { 0.0,   -5.0,   -0.0};
		float targetTA = 15.0;


		// vector<float> dung_beetle_pose = {-0.1, -0.5,  0.0, // Hind
		// 								  0.2, -0.5,  -0.1, // Mid
		// 								  0.2, -1.3,  0.0, // Front
		// 								  -0.2885, -0.35, -0.1};

		/////// joint target V2 raising body
//		std::vector<float> targetBC = { 30.0,   -5.0,  -30.0};		// joint bias manual setting
//		std::vector<float> targetCF = {-70.0,  -35.0,  -25.0};
//		std::vector<float> targetFT = { 5.0,   10.0,   -9.0};
//		float targetTA = 0;
		////////////////////////////////////

		/////// joint bias for rolling behavior
		// std::vector<float> biasBCRoll = {40.0, 0.0, -20.0};
		// std::vector<float> biasCFRoll = {-10.0,  0.0,  -30.0};
		// std::vector<float> biasFTRoll = {90.0,  120.0,  110.0};


		std::vector<float> targetBC_Roll = {10,   10, 0}; // new rolling pose locomotion paper
		std::vector<float> targetCF_Roll = {-75, -35, -35};
		std::vector<float> targetFT_Roll = {10, -5, 0};
		float default_targetTA_Roll = 25;
		float targetTA_Roll = 25;

		std::vector<float> BC_Roll_fac = { 0.5 ,  0.4,  0.4 }; // new rolling pose locomotion paper, (base value : {0.5 ,  0.4,  0.4 })
		// std::vector<float> BC_Roll_fac = { 0.5 ,  0.2,  0.2 }; // new rolling pose test roll mid hind leg
		std::vector<float> CF_Roll_fac = { 0.55,  0.6,  0.55}; //, (base value : {0.55,  0.6,  0.55})
		std::vector<float> FT_Roll_fac = { 0.7 ,  0.7,  0.6};  //, (base value : {0.7 , -0.7, -0.6} is for walking)		 

		// vector<float> dung_beetle_ball_pose = {-0.1, -0.79, -0.79,
		// 								0.17, -0.96, -0.7,
		// 								0.1, -1.31, -0.35,
		// 								-0.5, -0.64, -0.1};


		//Joint Min ////
		std::vector<float> minBC = {   0, -90, -90 };	// joint Minimum angle
		std::vector<float> minCF = {   -90, -90, -90 };
		std::vector<float> minFT = {   0,   0,   0 };

		// Joint MAX ////
		std::vector<float> maxBC = {  90,   0,   0 };	// joint Maximun angle
		std::vector<float> maxCF = {  0,   0,   0 };
		std::vector<float> maxFT = { 180, 180, 180 };


		//Joint Range ////
		std::vector<float> rangeBC = { maxBC[0]-minBC[0], maxBC[1]-minBC[1], maxBC[2]-minBC[2]};	// joint range
		std::vector<float> rangeCF = { maxCF[0]-minCF[0], maxCF[1]-minCF[1], maxCF[2]-minCF[2]};
		std::vector<float> rangeFT = { maxFT[0]-minFT[0], maxFT[1]-minFT[1], maxFT[2]-minFT[2]};
		float rangeTA = 45;

		//Joint mid ////
		std::vector<float> midBC = { (maxBC[0]+minBC[0])/2, (maxBC[1]+minBC[1])/2, (maxBC[2]+minBC[2])/2};	// joint range
		std::vector<float> midCF = { (maxCF[0]+minCF[0])/2, (maxCF[1]+minCF[1])/2, (maxCF[2]+minCF[2])/2};
		std::vector<float> midFT = { (maxFT[0]+minFT[0])/2, (maxFT[1]+minFT[1])/2, (maxFT[2]+minFT[2])/2};


		std::vector<float> positions;	//save signal value
		std::vector<float> old_positions;	//save signal value

		// joint Torques
//		std::vector<float> sw_sens_rawbc;
//		std::vector<float> sw_sens_rawcf;
//		std::vector<float> FT_torque;
		std::vector<float> old_sw_sens_rawbc = {0,0,0, 0,0,0};
		std::vector<float> old_sw_sens_rawcf = {0,0,0, 0,0,0};
		std::vector<float> old_FT_torque = {0,0,0, 0,0,0};
		float input_w = 0.5;
		float self_w = 0.5;

		// standing system
		bool stand_extend = false;
		std::vector<float> error_st = {0,0,0,0,0,0};
		std::vector<float> olderror_st = {0,0,0,0,0,0};
		std::vector<float> sens_st = {0,0,0,0,0,0};
		float err_st_fac = 0.5;

		//// Foot contact system
        bool activate_fc_closeloop = true;
		float fc_threshold = 0.1;
		float fc_selfw = 0.95;
		float fc_err_fac = 0.2;
		std::vector<float> openlsignal = {0,0,0,0,0,0};
		std::vector<float> old_openlsignal = {0,0,0, 0,0,0};
		std::vector<float> eff = {0,0,0, 0,0,0};
		std::vector<int> fcphase = {0,0,0, 0,0,0};
		std::vector<float> fc_sens_raw = {0,0,0, 0,0,0};
		std::vector<float> fc_sens = {0,0,0, 0,0,0};
		std::vector<float> fc_error_st = {0,0,0,0,0,0};
		std::vector<float> oldfc_error_st = {0,0,0,0,0,0};
		std::vector<float> max_fc_error = {0,0,0,0,0,0};
        std::vector<bool> stanceToSwing = {0,0,0,0,0,0};
        std::vector<bool> swingToStance = {0,0,0,0,0,0};
		//////

		//// Swing obstacle avoidance system
		bool activate_sw_avoid = false;
		float sw_threshold = 0.5;
		std::vector<float> sw_sens_raw = {0,0,0, 0,0,0};
		std::vector<float> sw_sens_rawbc = {0,0,0, 0,0,0};
		std::vector<float> sw_sens_rawcf = {0,0,0, 0,0,0};
		std::vector<float> sw_sens = {0,0,0, 0,0,0};
		std::vector<float> sw_error = {0,0,0,0,0,0};
		std::vector<float> sw_error_mem = {0,0,0,0,0,0};
		std::vector<float> sw_err_fac = {0,0,0,0,0,0};
		std::vector<float> oldsw_error = {0,0,0,0,0,0};
		float sw_selfw = 0.9;
		float sw_err_threshold = 0.6;
		// float sw_err_fac = 0.7;
        std::vector<float> activate_sw_leg_avoid = {0,0,0,0,0,0};
		std::vector<float> max_sw_error = {0,0,0,0,0,0};
    	int reflex_delay;
        std::vector<Delayline> tau_reflexChain;

		///// avoid switching system
			bool activate_sw_switch = false;
			float sw_switch_threshold = 0.5;
			float sw_switch_err_threshold = 0.50;
			std::vector<float> sw_switch_sens = {0,0,0,0,0,0};
			std::vector<float> sw_switch_err = {0,0,0,0,0,0};
			std::vector<float> sw_switch_spike = {0,0,0,0,0,0};
			std::vector<float> oldsw_switch_err = {0,0,0,0,0,0};
			std::vector<float> sw_switch_fac = {1.0,1.0,1.0,1.0,1.0,1.0};;
			std::vector<float> FT_torque = {0,0,0,0,0,0};

		//////

		//// Swing Landing system
//		bool activate_sw_land = false;
		std::vector<float> fc_error_sw = {0,0,0,0,0,0};
		std::vector<float> oldfc_error_sw = {0,0,0,0,0,0};


		////// Rolling system
		bool activate_rolling_grab = false;
		bool activate_roll_control = true;
		bool activate_ball_distance = false;
		bool activate_pitch_control = true;
		float ball_push_input = 0.0;
		float ball_push_gain = 1.0;
		float ground_push_gain = 0.1;
		// Contact Manipulation
		std::vector<float> ball_push_sig 		= {0,0,0, 0,0,0};
		std::vector<float> old_ball_push_sig 	= {0,0,0, 0,0,0};
		std::vector<float> ground_push_sig 		= {0,0,0, 0,0,0};
		std::vector<float> old_ground_push_sig 	= {0,0,0, 0,0,0};
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
		float pitch_bias = 0;
		float pitch_freq_mod = 0;
		float pitch;
		float pitch_error;
		float pitch_upper_bound = -5;
		float pitch_lower_bound = -10;
		float pitch_range = 5;
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
		//// from Roll Angle
		float roll_range  = 5.0;
		float roll_left_amp_gain  = 1.0;
		float roll_right_amp_gain = 1.0;
		// Roll Control
		std::vector<float> omega = {0,0,0};
		std::vector<float> angle = {0,0,0};
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

		// Stiffness Control
		float defalut_jointMaxTorque_frontL = 4.0; // 7.0 : 5 deg
		float jointMaxTorque_frontL = 0.0;
		std::vector<float> jointMaxTorqueArray = {0,0,0, 0,0,0};
		float pitch_jointStiff_mod_fac = 0.0; //defalt 0.5
		float pitch_jointStiff_mod = 0.0;
		float jointStiffLowerBound = 4.0;
		vector<float> trackingError  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		// muscle model
		// AMC data.
        // vector<float> applied_torque;
        vector<float> current_positions  = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0};
        vector<float> current_torques    = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0};
        vector<float> desired_positions  = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0};
        // vector<float> current_velocities = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        // vector<float> desired_velocities = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        // // vector<float> desired_positions  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		// float MAX_TORQUE = 100;

		/// decouple CPG system
		float cpg_input_w_f = 0.00;
		float cpg_input_w_b = 0.00;
		std::vector<float> cpg_input_mod = {0,0,0,0,0,0};
		/// GRAB CPG system
		float localJointPos_bc;
		float localJointPos_cf;

		//// reservoir signal
		std::vector<float> esn_reflex;
		std::vector<float> esn_input;
		std::vector<float> esn_expect;

		float alpha = 0.002;

        // Object declarations
        // modularController * MC1;
        std::vector<modularController*> MC;
        std::shared_ptr<simRos2Class> simRos;
		SO2CPG * so2;
        AMC * complianceController;
        Joystick * joystick;
        // DelayN * tmp;
};

#endif //DUNG_BEETLE_CONTROLLER_DUNGBEETLECONTROLLER_H

