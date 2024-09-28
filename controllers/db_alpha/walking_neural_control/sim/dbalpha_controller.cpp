//
// Created by Binggwong on 15/07/19.
//

#include "dbalpha_controller.h"

// Main code:
dungBeetleController::dungBeetleController(int argc,char* argv[])
{
	// Joy stick initialize
    if(useJoy){
        joystick = new Joystick;
        if (!joystick->isFound()){
        	printf("Joystick open failed.\n");
        }
        else{
        	printf("JoyStick Connected.\n");
        }
    }
    positions.resize(19);				// set position vectors size to 18
    old_positions.resize(19);				// set position vectors size to 18

    //Delayline
    for(int i = 0; i<6; i++){
        Delayline tmp(40);
        tauCF.push_back(tmp);
    	modularController * MC_temp = nullptr;	// initiate modular controller
		MC_temp = new modularController(1, true);
    	MC.push_back(MC_temp);	// initiate modular controller
    }
	// MC[0]->pcpgc1->setThreshold(0.5)

	// SimRos Class Communication Initialize
	rclcpp::init(argc, argv);
	std::cout << "Run db_beta_controller" << std::endl;

	simRos = std::make_shared<simRos2Class>(argc, argv);

	// Muscle Model motor Control
    // complianceController = new AMC(0.05, 35.0, 5.0); //change to 5
    // complianceController->getParams();

	//--------------------------------
    // so2 = new SO2CPG(); 
	// //destabilize cpg to oscillate
	// so2->setOutput(0, 0.1);
	// so2->setOutput(1, 0.1);
	// so2->setActivity(0, 0.1);
	// so2->setActivity(1, 0.1);

	// //set cpg weights
	// so2->setWeight(0, 0, 1.4);
	// so2->setWeight(0, 1, 0.18 + 0.2); // MI = 0.01, 0.07 middle, 0.2 fast
	// so2->setWeight(1, 0,-0.18 - 0.2); // MI = 0.01
	// so2->setWeight(1, 1, 1.4);

	// //set bias
	// so2->setBias(0, 0.0); // cpg_bias = 0.0
	// so2->setBias(1, 0.0);
	//---------------------------------

    objective = 1; 		//Walking forward = 1	//walking backward = 2
						//Rolling = 3			//Rolling on Inclined Floor = 4
	// for(const int i : {0,3}){
	// 	// MC[i]->threshold1 = 0.6;
	// 	// MC[i]->threshold2 = 0.3;
	// 	// MC[i]->pcpgc1->uplimit = 45;
	// 	// MC[i]->pcpgc2->uplimit = 45;
	// }

	// Initialize Vrn Input Neuron
	for (int i=0; i<6; i++){
		MC[i]->setVrnInputNeurons(0, 1);
		// MC[0]->setVrnInputNeurons(1, 1);
		MC[i]->setVrnInputNeurons(2, 1);
		// MC[0]->setVrnInputNeurons(3, 1);
		// Initialize PSN Input Neuron
		MC[i]->setPsnInputNeurons(0,0);
		// MC[i]->cpg->setGamma(0.0);
	}
	// MC[0]->cpg->setGamma(0.0001);
	// MC[3]->cpg->setGamma(0.0001);
	// MC[0]->setPsnInputNeurons(1,0);
	// MC[0]->setPsnInputNeurons(2,0);
	// MC[0]->setPsnInputNeurons(3,0);
	// MC[0]->setPsnInputNeurons(4,0);
	// MC[0]->setPsnInputNeurons(5,0);

	// Step CPG to stable period
	std::vector<float> delayCPG;
	std::vector<float> CPGindex;
	delayCPG = {0, 37, 75, 113};
	CPGindex = {5, 4, 2, 1};
	for(int i=0; i<75; i++){
		for (int j=0; j<6; j=j+2){
			MC[j]->step();
		}
		// so2->step();
    }
	// for(int i=0; i<4; i++){
	// 	for(int j=0; j<delayCPG[i]; j++){
	// 		MC[CPGindex[i]]->step();
	// 	}
    // }


	// //update to steady state
    for(int n=0; n<200; n++){
		for (int j=0; j<6; j++){
			MC[j]->step();
		}    
	}

    /// Variable Log file collection
    // fc_Loss.open("/home/binggwong/experiments/dbalpha/fc_Loss.csv");
    // fc_Loss << "time\tlossleg0\tlossleg1\tlossleg2\tlossleg3\tlossleg4\tlossleg5" << "\n";
	if (objective == 3){
		targetBC_Roll[0] = testParameters[0];
		targetBC_Roll[1] = testParameters[1];
		targetBC_Roll[2] = testParameters[2];
		targetCF_Roll[0] = testParameters[3];
		targetCF_Roll[1] = testParameters[4];
		targetCF_Roll[2] = testParameters[5];
		targetFT_Roll[0] = testParameters[6];
		targetFT_Roll[1] = testParameters[7];
		targetFT_Roll[2] = testParameters[8];
		targetTA_Roll = testParameters[9];
		if (testParameters[10] > 0) activate_fc_closeloop = true; else activate_fc_closeloop = false;
		fc_threshold = testParameters[11];
		fc_selfw = testParameters[12];
		fc_err_fac = testParameters[13];

		cpg_input_w_f = testParameters[14];
		cpg_input_w_b = testParameters[15];

		pitch_bias = testParameters[16];
		pitch_freq_mod_fac = testParameters[17];
		pitch_jointBias_mod_fac = testParameters[18];

		roll_leg_fac = testParameters[20];
		
		cout << "SetUp Tune parameters" << endl;

	}
    reflex_delay = 2;
    for(int i = 0; i<6; i++){
    	Delayline tmp_delay(10);
    	tau_reflexChain.push_back(tmp_delay);
    }
    cout << "SetUp db_alpha Controller" << endl;

	// Log data to CSV files
	ballControlLog.open("/home/binggwong/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/Experiment_data/rolling/control/ballControlLog.csv");
	rollControlLog.open("/home/binggwong/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/Experiment_data/rolling/control/rollControlLog.csv");
	pitchControlLog.open("/home/binggwong/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/Experiment_data/rolling/control/pitchControlLog.csv");
	localLegLog.open("/home/binggwong/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/Experiment_data/rolling/control/localLegLog.csv");

	sim_counter = 0;
}
// Function that runs the controller and keeps it active
bool dungBeetleController::runController()
{
	cout << "#########################################" << endl;
	cout << "##### Controller is running step " << sim_counter << "###" << endl;
	float error;
	std::vector<float> plotdata;

	// Clear Log data array
	ballControlData.clear();
	rollControlData.clear();
	pitchControlData.clear();
	localLegData.clear();

    if (rclcpp::ok())
    {
		// Get all feedback information from simRos2
		updateJointFeedback();
		updateTorqueFeedback();
		updateFootContactFeedback();
		updateTestParam();
		updateImuFeedback();
		updateJoyAxesFeedback();
		updateJoyButtonFeedback();
		updateSimTime();

    	if(standcount < 1){
			// Use Joystick control
			if(useJoy)
			{
				printf("joy input \n");
				float axis1 = axes[1];
				if(axis1 > 0 ){
					objective = 1;
				}
				else if(axis1 < 0){
					objective = 2;
					cout << "ob2" << endl;
				}
				//Axes
	    		// cout << " axes [0] : "  << axes[0]
				// 	 << " axes [1] : "  << axes[1]
				// 	 << " axes [2] : "  << axes[2]
				// 	 << " axes [3] : "  << axes[3]<< endl;

	    		// //Buttons
	    		// cout << " button [0] : "  << buttons[0]
				// 	 << " button [1] : "  << buttons[1]
				// 	 << " button [2] : "  << buttons[2]
				// 	 << " button [3] : "  << buttons[3]<< endl;


			}
    	}


    	// Use Keyborad input control
        if(useKeyboard){
            if(inputCounter == 5){
                keyboard_input = getch();
                inputCounter = 0;
            }
            else{
            	inputCounter += 1;
            }

            // doMotion = keyboard_input;
            doMotion = 'R';
			// if(sim_counter > 100){
			// 	doMotion = 'R';
			// }
			// else{
			// 	doMotion = 'r';
			// }
            // doMotion = 'r';
			cout << "Keyboard input : " << doMotion << endl;
        }

//        doMotion = 'b';
        if(doMotion == 'w' or doMotion == 'b' or doMotion == 'q' or doMotion == 'e'
        		or doMotion == 'd' or doMotion == 'a' or doMotion == 'R' or doMotion == 'g' 
				or doMotion == 'u' or doMotion == 'o' or doMotion == 'r' or doMotion == 't'){

			for (int i=0; i<6; i++){
    	    	MC[i]->setMI(default_MI); // slow 0.0005
			}

            if(waiter >= 0)
            {
                error = 0;
                waiter--;
            }
            else
            {
                waiter = -10;
            }

            // check robot objective to initialize parameter
            // initialize Input neuron value, joint Bias
            if( objective == 1 || objective == 2)
            {
            	targetBC[0] = testParameters[0];
            	targetBC[1] = testParameters[1];
            	targetBC[2] = testParameters[2];
            	targetCF[0] = testParameters[3];
    			targetCF[1] = testParameters[4];
    			targetCF[2] = testParameters[5];
    			targetFT[0] = testParameters[6];
    			targetFT[1] = testParameters[7];
    			targetFT[2] = testParameters[8];
    			targetTA = testParameters[9];
    //			biasBC0 = targetBC[0];
    //			biasBC1 = targetBC[1];
    //			biasBC2 = targetBC[2];
    //			biasBC3 = targetBC[0];
    //			biasBC4 = targetBC[1];
    //			biasBC5 = targetBC[2];
    //
    //			biasCF0 = targetCF[0]-90.0;
    //			biasCF1 = targetCF[1]-90.0;
    //			biasCF2 = targetCF[2]-90.0;
    //			biasCF3 = targetCF[0]-90.0;
    //			biasCF4 = targetCF[1]-90.0;
    //			biasCF5 = targetCF[2]-90.0;
    //
    //			biasFT0 = targetFT[0]-90.0;
    //			biasFT1 = targetFT[1]-90.0;
    //			biasFT2 = targetFT[2]-90.0;
    //			biasFT3 = targetFT[0]-90.0;
    //			biasFT4 = targetFT[1]-90.0;
    //			biasFT5 = targetFT[2]-90.0;

    			// biasTA = biasTA/rangeTA;
				// cpg_input_w_f = testParameters[14];

            }
           	else if( objective == 3 || objective == 4){
            	targetBC_Roll[0] = testParameters[0];
            	targetBC_Roll[1] = testParameters[1];
            	targetBC_Roll[2] = testParameters[2];
            	targetCF_Roll[0] = testParameters[3];
    			targetCF_Roll[1] = testParameters[4];
    			targetCF_Roll[2] = testParameters[5];
    			targetFT_Roll[0] = testParameters[6];
    			targetFT_Roll[1] = testParameters[7];
    			targetFT_Roll[2] = testParameters[8];
    			targetTA_Roll = testParameters[9];

				if (testParameters[10] > 0) activate_fc_closeloop = true; else activate_fc_closeloop = false;
				fc_threshold = testParameters[11];
				fc_selfw = testParameters[12];
				fc_err_fac = testParameters[13];

				cpg_input_w_f = testParameters[14];
				cpg_input_w_b = testParameters[15];

				pitch_bias = testParameters[16];
				pitch_freq_mod_fac = testParameters[17];
				pitch_jointBias_mod_fac = testParameters[18];

				// if (testParameters[19] > 0) activate_roll_control = true; else activate_roll_control = false;
				// if (testParameters[19] > 0) activate_pitch_control = true; else activate_pitch_control = false;
				roll_leg_fac = testParameters[20];
				// default_front_MI = testParameters[21];
				// default_back_MI = testParameters[22];
				target_robot_ball_distance = testParameters[23];
				// cout << "front_MI : " << front_MI << endl;
				// cout << "back_MI : " << back_MI << endl;
				// cout << "ball_push_gain : " << ball_push_gain << endl;				

				////// Neuromechanical Control
				//// Set parameters from UI bar
				defalut_jointMaxTorque_frontL = testParameters[24];
				// float newB = testParameters[25];
				// float newBeta = testParameters[26];
				// complianceController->setA(newA);
				// complianceController->setB(newB);
				// complianceController->setBeta(newBeta);


				//////////////////////////////////////////////////////////////
				// IMU Sensory input signal preprocessing 
				//////////////////////////////////////////////////////////////
				omega = std::vector<float> (imuSensors.begin(), imuSensors.begin() + 3);
				angle = std::vector<float> (imuSensors.begin()+3, imuSensors.begin() + 6); // (roll, pitch, yaw) convention
				accel = std::vector<float> (imuSensors.begin()+6, imuSensors.end());
				// apply sensors filters
				for(int i = 0; i < 3; i++){
					omega[i] = (angle_input_w*omega[i] + angle_self_w*old_omega[i])*0.2;
					angle[i] = (angle_input_w*angle[i] + angle_self_w*old_angle[i]);
					// accel[i] = roll_angle_input_w*accel[i] + roll_angle_self_w*accel[i];
				}
				old_omega = omega;
				old_angle = angle;
				// old_accel = accel;
				

				//////////////////////////////////////////////////////////////
				// Pitch control 
				//////////////////////////////////////////////////////////////
				// pitch FC Modulation
				// Sensory input
				pitch = angle[1];
				pitch_error = pitch_bias - pitch;
				pitch_lower_bound = pitch_bias - pitch_range/2;
				pitch_upper_bound = pitch_bias + pitch_range/2;

				cout << "pitch angle : " << pitch << endl;

				if (activate_pitch_control){
					// if (fc_error_st[1] > 0.9 or fc_error_st[4] > 0.9){
					// 	pitch_fc_miss_error = 1.0;
					// 	pitch_fc_miss_mod += pitch_fc_miss_error*pitch_fc_miss_gain;
					// }
					// else{
					// 	pitch_fc_miss_error = 0.0;
					// 	pitch_fc_miss_mod *= 0.999;
					// }
					// pitch_bias = pitch_bias + pitch_fc_miss_mod;
					cout << "fc_error_st Left : " << fc_error_st[1] << endl;
					cout << "fc_error_st Right : " << fc_error_st[4] << endl;
					cout << "pitch_fc_miss_error : " << pitch_fc_miss_error << endl;
					cout << "pitch_fc_miss_mod : " << pitch_fc_miss_mod << endl;
					cout << "pitch_bias : " << pitch_bias << endl;

					// Integral Learner : frequency modulation from pitch error
					// if (pitch > pitch_upper_bound or pitch < pitch_lower_bound){
					// 	cout << "Activate Integral Learner : " << endl;
					// 	pitchIntegralError += pitch_error;
					// 	slowLearnerP = (Asp*slowLearnerP) + (Bsp*pitch_error) + (Bsi*pitchIntegralError);
					// 	dualLearner = slowLearnerP*dil_gamma;
					// 	default_back_MI = default_back_MI - dualLearner;
					// 	default_back_MI = limitRangeFunction(default_back_MI, 0.00, 0.2);
					// }
					// else{
					// 	cout << "Integral Learner Idle" << endl;
					// 	pitchIntegralError = 0;
					// 	slowLearnerP *= 0.99;
					// }

					// cout << "pitchIntegralError : " << pitchIntegralError << endl;
					// cout << "slowLearnerP : " << slowLearnerP << endl;
					// cout << "dualLearner : " << dualLearner << endl;
					// cout << "default_back_MI : " << default_back_MI << endl;
					// low limit
					// // Original Dual Learner ////////////
					// // Dual-Learner v.4
					// integralError += error;
					// fastLearnerP = (Afp*fastLearner) + (Bfp*error) + (Bfi*integralError);
					// slowLearnerP = (Asp*slowLearner) + (Bsp*error) + (Bsi*integralError);
					// dualLearner = fastLearnerP + slowLearnerP;
					// // Calculate controlOutput
					// controlOutput = baseState - dualLearner;
					// //////////////////////////////////////

					// Pitch Boundary control : Leg frequency modulation from pitch error
					// pitch_freq_mod 	= pitch_error * pitch_freq_mod_fac;

					///////////////
					// Modulated MI ////////////
					// if(pitch > pitch_upper_bound){
					// 	front_MI = 0.0;
					// }
					// else{
					// 	front_MI = default_front_MI;
					// 	// front_MI = default_front_MI + pitch_freq_mod;
					// }				
					// if(pitch < pitch_lower_bound){
					// 	back_MI = 0.0;
					// }
					// else{
					// 	// back_MI  = default_back_MI  - pitch_freq_mod;
					// 	back_MI  = default_back_MI;
					// }
					////////////////////////////////
					front_MI = default_front_MI;
					back_MI = default_back_MI;

					cout << "front MI : " << front_MI << endl;
					cout << "back  MI : " << back_MI << endl;
					// if (back_MI > 0.02){
					// 	back_MI = 0.02;
					// }
					// else if (back_MI < -0.01){
					// 	back_MI = -0.01;
					// }
					// cout << "pitch_freq_mod : " << pitch_freq_mod << endl;

					for(const int i : {1,2, 4,5}){
						MC[i]->setMI(back_MI);
					}
					for(const int i : {0,3}){
						MC[i]->setMI(front_MI);
						// cout << "MC" << i << " MI : " << MC[i]->getMI() << endl;
					}

					// pitch orientation to modulate joint bias of middle and hind leg
					// targetBC_Roll[2] -= (pitch - pitch_bias) *0.2;
					// targetBC_Roll[1] -= (pitch - pitch_bias) *0.2;
					// cout << "targetBC_Roll" << targetBC_Roll[2] << endl;

					// pitch orientation to modulate backbone joint
					// targetTA_Roll -= (pitch - pitch_bias);
					// cout << "targetTA_Roll : " << targetTA_Roll << endl;

					// Leg Amplitude Modulation
					pitch_front_amp_gain = (0.5*pitch_error)/pitch_range + 1.0;
					pitch_front_amp_gain = limitRangeFunction(pitch_front_amp_gain, 0.0, 1);
					
					pitch_back_amp_gain = (-0.5*pitch_error)/pitch_range + 1.0;
					pitch_back_amp_gain = limitRangeFunction(pitch_back_amp_gain, 0.0, 1);

					// low pass the amplitude gain
					pitch_front_amp_gain = 0.5*pitch_front_amp_gain + 0.5*pitch_front_amp_gain_old;
					pitch_back_amp_gain  = 0.5*pitch_back_amp_gain  + 0.5*pitch_back_amp_gain_old;
					pitch_front_amp_gain_old = pitch_front_amp_gain;
					pitch_back_amp_gain_old  = pitch_back_amp_gain;

					// roll_left_amp_gain  = (2.0*angle[0])/roll_range + 1.0;
					// roll_left_amp_gain = limitRangeFunction(roll_left_amp_gain, -1.0, 2.0);

					// roll_right_amp_gain  = -(2.0*angle[0])/roll_range  + 1.0;
					// roll_right_amp_gain = limitRangeFunction(roll_right_amp_gain, -1.0, 2.0);

					// pitch_jointStiff_mod_fac = 0.0;				
					// Leg Stiffness modulation from pitch error
					// pitch_jointStiff_mod = pitch_error * pitch_jointStiff_mod_fac;
					// jointMaxTorque_frontL = defalut_jointMaxTorque_frontL + pitch_jointStiff_mod;
					// if (jointMaxTorque_frontL < jointStiffLowerBound){
					// 	jointMaxTorque_frontL = jointStiffLowerBound;
					// }
					// cout << "jointMaxTorque : " << jointMaxTorque_frontL << endl;

				}
				else{
					pitch_front_amp_gain = 1.0;
					pitch_back_amp_gain = 1.0;

					roll_left_amp_gain = 1.0;
					roll_right_amp_gain = 1.0;
					
					pitch_jointStiff_mod_fac = 0.0;
					// Leg Stiffness modulation from pitch error
					// pitch_jointStiff_mod = pitch_error * pitch_jointStiff_mod_fac;
					// jointMaxTorque_frontL = defalut_jointMaxTorque_frontL + pitch_jointStiff_mod;
					// if (jointMaxTorque_frontL < jointStiffLowerBound){
					// 	jointMaxTorque_frontL = jointStiffLowerBound;
					// }
					// cout << "jointMaxTorque : " << jointMaxTorque_frontL << endl;

					// Fix MI /////
					front_MI = default_front_MI;
					back_MI = default_back_MI;
					for(const int i : {1,2, 4,5}){
						MC[i]->setMI(back_MI);
					}
					for(const int i : {0,3}){
						MC[i]->setMI(front_MI);
					}
				}

				cout << "//// Leg Amplitude Modulation ////" << endl;
				cout << "pitch_front_amp_gain : " << pitch_front_amp_gain << endl;
				cout << "pitch_back_amp_gain : " << pitch_back_amp_gain << endl;

				cout << "roll_left_amp_gain : " << roll_left_amp_gain << endl;
				cout << "roll_right_amp_gain : " << roll_right_amp_gain << endl;
				cout << "/////////////////////////////////" << endl;
				//////////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////////
				// Roll Control
				//////////////////////////////////////////////////////////////

				// roll omega, angle to front leg CPG modulation
				// roll_cpg_mod_l = sigmoid(relu(-omega[0]), -0.05, 100, -100);
				// roll_cpg_mod_r = sigmoid(relu( omega[0]), -0.05, 100, -100);
				// roll_cpg_mod_l = relu(-omega[0]);
				// roll_cpg_mod_r = relu( omega[0]);
				// trim roll angle at 10 to 20 degree

				// roll_cpg_mod_l = 0.5*relu(-angle[0]-10) + 0.5*roll_mod_l_old;
				// roll_cpg_mod_r = 0.5*relu( angle[0]-10) + 0.5*roll_mod_r_old;
				roll_cpg_mod_l = relu(-angle[0]-10);
				roll_cpg_mod_r = relu( angle[0]-10);
				roll_mod_l_old = roll_cpg_mod_l;
				roll_mod_r_old = roll_cpg_mod_r;
				// roll_cpg_mod_l = sigmoid(omega[0],  1,  5, 5);
				// roll_cpg_mod_r = sigmoid(omega[0], -1, -5, 5);
				// roll_cpg_mod_l = 0.0;
				// roll_cpg_mod_r = 0.0;
				cout << "roll_cpg_mod Left  : " << roll_cpg_mod_l << endl;
				cout << "roll_cpg_mod Right : " << roll_cpg_mod_r << endl;

				//////////////////////////////////////////////////////////////


           	}

			// set PSN parameter for forward walking
			for (int i=0; i<6; i++){
				MC[i]->setPsnInputNeurons(0,0);
			}
			// MC[0]->setPsnInputNeurons(1,0);
			// MC[0]->setPsnInputNeurons(2,0);
			// MC[0]->setPsnInputNeurons(3,0);
			// MC[0]->setPsnInputNeurons(4,0);
			// MC[0]->setPsnInputNeurons(5,0);

			// MC[0]->setPsnInputNeurons(0,0);
			// MC[0]->setPsnInputNeurons(1,0);
			// MC[0]->setPsnInputNeurons(2,0);
			// MC[0]->setPsnInputNeurons(3,0);
			// MC[0]->setPsnInputNeurons(4,0);
			// MC[0]->setPsnInputNeurons(5,0);

            // float c1  = MC[0]->getpmnOutput(0);
            // float c1h = MC[0]->getpmnOutput(1);
//            float cpg = MC[0]->getCpgOutput(0);

            // float c2  = MC[0]->getpmnOutput(0);
    		// float c2h = MC[0]->getpmnOutput(1);

    		float lift0 = MC[0]->getpmnOutput(0);
    		float lift1 = MC[1]->getpmnOutput(0);
    		float lift2 = MC[2]->getpmnOutput(0);
    		float lift3 = MC[3]->getpmnOutput(0);
    		float lift4 = MC[4]->getpmnOutput(0);
    		float lift5 = MC[5]->getpmnOutput(0);

    		float down0 = MC[0]->getpmnOutput(1);
    		float down1 = MC[1]->getpmnOutput(1);
    		float down2 = MC[2]->getpmnOutput(1);
    		float down3 = MC[3]->getpmnOutput(1);
    		float down4 = MC[4]->getpmnOutput(1);
    		float down5 = MC[5]->getpmnOutput(1);

			// cout << "CPG Output  : " << MC[1]->getpmnOutput(1) << endl;
			// cout << "PCPG Output : " << MC[2]->getpmnOutput(1) << endl;


//            float c1 = MC[0]->getpmnOutput(2);
//            float c1h = -c1;


//			tauCF[0].Write(c1);
//			tauCF[1].Write(c1h);
//			c2 = tauCF[0].Read(15);
//			c2h = tauCF[1].Read(15);


//    		plotdata.push_back(c1);

//    		tauCF[0].Write(c2);
//    		tauCF[1].Write(c2h);
//    		c2 = tauCF[0].Read(6);
//    		c2h = tauCF[1].Read(6);

    		fac = 1.0;
    		bb = 1.0;
    		fw = 1.0;
    		bw = 1.0;
			r = 1.0;
			l = 1.0;
			if (objective == 1) {
    			fac = 1.0;
			}

    		// Selecting behavior
    		int gallop = buttons[0];
    		int backward = buttons[1];
    		int jump = buttons[2];
    		int backflip = buttons[3];

    		///////// Read open loop Efference copy ////////
    		// float eff_vrn3 = MC[0]->getVrnOutput1(6);
    		// float eff_vrn4 = MC[0]->getVrnOutput3(6);
			
			// openlsignal = {eff_vrn4,eff_vrn3,eff_vrn4,eff_vrn3,eff_vrn4,eff_vrn3};
			
			for (int i=0; i<6; i++){
				eff[i] = MC[i]->getVrnOutput1(6);
				tauCF[i].Write(eff[i]);
				openlsignal[i] = tauCF[i].Read(3);
			}
    		for (int i = 0; i < 6; i++){
    			if (openlsignal[i] > old_openlsignal[i]){
    				fcphase[i] = 0;
    			}
    			else{
    				fcphase[i] = 1;
    			}
    		}

			// old_openlsignal = {eff_vrn4,eff_vrn3,eff_vrn4,eff_vrn3,eff_vrn4,eff_vrn3};
			for (int i=0; i<6; i++){
				old_openlsignal[i] = openlsignal[i];
			}
    		///////// Read open loop Efference copy ////////


			/////////  Read Sensor data ///////////////////
			sw_sens_rawbc = std::vector<float> (jointTorques.begin(), jointTorques.begin() + 6);
    		sw_sens_rawcf = std::vector<float> (jointTorques.begin() + 6, jointTorques.begin() + 12);
    		FT_torque = std::vector<float> (jointTorques.begin() + 12, jointTorques.end());

    		// for(int i = 0; i < 6; i++){
    		// 	sw_sens_rawbc[i] = input_w*sw_sens_rawbc[i] + self_w*old_sw_sens_rawbc[i];
			// 	sw_sens_rawcf[i] = input_w*sw_sens_rawcf[i] + self_w*old_sw_sens_rawcf[i];
			// 	FT_torque[i] = input_w*FT_torque[i] + self_w*old_FT_torque[i];
			// }
    		// old_sw_sens_rawbc = sw_sens_rawbc;
    		// old_sw_sens_rawcf = sw_sens_rawcf;
    		// old_FT_torque = FT_torque;

    		fc_sens_raw = sw_sens_rawcf; // forceSensors;
			sw_sens_raw = sw_sens_rawbc;
			/////////  Read Sensor data ///////////////////

			//////////////////////////////////////////////////////////////
			// Ball Control (Robot Ball distance Control) 
			//////////////////////////////////////////////////////////////
			current_positions = jointPositions;
			// target_robot_ball_distance = -0.40;
			represent_joint_pos_ball_contact_sens = 0.7*0.25*(joint_pos_ball_contact_sens[1]+joint_pos_ball_contact_sens[2]
														+joint_pos_ball_contact_sens[4]+joint_pos_ball_contact_sens[5])
														+0.3*represent_joint_pos_ball_contact_sens;
			e_ball = target_robot_ball_distance - represent_joint_pos_ball_contact_sens;

			ball_push_input = e_ball*robot_ball_distance_gain;
			ball_push_input = tanh(ball_push_input);
			// ball_push_gain = limitRangeFunction(ball_push_gain, -1.0, 1.0);
			if (activate_ball_distance){
				ball_push_gain = 1.0;
			}
			else{
				ball_push_gain = 0.0;
			}
			
			cout << "#####################################" << endl;
			cout << "---- Robot Ball distance Control ----" << endl;
			cout << "represent_joint_pos_ball_contact_sens : " << represent_joint_pos_ball_contact_sens << endl;
			cout << "joint_pos_ball_contact_sens[1] : " << joint_pos_ball_contact_sens[1] << endl;
			cout << "joint_pos_ball_contact_sens[2] : " << joint_pos_ball_contact_sens[2] << endl;
			cout << "ball_push_input : " << ball_push_input << endl;
			cout << "ball_push_gain : " << ball_push_gain << endl;
			cout << "#####################################" << endl;
			//////////////////////////////////////////////////////////////


			//// Initial StanceToSwing and swingToStance setup/////////////
			// run once when the leg switch leg state (Stance, Swing)
			for(int i = 0; i < 6; i++){
				if(fcphase[i] == 0){
		//			cout << "Leg " << i << " swing" << endl;
					if(stanceToSwing[i] == false){
						// LS_act[i] = 0;
						// LS_out[i] = 0;
						// LS_out_old[i] = 0;

						fc_sens[i] = 0;
						// tilt_err[i] = 0;
						fc_error_st[i] = 0;
						oldfc_error_st[i] = 0;

						swingToStance[i] = false;
		//					cout << " stanceToSwing" << endl;
						stanceToSwing[i] = true;

						old_ball_push_sig[i] = 0;
						ball_push_sig[i] = 0;

						old_ground_push_sig[i] = 0;
						ground_push_sig[i] = 0;

						continue;
					}
				}
				else if(fcphase[i] == 1){
		//			cout << "Leg " << i << " stance" << endl;
					if(swingToStance[i] == false){
						sw_sens[i] = 0;
						sw_switch_sens[i] = 0;

						sw_error[i] = 0;
						sw_switch_err[i] = 0;

						oldsw_error[i] = 0;
						oldsw_switch_err[i] = 0;

						activate_sw_leg_avoid[i] = 0;
						sw_switch_spike[i] = 0;

						stanceToSwing[i] = false;
		//					cout << " swingToStance" << endl;
						swingToStance[i] = true;
		//				sw_switch_fac[i] = 0;
						LS_act[i] = 0;
						LS_out[i] = 0;
						LS_out_old[i] = 0;

		//				if(max_sw_error[i] > 0.4){
		//					cout << "Swing error --> FC_error" << endl;
		//					fc_error_st[i] = -max_sw_error[i]*0.5;
		//					oldfc_error_st[i] = -oldsw_error[i]*0.5;
		//				}
		//				else{
						// fc_error_st[i] = max_fc_error[i];
						// oldfc_error_st[i] = max_fc_error[i];
		//				}
						continue;
					}
				}

		//	}
			/////////StanceToSwing and swingToStance setup/////////


			////////   Stance Phase Foot Extend System ////////////////
			//// threshold foot contact sensor signal /////
		//	for(int i = 0; i < 6; i++){
				if (-fc_sens_raw[i] > fc_threshold){
					fc_sens[i] = 1;
				}
				else {
					fc_sens[i] = 0;
				}

				if(fcphase[i] == 1){

					if(true or activate_fc_closeloop or activate_rolling_grab){
						//// calculate error ////
						if (fc_sens[i] == 0){
							fc_error_st[i] = fc_selfw * oldfc_error_st[i] +
								(1.0-fc_selfw) * (fcphase[i] - fc_sens[i]);
							if(doMotion=='R'){
								// ball_push_sig[i] -= 0.01;
								// ground_push_sig[i] -= 0.01;
								ball_push_sig[i] *= 0.98;
								ground_push_sig[i] *= 0.98;
								ball_push_sig[i] = lowerLimitFunction(ball_push_sig[i], 0.0);
								ground_push_sig[i] = lowerLimitFunction(ball_push_sig[i], 0.0);
							}

						}
						else if (fc_sens[i] == 1)
						{
							fc_error_st[i] -= 0.01;
							fc_error_st[i] = lowerLimitFunction(fc_error_st[i], 0.0);
							if(doMotion=='R'){
								// ball_push_sig[i] = tanh(0.05 * fc_sens[i]+ 0.95 * old_ball_push_sig[i]);
								// ground_push_sig[i] = tanh(0.05 * fc_sens[i]+ 0.95 * old_ground_push_sig[i]);
								ball_push_sig[i] = tanh(0.1 * ball_push_input+ 0.9 * old_ball_push_sig[i]);
								ground_push_sig[i] = tanh(0.05 * ball_push_input+ 0.95 * old_ground_push_sig[i]);
							}
							joint_pos_ball_contact_sens[i] = current_positions[i+6];
							// cout << "Collect joint_pos_ball_contact_sens Data !!!" << endl;
							// cout << "current_positions " << i << " : " << current_positions[i+6] << endl;

						}
					
						

						//// Memory term for max value of leg Extend with decay ////////
						//// for extend in the begin of stance phase /////
						if(max_fc_error[i] < fc_error_st[i]){
							max_fc_error[i] = fc_error_st[i];
						}
						max_fc_error[i] *= 0.95;
					}
					//////// tiliting stance Reflex //////////
					// float tilt_angle = angle[0]*degtoRad;
					// tilt_err[i] = tanh(0.1*tilt_angle + 0.9*tilt_err[i]);
				}
		//	}
		//	oldfc_error_st = fc_error_st;
			////////   Stance Phase Foot Extend System ////////////////


			////////   Swing Phase Foot Retract System for Obstacle ////////////////
		//	for(int i = 0; i < 6; i++){

				if(fcphase[i] == 0){
		//			cout << "Leg " << i << " swing" << endl;
					/////// Threshold BC joint Torque ///////

					if (-sw_sens_raw[i] > sw_threshold){
						sw_sens[i] = 1;
					}
					else {
						sw_sens[i] = 0;
					}

					///////// Leg state neuron ////////////////////
					LS_act[i] = LS_input_w * (1-fc_sens[i]) + (LS_self_w * LS_out_old[i]);
					LS_out[i] = tanh(LS_act[i]);
					LS_out_old = LS_out;
					///////////////////////////////////////////////
					if (LS_out[i] > 0.9){

						if(activate_sw_avoid){

							//// calculate error
							sw_error[i] = sw_selfw * oldsw_error[i] +
									(1.0-sw_selfw) * sw_sens[i];

							//// Memory term for max value of leg Extend with decay ////////
							//// for extend in the begin of stance phase /////
							// if(sw_error[i] > sw_error_mem[i]){
							// 	sw_error_mem[i] = sw_error[i];
							// }
							sw_error_mem[i] += sw_error[i];
							//// activate spiking error when sw_error exceed threshold
							// if(sw_error[i] > sw_err_threshold and activate_sw_leg_avoid[i] == 0){
								// activate_sw_leg_avoid[i] = 1;
				//						fc_error_sw[i] = -10.0;
				//						oldfc_error_sw[i] = -9.0;
		//						sw_error[i] = 1.0;
			//					cout << "activate Leg" << i << endl;
							// }
			//				else if(sw_error[i] < 1.0 and activate_sw_leg_avoid[i] == 1){
			//					activate_sw_leg_avoid[i] = 0;
			////					sw_error[i] = 0;
			////					oldsw_error[i] = 0;
			//	////						fc_error[i] = -5.0;
			//				}
						}
						sw_error_mem[i] *= 0.99;
					////////////////////////////////////////////////////////////////

					///// avoid switching system
					/////// Threshold FT joint Torque ///////
						// if(activate_sw_switch == true ){
						// 	if (FT_torque[i] > sw_switch_threshold){
						// 		sw_switch_sens[i] = 1;
						// 	}
						// 	else {
						// 		sw_switch_sens[i] = 0;
						// 	}
						// 	//// calculate error
						// 	sw_switch_err[i] = sw_selfw * oldsw_switch_err[i] +
						// 			(1.0-sw_selfw) * sw_switch_sens[i];

						// 	//////
						// 	if(sw_switch_err[i] > sw_switch_err_threshold){
						// 		sw_switch_spike[i] = 2.0;
						// 		cout << "FT Movement Switch Leg" << i << endl;
						// 	}
						// }
					}
					//////////////////////////////////////////
				}
			}
			oldfc_error_st = fc_error_st;
			oldsw_switch_err = sw_switch_err;
			oldsw_error = sw_error;
			sw_err_fac = sw_error_mem;
			old_ball_push_sig = ball_push_sig;
			old_ground_push_sig = ground_push_sig;
			////////   Swing Phase Foot Retract System for Obstacle ////////////////

			//test swing switch system
//			plotdata.push_back(FT_torque[0]);
//			plotdata.push_back(sw_switch_sens[0]);
//			plotdata.push_back(sw_switch_err[0]);
//			plotdata.push_back(sw_switch_err_threshold);
//			plotdata.push_back(MC[0]->getVrnOutput(6));
//			plotdata.push_back(sw_sens_rawbc[0]);
//			plotdata.push_back(sw_sens_rawcf[0]);
//			plotdata.push_back(sw_threshold);
//			plotdata.push_back(sw_error[1]);
//			plotdata.push_back(sw_switch_err[1]);
//			plotdata.push_back(fcphase[1]);
//			plotdata.push_back(swingToStance[1]);


    		// Turning direction factor
    		float axis3 = axes[3];
    		if (axis3 == 0 ){
    			l = 1;
    			r = 1;
    		}
    		else if (axis3 > 0){
    			l = 1 - axis3;
    			r = 1;
    		}
    		else if (axis3 < 0){
    			l = 1;
    			r = 1 + axis3;
    		}

    		if (backward or doMotion == 'b' or doMotion == 'g')
    		{
				for (int i=0; i<6; i++){
					MC[i]->setPsnInputNeurons(0,1);
				}
    		}
    		else if (doMotion == 'q'){
				cout << "Left Turn" << endl;
    			MC[0]->setPsnInputNeurons(0,1);
				for (int i=0; i<3; i++){
					MC[i]->setPsnInputNeurons(0,1);
				}
    		}
    		else if (doMotion == 'e'){
//    			r = -1.0;
				cout << "Right Turn" << endl;
				for (int i=3; i<6; i++){
					MC[i]->setPsnInputNeurons(0,1);
				}
			}
    		else if (doMotion == 'a'){
				cout << "Left Curve" << endl;
    			l = 0.5;
    		}
    		else if (doMotion == 'd'){
				cout << "Right Curve" << endl;
    			r = 0.5;
    		}


    		//Walking speed
    		joySpeed = axes[1];
    	    if(joySpeed > 0.2){
    	    	MC[0]->setMI(joySpeed/2);
    	    }

    		// Backward
    		float bk = -1;

//            c1  = 0;
//            c1h = 0;
//
//
//            c2  = 0;
//    		c2h = 0;

    		if(gallop){
    			actuateGallop();
    		}
    //		else if(backward){
    //			actuateBackward();
    //		}
    		else if(jump){
    			actuateJump();
    		}
    		else if(backflip){
    			actuateBackFlip();
    		}
			else if(doMotion == 'r'){
    			actuateStand();
			}
    		else
    		{
    			// Assign joint position
    			// Sending joint angle to V-REP in Radian unit by adding *degtoRad*jointRangeXX;
    			// Body-Coxa Joint Position
    			positions.at(BC0) =  l * fac * lift0 * fw * 0.5 	   + ( targetBC[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
    			positions.at(BC1) =  l * fac * down1  * bw * 0.4 	   + ( targetBC[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
    			positions.at(BC2) =  l * fac * down2 * bw * 0.4 	   + ( targetBC[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
    			positions.at(BC3) =  r * fac * lift3  * fw * 0.5 	   + ( targetBC[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
    			positions.at(BC4) =  r * fac * down4 * bw * 0.4 	   + ( targetBC[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
    			positions.at(BC5) =  r * fac * down5  * bw * 0.4 	   + ( targetBC[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

    			// Coxa-Femur Joint Position
    			positions.at(CF0) =  l * fac * down0 * bw * 0.55     + ( targetCF[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
    			positions.at(CF1) =  l *-fac * lift1  * bw * 0.5      + ( targetCF[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
    			positions.at(CF2) =  l *-fac * lift2 * bw * 0.55     + ( targetCF[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
    			positions.at(CF3) =  r * fac * down3  * bw * 0.55 	  + ( targetCF[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
    			positions.at(CF4) =  r *-fac * lift4 * bw * 0.5      + ( targetCF[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
    			positions.at(CF5) =  r *-fac * lift5  * bw * 0.55     + ( targetCF[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

    			// Femur-Tibia Joint Position
    			positions.at(FT0) =  l *fac * down0 * 0.7  + ( targetFT[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
    			positions.at(FT1) =  l *fac * lift1  * -0.6       + ( targetFT[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
    			positions.at(FT2) =  l *fac * lift2 * -0.6       + ( targetFT[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
    			positions.at(FT3) =  r *fac * down3  * 0.7  + ( targetFT[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
    			positions.at(FT4) =  r *fac * lift4 * -0.6       + ( targetFT[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
    			positions.at(FT5) =  r *fac * lift5  * -0.6       + ( targetFT[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
    			
				positions.at(TA) =  targetTA*degtoRad;
    			// cout << "TA position : " << positions.at(BC0) << endl;
    			// cout << "TA position : " << targetTA << endl;
    			// cout << "TA position : " << degtoRad << endl;
    			// cout << "TA position : " << positions.at(TA) << endl;

    			if (doMotion == 'q'){
    				cout << doMotion << endl;

//    				float temp = c1;
//    				c1 = c2;
//    				c2 = temp;
//    				float temph = c1h;
//    				c1h = c2h;
//    				c2h = temph;
        			fw = 1.0;
        			bw = 1.0;

        			// Body-Coxa Joint Position
        			positions.at(BC0) =  l * fac * lift0 * fw * 0.5 	   + ( targetBC[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
        			positions.at(BC1) =  l * fac * down1  * bw * 0.4 	   + ( targetBC[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
        			positions.at(BC2) =  l * fac * down2 * bw * 0.4 	   + ( targetBC[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];

        			// Coxa-Femur Joint Position
        			positions.at(CF0) =  l * fac * down0 * bw * 0.55     + ( targetCF[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
        			positions.at(CF1) =  l *-fac * lift1  * bw * 0.5      + ( targetCF[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
        			positions.at(CF2) =  l *-fac * lift2 * bw * 0.55     + ( targetCF[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];

        			// Femur-Tibia Joint Position
//        			positions.at(FT0) =  l *fac * c2h * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//        			positions.at(FT1) =  l *fac * c1  * 0.3       + ( targetFT[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//        			positions.at(FT2) =  l *fac * c1h * 0.3       + ( targetFT[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];

        			positions.at(FT0) =  ( targetFT[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
        			positions.at(FT1) =  ( targetFT[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
        			positions.at(FT2) =  ( targetFT[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
        			positions.at(FT3) =  ( targetFT[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
        			positions.at(FT4) =  ( targetFT[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
        			positions.at(FT5) =  ( targetFT[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

    			}
    			else if (doMotion == 'e'){

//    				float temp = c1;
//    				c1 = c2;
//    				c2 = temp;
//    				float temph = c1h;
//    				c1h = c2h;
//    				c2h = temph;
        			fw = 1.0;
        			bw = 1.0;

//        			fac = 1.2;

        			// Body-Coxa Joint Position
        			positions.at(BC3) =  r * fac * lift3  * fw * 0.5 	   + ( targetBC[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
        			positions.at(BC4) =  r * fac * down4 * bw * 0.4 	   + ( targetBC[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
        			positions.at(BC5) =  r * fac * down5  * bw * 0.4 	   + ( targetBC[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

        			// Coxa-Femur Joint Position
        			positions.at(CF3) =  r * fac * down3  * bw * 0.55 	  + ( targetCF[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
        			positions.at(CF4) =  r *-fac * lift4 * bw * 0.5      + ( targetCF[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
        			positions.at(CF5) =  r *-fac * lift5  * bw * 0.55     + ( targetCF[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

        			// Femur-Tibia Joint Position
        			positions.at(FT0) =  ( targetFT[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
        			positions.at(FT1) =  ( targetFT[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
        			positions.at(FT2) =  ( targetFT[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
        			positions.at(FT3) =  ( targetFT[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
        			positions.at(FT4) =  ( targetFT[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
        			positions.at(FT5) =  ( targetFT[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
    			}
    			else if (doMotion == 'R' or doMotion == 'u' or doMotion == 'o' or doMotion == 't'){
//					float temp = c1;
//					c1 = c2;
//					c2 = temp;
//					float temph = c1h;
//					c1h = c2h;
//					c2h = temph;
	    			fw = 1.0;
	    			bw = 1.0;
	    			fac = 0.5;
        			MC[0]->setPsnInputNeurons(0,1);
        			MC[3]->setPsnInputNeurons(0,1);
					if(doMotion == 't'){
						MC[1]->setPsnInputNeurons(0,1);
						MC[2]->setPsnInputNeurons(0,1);
						MC[4]->setPsnInputNeurons(0,1);
						MC[5]->setPsnInputNeurons(0,1);
					}

					if (abs(angle[0]) < 10){

// /////////////////////////////////////////
						positions.at(BC0) =  l * fac * lift0  * fw * BC_Roll_fac[0]*pitch_front_amp_gain  	   + ( targetBC_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
						positions.at(BC1) =  l * fac * down1  * bw * BC_Roll_fac[1]*pitch_back_amp_gain 	   + ( targetBC_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
						positions.at(BC2) =  l * fac * down2  * bw * BC_Roll_fac[2]*pitch_back_amp_gain 	   + ( targetBC_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
						positions.at(BC3) =  r * fac * lift3  * fw * BC_Roll_fac[0]*pitch_front_amp_gain 	   + ( targetBC_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
						positions.at(BC4) =  r * fac * down4  * bw * BC_Roll_fac[1]*pitch_back_amp_gain 	   + ( targetBC_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
						positions.at(BC5) =  r * fac * down5  * bw * BC_Roll_fac[2]*pitch_back_amp_gain  	   + ( targetBC_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

						// Coxa-Femur Joint Position
						positions.at(CF0) =  l * fac * down0  * bw * CF_Roll_fac[0]*1   + ( targetCF_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
						positions.at(CF1) =  l *-fac * lift1  * bw * CF_Roll_fac[1]     + ( targetCF_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
						positions.at(CF2) =  l *-fac * lift2  * bw * CF_Roll_fac[2]     + ( targetCF_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
						positions.at(CF3) =  r * fac * down3  * bw * CF_Roll_fac[0]*1 	+ ( targetCF_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
						positions.at(CF4) =  r *-fac * lift4  * bw * CF_Roll_fac[1]     + ( targetCF_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
						positions.at(CF5) =  r *-fac * lift5  * bw * CF_Roll_fac[2]     + ( targetCF_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

						// Femur-Tibia Joint Position
						positions.at(FT0) =  l * fac * down0  * FT_Roll_fac[0] 			+ ( targetFT_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
						positions.at(FT1) =  l * fac * lift1  * FT_Roll_fac[1]     		+ ( targetFT_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
						positions.at(FT2) =  l *-fac * lift2  * FT_Roll_fac[2]       	+ ( targetFT_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
						positions.at(FT3) =  r * fac * down3  * FT_Roll_fac[0]  			+ ( targetFT_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
						positions.at(FT4) =  r * fac * lift4  * FT_Roll_fac[1]      		+ ( targetFT_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
						positions.at(FT5) =  r *-fac * lift5  * FT_Roll_fac[2]       	+ ( targetFT_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
// /////////////////////////////////////////

						positions.at(TA) =  targetTA_Roll*degtoRad;
					}
					else{
						positions.at(BC0) =  l * fac * lift0  * fw * BC_Roll_fac[0]*pitch_front_amp_gain  	   + ( targetBC_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
						positions.at(BC1) =   ( targetBC_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
						positions.at(BC2) =   ( targetBC_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
						positions.at(BC3) =  r * fac * lift3  * fw * BC_Roll_fac[0]*pitch_front_amp_gain 	   + ( targetBC_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
						positions.at(BC4) =   ( targetBC_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
						positions.at(BC5) =   ( targetBC_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

						// Coxa-Femur Joint Position
						positions.at(CF0) =  l * fac * down0  * bw * CF_Roll_fac[0]*1   + ( targetCF_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
						positions.at(CF1) =   ( targetCF_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
						positions.at(CF2) =   ( targetCF_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
						positions.at(CF3) =  r * fac * down3  * bw * CF_Roll_fac[0]*1 	+ ( targetCF_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
						positions.at(CF4) =   ( targetCF_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
						positions.at(CF5) =   ( targetCF_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

						// Femur-Tibia Joint Position
						positions.at(FT0) =  l * fac * down0  * FT_Roll_fac[0] 			+ ( targetFT_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
						positions.at(FT1) =   ( targetFT_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
						positions.at(FT2) =   ( targetFT_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
						positions.at(FT3) =  r * fac * down3  * FT_Roll_fac[0]  			+ ( targetFT_Roll[0]  * degtoRad);//( MC[0].getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
						positions.at(FT4) =   ( targetFT_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
						positions.at(FT5) =   ( targetFT_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
						
					}
					if (doMotion == 'o'){
						MC[1]->setPsnInputNeurons(0,0);
						MC[2]->setPsnInputNeurons(0,0);
						MC[4]->setPsnInputNeurons(0,1);
						MC[5]->setPsnInputNeurons(0,1);
						// r = 0.5;
						positions.at(BC4) =  r * fac * down4 * bw * BC_Roll_fac[4] 	   + ( targetBC_Roll[1]  * degtoRad);
						positions.at(BC5) =  r * fac * down5  * bw * BC_Roll_fac[5] 	   + ( targetBC_Roll[2]  * degtoRad);

						positions.at(CF4) =  r *-fac * lift4 * bw * CF_Roll_fac[4]      + ( targetCF_Roll[1]  * degtoRad);
						positions.at(CF5) =  r *-fac * lift5  * bw * CF_Roll_fac[5]     + ( targetCF_Roll[2]  * degtoRad);

						positions.at(FT4) =  r *fac * lift4  * FT_Roll_fac[1]*-1       	+ ( targetFT_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
						positions.at(FT5) =  r *fac * lift5  * FT_Roll_fac[2]*-1       	+ ( targetFT_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
					
						// positions.at(CF3) -=  0.1;
						// positions.at(FT3) +=  0.1;

						}
					else if (doMotion == 'u') {
						MC[1]->setPsnInputNeurons(0,1);
						MC[2]->setPsnInputNeurons(0,1);
						MC[4]->setPsnInputNeurons(0,0);
						MC[5]->setPsnInputNeurons(0,0);
						// l = 0.5;
						positions.at(BC1) =  l * fac * down1  * bw * BC_Roll_fac[1] 	   + ( targetBC_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
						positions.at(BC2) =  l * fac * down2 * bw * BC_Roll_fac[2] 	   + ( targetBC_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];

						positions.at(CF1) =  l *-fac * lift1  * bw * CF_Roll_fac[1]      + ( targetCF_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
						positions.at(CF2) =  l *-fac * lift2 * bw * CF_Roll_fac[2]     + ( targetCF_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];

						positions.at(FT4) =  r *fac * lift4  * FT_Roll_fac[1]*-1       	+ ( targetFT_Roll[1]  * degtoRad);//( MC[0].getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
						positions.at(FT5) =  r *fac * lift5  * FT_Roll_fac[2]*-1       	+ ( targetFT_Roll[2]  * degtoRad);//( MC[0].getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
					}

				}

    			// cout << fw << " " << bw << " " << endl;
    //			positions.at(TA) = 0.349206349;
    			// positions.at(TA) = testParameters[10] * degtoRad * bb;
    			// cout << "TA position : " << positions.at(TA) << endl;

    			// fc_err_fac = testParameters[9];
    			// cout << "fc_err_fac : " << fc_err_fac << "\n";

    			if(activate_fc_closeloop and (doMotion != 'R' and doMotion != 'r')){
					positions.at(BC0) -= fc_error_st[0] * fc_err_fac;
					positions.at(BC3) -= fc_error_st[3] * fc_err_fac;
					positions.at(BC1) += fc_error_st[1] * fc_err_fac;
					positions.at(BC2) += fc_error_st[2] * fc_err_fac;
					positions.at(BC4) += fc_error_st[4] * fc_err_fac;
					positions.at(BC5) += fc_error_st[5] * fc_err_fac;

					positions.at(CF0) += fc_error_st[0] * fc_err_fac;
					positions.at(CF3) += fc_error_st[3] * fc_err_fac;
					positions.at(CF1) += fc_error_st[1] * fc_err_fac;
					positions.at(CF2) += fc_error_st[2] * fc_err_fac;
					positions.at(CF4) += fc_error_st[4] * fc_err_fac;
					positions.at(CF5) += fc_error_st[5] * fc_err_fac;
		//
					// positions.at(FT0) += fc_error_st[0] * fc_err_fac*ball_push_fac;
					// positions.at(FT3) += fc_error_st[3] * fc_err_fac*ball_push_fac;
					// positions.at(FT1) += fc_error_st[1] * fc_err_fac*ball_push_fac;
					// positions.at(FT2) += fc_error_st[2] * fc_err_fac*ball_push_fac;
					// positions.at(FT4) += fc_error_st[4] * fc_err_fac*ball_push_fac;
					// positions.at(FT5) += fc_error_st[5] * fc_err_fac*ball_push_fac;

    			}
    			else if(activate_fc_closeloop and (doMotion == 'R' or doMotion == 'r')){
    				// rolling only activate CF
					positions.at(CF0) += fc_error_st[0] * fc_err_fac;
					positions.at(CF3) += fc_error_st[3] * fc_err_fac;
					positions.at(CF1) += fc_error_st[1] * fc_err_fac;
					positions.at(CF2) += fc_error_st[2] * fc_err_fac;
					positions.at(CF4) += fc_error_st[4] * fc_err_fac;
					positions.at(CF5) += fc_error_st[5] * fc_err_fac;

					positions.at(FT0) += fc_error_st[0] * fc_err_fac;
					positions.at(FT3) += fc_error_st[3] * fc_err_fac;
					positions.at(FT1) -= fc_error_st[1] * fc_err_fac;
					positions.at(FT2) -= fc_error_st[2] * fc_err_fac;
					positions.at(FT4) -= fc_error_st[4] * fc_err_fac;
					positions.at(FT5) -= fc_error_st[5] * fc_err_fac;

					// Contact Manipulation
					positions.at(BC0) += ball_push_sig[0] * ball_push_gain;
					positions.at(BC3) += ball_push_sig[3] * ball_push_gain;

					positions.at(BC1) += ball_push_sig[1] * ball_push_gain;
					positions.at(BC2) += ball_push_sig[2] * ball_push_gain;
					positions.at(BC4) += ball_push_sig[4] * ball_push_gain;
					positions.at(BC5) += ball_push_sig[5] * ball_push_gain;

					positions.at(CF0) += ball_push_sig[0] * ball_push_gain;
					positions.at(CF3) += ball_push_sig[3] * ball_push_gain;

					positions.at(CF1) += ball_push_sig[1] * ball_push_gain;
					positions.at(CF2) += ball_push_sig[2] * ball_push_gain;
					positions.at(CF4) += ball_push_sig[4] * ball_push_gain;
					positions.at(CF5) += ball_push_sig[5] * ball_push_gain;

					positions.at(FT0) += ball_push_sig[0] * ball_push_gain;
					positions.at(FT3) += ball_push_sig[3] * ball_push_gain;

					positions.at(FT1) += ball_push_sig[1] * ball_push_gain;
					positions.at(FT2) += ball_push_sig[2] * ball_push_gain;
					positions.at(FT4) += ball_push_sig[4] * ball_push_gain;
					positions.at(FT5) += ball_push_sig[5] * ball_push_gain;

					if (activate_roll_control){
						// roll velocity --> leg extension reflex
					cout << "activate_roll_control " << endl;
					// angular velocity regulate leg movement reflex
					positions.at(CF0) += roll_cpg_mod_l * roll_leg_fac;
					positions.at(CF3) += roll_cpg_mod_r * roll_leg_fac;

					positions.at(FT0) += roll_cpg_mod_l * roll_leg_fac*2.0;
					positions.at(FT3) += roll_cpg_mod_r * roll_leg_fac*2.0;
					}

    			}

    			if (activate_sw_avoid){
					// for(int i =0; i<6; i++){
					// 	sw_err_fac[i] = 0.7;
					// }
					positions.at(BC0) -= sw_error[0] * sw_err_fac[0]*1.0 - tau_reflexChain[0].Read(reflex_delay)* sw_err_fac[0]*1.5;
					positions.at(BC3) -= sw_error[3] * sw_err_fac[3]*1.0 - tau_reflexChain[3].Read(reflex_delay)* sw_err_fac[3]*1.5;
					positions.at(BC1) -= sw_error[1] * sw_err_fac[1]*1.5 ;
					positions.at(BC2) -= sw_error[2] * sw_err_fac[2]*1.5 ;
					positions.at(BC4) -= sw_error[4] * sw_err_fac[4]*1.5 ;
					positions.at(BC5) -= sw_error[5] * sw_err_fac[5]*1.5 ;

					// cout << "tau_reflexChain  :  " << tau_reflexChain[3].Read(5) << endl;

					positions.at(CF0) -= sw_error[0] * sw_err_fac[0] * 1.5;
					positions.at(CF3) -= sw_error[3] * sw_err_fac[3] * 1.5;
					positions.at(CF1) -= tau_reflexChain[1].Read(reflex_delay)* sw_err_fac[1] * 1.5;//sw_error[1] * sw_err_fac * 1.5;
					positions.at(CF2) -= tau_reflexChain[2].Read(reflex_delay)* sw_err_fac[2] * 1.5;//sw_error[2] * sw_err_fac * 1.5;
					positions.at(CF4) -= tau_reflexChain[4].Read(reflex_delay)* sw_err_fac[4] * 1.5;//sw_error[4] * sw_err_fac * 1.5;
					positions.at(CF5) -= tau_reflexChain[5].Read(reflex_delay)* sw_err_fac[5] * 1.5;//sw_error[5] * sw_err_fac * 1.5;

			//		for(int i=0; i<6; i++){
					positions.at(FT0) +=  (sw_error[0] * sw_err_fac[0] * 1.5 );// - sw_switch_err[0];
					positions.at(FT3) +=  (sw_error[3] * sw_err_fac[3] * 1.5 );// - sw_switch_err[3];
					positions.at(FT1) +=  (sw_error[1] * sw_err_fac[1] * 1.5 );// - sw_switch_err[1];
					positions.at(FT2) +=  (sw_error[2] * sw_err_fac[2] * 1.5 );// - sw_switch_err[2];
					positions.at(FT4) +=  (sw_error[4] * sw_err_fac[4] * 1.5 );// - sw_switch_err[4];
					positions.at(FT5) +=  (sw_error[5] * sw_err_fac[5] * 1.5 );// - sw_switch_err[5];

    			}
    			if(doMotion == 'g'){

					positions.at(BC2) = targetBC[2]  * degtoRad;
					positions.at(CF2) = targetCF[2]  * degtoRad;
					positions.at(FT2) = targetFT[2]  * degtoRad;

					positions.at(BC5) = targetBC[2]  * degtoRad;
					positions.at(CF5) = targetCF[2]  * degtoRad;
					positions.at(FT5) = targetFT[2]  * degtoRad;
    			}

				// cout << positions.at(TA) << endl;
				for (int i=0;i<1;i++){
					// cout << "position at FT3 " << i <<y " : " << positions.at(FT3) << endl;
					// cout << "CF_torque " << i << " : " << sw_sens_rawcf[i] << endl;
				}

				// publish joint position
				simRos->setLegMotorPosition(positions);	

				// // Muscle Model
				// // 5. Calculate desired joint velocities
				// desired_velocities.clear();
				// desired_velocities.resize(19);
				// for(int i=0; i<18; i++){
				// 	float dx = positions[i] - old_positions[i];
				// 	float angular_velocity = (float) dx/0.01666;
				// 	desired_velocities[i] = angular_velocity;
				// }
				// current_positions = jointPositions;
				// current_velocities = jointVelocities;
				desired_positions = positions;
				for (int i=0;i<current_positions.size();i++){
					trackingError[i] = desired_positions[i] - current_positions[i];
				}				
				// old_positions = positions;
      			// // 6. ADAPTATION LAW: Calculate joint torques
				// //vector<float> controlled_torque = complianceController->interpolateTorque(current_positions, desired_positions, current_velocities, desired_velocities); // Matrix K and D
				// std::vector<float> controlled_torque = complianceController->approximateTorque(current_positions, desired_positions, current_velocities, desired_velocities); // Vector K and D

				// // 7A. Threshold max torque
				// // vector<float> applied_torque;
				// applied_torque.resize(controlled_torque.size());
				// for(size_t i=0; i<controlled_torque.size(); i++)
				// {
				// 	if(controlled_torque[i] > MAX_TORQUE)
				// 	{
				// 		applied_torque.at(i) = MAX_TORQUE;
				// 	}
				// 	else if(controlled_torque[i] < -MAX_TORQUE)
				// 	{
				// 		applied_torque.at(i) = -MAX_TORQUE;
				// 	}
				// 	else
				// 	{
				// 		applied_torque.at(i) = controlled_torque[i];
				// 	}
				// }		
				// jointMaxTorqueArray = applied_torque;

				// publish joint Max Torque
				// for (int i=0;i<6;i++){
				// 	if(i%3 == 0){
				// 		jointMaxTorqueArray[i] = jointMaxTorque_frontL;
				// 	}
				// 	else{
				// 		jointMaxTorqueArray[i] = jointMaxTorque_frontL;
				// 	}
					
				// }
				// simRos->setMotorMaxTorque(jointMaxTorqueArray);	// publish joint position
    	//		std::cout << midBC[0] << "  " << targetBC[0] << " " << biasBC0 << "\n";


				// simRos->publish_neuron_activity(neuron_act);
    	//		MC[0].setCpgBias(0.01 + alpha*simRos.forceSensors[0], 0.01 + alpha*simRos.forceSensors[3]);

				// CPG interconnection 
				// hypo 1 : purturbation to CPG self stance inhibition
				cpg_input_mod[0] = -abs(-fc_sens_raw[0])*cpg_input_w_f*0.2; //- abs(fc_sens_raw[3])*cpg_input_w;
				cpg_input_mod[3] = -abs(-fc_sens_raw[3])*cpg_input_w_f*0.2; //- abs(fc_sens_raw[0])*cpg_input_w;

				// cpg_input_mod[1] = -abs(-fc_sens_raw[1])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[2])*cpg_input_w - abs(fc_sens_raw[4])*cpg_input_w;
				// cpg_input_mod[2] = -abs(-fc_sens_raw[2])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[1])*cpg_input_w - abs(fc_sens_raw[5])*cpg_input_w;
				// cpg_input_mod[4] = -abs(-fc_sens_raw[4])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[5])*cpg_input_w - abs(fc_sens_raw[1])*cpg_input_w;
				// cpg_input_mod[5] = -abs(-fc_sens_raw[5])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[4])*cpg_input_w - abs(fc_sens_raw[2])*cpg_input_w;

				// hypo 2 : left-right interleg swing inhibition
				// cpg_input_mod[0] = -abs(1-fc_sens[3])*cpg_input_w_f*0.2; 
				// cpg_input_mod[3] = -abs(1-fc_sens[0])*cpg_input_w_f*0.2;

				// cpg_input_mod[1] = -abs(1-fc_sens[4])*cpg_input_w_b*0.1;
				// cpg_input_mod[2] = -abs(1-fc_sens[5])*cpg_input_w_b*0.1;
				// cpg_input_mod[4] = -abs(1-fc_sens[1])*cpg_input_w_b*0.1;
				// cpg_input_mod[5] = -abs(1-fc_sens[2])*cpg_input_w_b*0.1;

				// hypo 3 : front-hind interleg swing inhibition
				// cpg_input_mod[0] += -abs(1-fc_sens[3])*cpg_input_w*0.1; 
				// cpg_input_mod[3] += -abs(1-fc_sens[0])*cpg_input_w*0.1; 

				// cpg_input_mod[1] += -abs(1-fc_sens[2])*cpg_input_w_b*0.1;
				// cpg_input_mod[2] += -abs(1-fc_sens[1])*cpg_input_w_b*0.1;
				// cpg_input_mod[4] += -abs(1-fc_sens[5])*cpg_input_w_b*0.1;
				// cpg_input_mod[5] += -abs(1-fc_sens[4])*cpg_input_w_b*0.1;

				// hypo 4 : interleg swing exitation
				// cpg_input_mod[1] += abs(fc_sens[5])*cpg_input_w_b*0.05; 
				// cpg_input_mod[2] += abs(fc_sens[4])*cpg_input_w_b*0.05; 
				// cpg_input_mod[4] += abs(fc_sens[2])*cpg_input_w_b*0.05;
				// cpg_input_mod[5] += abs(fc_sens[1])*cpg_input_w_b*0.05;

				// IMU CPG modulation 
				// same side tilt excite swing 
				// cpg_input_mod[0] = roll_cpg_mod_l*cpg_input_w_f*0.1; //- abs(fc_sens_raw[3])*cpg_input_w;
				// cpg_input_mod[3] = roll_cpg_mod_r*cpg_input_w_f*0.1; //- abs(fc_sens_raw[0])*cpg_input_w;
				// oppose side tilt excite swing
				// cpg_input_mod[0] = roll_cpg_mod_r*cpg_input_w_f*0.1; //- abs(fc_sens_raw[3])*cpg_input_w;
				// cpg_input_mod[3] = roll_cpg_mod_l*cpg_input_w_f*0.1; //- abs(fc_sens_raw[0])*cpg_input_w;

				// cpg_input_mod[3] -= roll_cpg_mod_r*cpg_input_w_f*0.2; //- abs(fc_sens_raw[3])*cpg_input_w;
				// cpg_input_mod[0] -= roll_cpg_mod_l*cpg_input_w_f*0.2; //- abs(fc_sens_raw[0])*cpg_input_w;

				for (int i=0; i<6; i++){
					// phase modulation force to cpg self inhibitiom
					// cpg_input_mod[i] = -abs(-fc_sens_raw[i])*cpg_input_w*0.1;

					// MC[i]->setCpgInputNeuron(cpg_input_mod[i]);
				}

				for (int i=0; i<6; i++){
					MC[i]->step();				// update all neuron activity
					tauCF[i].Step();
					tau_reflexChain[i].Write(sw_error[i]);
					tau_reflexChain[i].Step();
				}
				// GRAB CPG
				// for (int i=0; i<6; i++){
				// 	float n = i%3;
				// 	float localJointPos_bc = jointPositions[i]   - targetBC_Roll[n] / BC_Roll_fac[n];
				// 	float localJointPos_cf = jointPositions[i+6] - targetCF_Roll[n] / CF_Roll_fac[n];
				// 	MC[i]->cpg->updateLoss(positions.at(i), localJointPos_bc, 
				// 							positions.at(i+6), localJointPos_cf);
				// }
			}
//            standcount = 0;

			// Visualize data
			// Loop data
			for (int i=0;i<3;i++){
				// plotdata.push_back(openlsignal[i]);
				// plotdata.push_back(MC[i]->getpcpgOutput1(0));
				// plotdata.push_back(sw_err_fac[i]);
				// plotdata.push_back(omega[i]);
				// plotdata.push_back(angle[i]);
				// plotdata.push_back(joint_pos_ball_contact_sens[i]);
			}
			plotdata.push_back(MC[0]->getCpgOutput(0));
			// Single data
			// plotdata.push_back(e_ball);
			// plotdata.push_back(ball_push_sig[1]);

			simRos->plot(plotdata);		// publish data for plotting in V-REP
        }
        else{
//        	if (standcount == 0){
        		actuateStand();
//        		standcount++;
//        	}
//        	else{
//        		;
//        	}
        }

        //-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

		// Post process

		// Collect Data for Logging
		// Ball Control System
		ballControlData.push_back(simulationTime);
		ballControlData.push_back(ball_push_gain);
		ballControlData.push_back(target_robot_ball_distance);
		ballControlData.push_back(e_ball);
		ballControlData.push_back(ball_push_input);
		for(int c = 0; c < 6; c++){ ballControlData.push_back(joint_pos_ball_contact_sens[c]); }
		for(int c = 0; c < 6; c++){ ballControlData.push_back(ball_push_sig[c]); }
		
		// Local Leg Control System
		localLegData.push_back(simulationTime);
		for(int c = 0; c < 6; c++){ localLegData.push_back(fcphase[c]); }
		for(int c = 0; c < 6; c++){ localLegData.push_back(fc_sens_raw[c]); }
		for(int c = 0; c < 6; c++){ localLegData.push_back(fc_sens[c]); }
		for(int c = 0; c < 6; c++){ localLegData.push_back(fc_error_st[c]); }
		// Roll Control System
		rollControlData.push_back(simulationTime);
		rollControlData.push_back(activate_roll_control);
		for(int c = 0; c < 3; c++){ rollControlData.push_back(omega[c]); }
		for(int c = 0; c < 3; c++){ rollControlData.push_back(angle[c]); }
		rollControlData.push_back(roll_cpg_mod_l);
		rollControlData.push_back(roll_cpg_mod_r);
		rollControlData.push_back(roll_leg_fac);		
		// Pitch Control System
		pitchControlData.push_back(simulationTime);
		pitchControlData.push_back(activate_pitch_control);
		pitchControlData.push_back(pitch);
		pitchControlData.push_back(pitch_error);
		pitchControlData.push_back(pitch_freq_mod);
		pitchControlData.push_back(front_MI);
		pitchControlData.push_back(back_MI);
		pitchControlData.push_back(pitch_front_amp_gain);
		pitchControlData.push_back(pitch_back_amp_gain);
		for (int i=0;i<12;i++){pitchControlData.push_back(trackingError[i]); }		

		// Log Data to csv Files
        if ( savedata and savedatacounter < savelimit)
        {
        	cout << "savedatacounter " << savedatacounter << "\n";
			saveVector(ballControlData, ballControlLog);
			saveVector(rollControlData, rollControlLog);
			saveVector(pitchControlData, pitchControlLog);
			saveVector(localLegData, localLegLog);
        	savedatacounter = savedatacounter + 1;
        }
		sim_counter += 1;


		rclcpp::spin_some(simRos);
        simRos->rosSpinOnce();	// Synchronize ROS rate with V-REP

        // Termination of the simulation
        if ((simRos->getSimState() != 1 && waiter < 180) || simRos->getSimTerminateState())
        {
            // Print Goodbye Messages
//            cout << "Position Controller = " << controlType << endl;
            cout << "Shutting down the controller." << endl;

            // Close data files
            // fc_Loss.close();
			ballControlLog.close();
			localLegLog.close();
            // Shutdown ROS node
            rclcpp::shutdown();

            // Delete objects
            // delete simRos;

            return !simRos->getSimTerminateState();
        }
        else
        {
            return !simRos->getSimTerminateState();
        }
    }
    else
    {
    	cout << "rclcpp::ok == false, Shutting down the controller." << endl;
    	return false;
    }
}


double rescale(double oldMax, double oldMin, double newMax, double newMin, double parameter){
    return (((newMax-newMin)*(parameter-oldMin))/(oldMax-oldMin))+newMin;
}

bool dungBeetleController::actuateStand(){

	// Jumping Cool cool!
	// Body-Coxa Joint Position
	if( objective == 3 or doMotion == 'r'){
		targetBC_Roll[0] = testParameters[0];
		targetBC_Roll[1] = testParameters[1];
		targetBC_Roll[2] = testParameters[2];
		targetCF_Roll[0] = testParameters[3];
		targetCF_Roll[1] = testParameters[4];
		targetCF_Roll[2] = testParameters[5];
		targetFT_Roll[0] = testParameters[6];
		targetFT_Roll[1] = testParameters[7];
		targetFT_Roll[2] = testParameters[8];
		targetTA_Roll    = testParameters[9];

		positions.at(BC0) =  targetBC_Roll[0]*degtoRad;
		positions.at(BC1) =  targetBC_Roll[1]*degtoRad;
		positions.at(BC2) =  targetBC_Roll[2]*degtoRad;
		positions.at(BC3) =  targetBC_Roll[0]*degtoRad;
		positions.at(BC4) =  targetBC_Roll[1]*degtoRad;
		positions.at(BC5) =  targetBC_Roll[2]*degtoRad;

		// Coxa-Femur Joint Position
		positions.at(CF0) =  targetCF_Roll[0]*degtoRad;
		positions.at(CF1) =  targetCF_Roll[1]*degtoRad;
		positions.at(CF2) =  targetCF_Roll[2]*degtoRad;
		positions.at(CF3) =  targetCF_Roll[0]*degtoRad;
		positions.at(CF4) =  targetCF_Roll[1]*degtoRad;
		positions.at(CF5) =  targetCF_Roll[2]*degtoRad;

		// Femur-Tibia Joint Position
		positions.at(FT0) =  targetFT_Roll[0]*degtoRad;
		positions.at(FT1) =  targetFT_Roll[1]*degtoRad;
		positions.at(FT2) =  targetFT_Roll[2]*degtoRad;
		positions.at(FT3) =  targetFT_Roll[0]*degtoRad;
		positions.at(FT4) =  targetFT_Roll[1]*degtoRad;
		positions.at(FT5) =  targetFT_Roll[2]*degtoRad;

		positions.at(TA) = targetTA_Roll*degtoRad;

		// positions.at(CF0) += fc_error_st[0] * fc_err_fac;
		// positions.at(CF3) += fc_error_st[3] * fc_err_fac;
		positions.at(CF1) += fc_error_st[1] * fc_err_fac;
		positions.at(CF2) += fc_error_st[2] * fc_err_fac;
		positions.at(CF4) += fc_error_st[4] * fc_err_fac;
		positions.at(CF5) += fc_error_st[5] * fc_err_fac;

		positions.at(FT1) -= fc_error_st[1] * fc_err_fac;
		positions.at(FT2) -= fc_error_st[2] * fc_err_fac;
		positions.at(FT4) -= fc_error_st[4] * fc_err_fac;
		positions.at(FT5) -= fc_error_st[5] * fc_err_fac;


	}
	else{
		positions.at(BC0) =  targetBC[0]*degtoRad;
		positions.at(BC1) =  targetBC[1]*degtoRad;
		positions.at(BC2) =  targetBC[2]*degtoRad;
		positions.at(BC3) =  targetBC[0]*degtoRad;
		positions.at(BC4) =  targetBC[1]*degtoRad;
		positions.at(BC5) =  targetBC[2]*degtoRad;

		// Coxa-Femur Joint Position
		positions.at(CF0) =  targetCF[0]*degtoRad;
		positions.at(CF1) =  targetCF[1]*degtoRad;
		positions.at(CF2) =  targetCF[2]*degtoRad;
		positions.at(CF3) =  targetCF[0]*degtoRad;
		positions.at(CF4) =  targetCF[1]*degtoRad;
		positions.at(CF5) =  targetCF[2]*degtoRad;

		// Femur-Tibia Joint Position
		positions.at(FT0) =  targetFT[0]*degtoRad;
		positions.at(FT1) =  targetFT[1]*degtoRad;
		positions.at(FT2) =  targetFT[2]*degtoRad;
		positions.at(FT3) =  targetFT[0]*degtoRad;
		positions.at(FT4) =  targetFT[1]*degtoRad;
		positions.at(FT5) =  targetFT[2]*degtoRad;

		positions.at(TA) = targetTA*degtoRad;
	}

	if(stand_extend == true){
		std::vector<float> CF_Torque = footContactSensors;

		for(int i = 0; i<6; i++){
			if (CF_Torque[i] > 1.0){
				sens_st[i] = 1;
			}
			else {
				sens_st[i] = 0;
			}
			//// calculate error ////
			if(sens_st[i] == 1){
				error_st[i] = 0.99 * olderror_st[i] +
							(1.0-0.99) * (1.0 - sens_st[i]);
			}
			else if(sens_st[i] == 0){
				error_st[i] -= 0.01;
			}
		}

		positions.at(CF0) += error_st[0] * err_st_fac;
		positions.at(CF3) += error_st[3] * err_st_fac;
		positions.at(CF1) += error_st[1] * err_st_fac;
		positions.at(CF2) += error_st[2] * err_st_fac;
		positions.at(CF4) += error_st[4] * err_st_fac;
		positions.at(CF5) += error_st[5] * err_st_fac;

		positions.at(FT0) -= error_st[0] * err_st_fac;
		positions.at(FT3) -= error_st[3] * err_st_fac;
		positions.at(FT1) -= error_st[1] * err_st_fac;
		positions.at(FT2) -= error_st[2] * err_st_fac;
		positions.at(FT4) -= error_st[4] * err_st_fac;
		positions.at(FT5) -= error_st[5] * err_st_fac;


		olderror_st = error_st;

	}

	// Actuate jumping
	simRos->setLegMotorPosition(positions);	// publish joint position

	// // Muscle Model
	// // 5. Calculate desired joint velocities
	// desired_velocities.clear();
	// desired_velocities.resize(19);
	// for(int i=0; i<18; i++){
	// 	float dx = positions[i] - old_positions[i];
	// 	float angular_velocity = (float) dx/0.01666;
	// 	desired_velocities[i] = angular_velocity;
	// }
	// current_positions = jointPositions;
	// current_velocities = jointVelocities;
	// desired_positions = positions;
	// old_positions = positions;
	// // 6. ADAPTATION LAW: Calculate joint torques
	// //vector<float> controlled_torque = complianceController->interpolateTorque(current_positions, desired_positions, current_velocities, desired_velocities); // Matrix K and D
	// std::vector<float> controlled_torque = complianceController->approximateTorque(current_positions, desired_positions, current_velocities, desired_velocities); // Vector K and D

	// // 7A. Threshold max torque
	// // vector<float> applied_torque;
	// applied_torque.resize(controlled_torque.size());
	// for(size_t i=0; i<controlled_torque.size(); i++)
	// {
	// 	if(controlled_torque[i] > MAX_TORQUE)
	// 	{
	// 		applied_torque.at(i) = MAX_TORQUE;
	// 	}
	// 	else if(controlled_torque[i] < -MAX_TORQUE)
	// 	{
	// 		applied_torque.at(i) = -MAX_TORQUE;
	// 	}
	// 	else
	// 	{
	// 		applied_torque.at(i) = controlled_torque[i];
	// 	}
	// }		
	// jointMaxTorqueArray = applied_torque;
	// simRos->setMotorMaxTorque(jointMaxTorqueArray);	// publish joint position

}

bool dungBeetleController::actuateJump(){

}

bool dungBeetleController::actuateGallop(){

}

bool dungBeetleController::actuateBackFlip(){

}

void dungBeetleController::Logdata_loss( double simtime, std::vector<float> fc_error)
{
    fc_Loss << simtime <<"\t"<< fc_error[0] <<"\t"<< fc_error[1] <<"\t"<< fc_error[2] <<"\t"<< fc_error[3] <<"\t"<< fc_error[4] <<"\t"<< fc_error[5] << "\n";
}

//----------------------------------------------------------- getch ----------------------- Reads input from the keyboard to control DB-Alpha.
char dungBeetleController::getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = keyboard_input;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
		RCLCPP_ERROR(rclcpp::get_logger("db_beta_controller"), "tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		RCLCPP_ERROR(rclcpp::get_logger("db_beta_controller"), "tcsetattr ICANON");

    if(rv == -1)
		RCLCPP_ERROR(rclcpp::get_logger("db_beta_controller"), "select");
    else if(rv == 0);
        //ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		RCLCPP_ERROR(rclcpp::get_logger("db_beta_controller"), "tcsetattr ~ICANON");
    return (buff);
}

void dungBeetleController::setWeights(std::vector<int> i, std::vector<int> j, std::vector<float> array){
	int count = 0;
	for(auto row : i){
		for(auto column : j){
			c_matrix.at(row*neuron_num+column) = array.at(count);
			count++;
		}
	}
}

void dungBeetleController::setWeights1on1(std::vector<int> i, std::vector<int> j, std::vector<float> array){

	for(int count=0; count< i.size(); count++){
		c_matrix.at(i[count]*neuron_num+j[count]) = array.at(count);
	}
}

void dungBeetleController::setNeuron_activity(std::vector<int> neurons, std::vector<float> activity){
	for(int count=0; count< neurons.size(); count++){
		neuron_act.at(neurons[count]) = activity.at(count);
	}
}

float dungBeetleController::relu(float input){
	float output;
	if(input > 0){
		output = input;
	}
	else{
		output = 0;
	}
	return output;
}

float dungBeetleController::limitRangeFunction(float input, float lowerLimit, float upperLimit){
	float output;
	if(input > upperLimit){
		output = upperLimit;
	}
	else if(input < lowerLimit){
		output = lowerLimit;
	}
	else{
		output = input;
	}
	return output;
}

float dungBeetleController::lowerLimitFunction(float input, float lowerLimit){
	float output;
	if(input > lowerLimit){
		output = input;
	}
	else{
		output = 0.0;
	}
	return output;
}

float dungBeetleController::sigmoid(float x, float expfac, float a, float b){
	return 1/(1+exp(expfac*(x*a+b)));
}

// Save vector to file in CSV format
void dungBeetleController::saveVector(vector<float> vec, ofstream& file)
{
	file << vec.at(0);
    for(size_t i=1; i<vec.size(); i++)
    {
        file << ", " << vec[i];
    }
    file << endl;
}

void dungBeetleController::updateJointFeedback(){
	jointPositions = simRos->getJointFeedback();
}

void dungBeetleController::updateTorqueFeedback(){
	jointTorques = simRos->getTorqueFeedback();
}

void dungBeetleController::updateFootContactFeedback(){
	footContactSensors = simRos->getFootContactFeedback();
}

void dungBeetleController::updateTestParam(){
	testParameters = simRos->getTestParam();
}

void dungBeetleController::updateImuFeedback(){
	imuSensors = simRos->getImuFeedback();
}

void dungBeetleController::updateJoyAxesFeedback(){
	axes = simRos->getJoyAxesFeedback();
}

void dungBeetleController::updateJoyButtonFeedback(){
	buttons = simRos->getJoyButtonFeedback();
}

void dungBeetleController::updateSimTime(){
	simulationTime = simRos->getSimTime();
}


