
#include "dungBeetleController.h"
#include <math.h>
#include <chrono>

//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-


// Object creator
dungBeetleController::dungBeetleController(int argc,char* argv[]) 
{
	// Check Joy Stick
    if(useJoy){
        joystick = new Joystick;
        if (!joystick->isFound()){
        	printf("Joystick open failed.\n");
        }
        else{
        	printf("JoyStick Connected.\n");
        }
    }
    ////////////////////

    // Control the amount of time that the robot will be walking (time_walking = upper_limit - lower_limit)
    lower_limit = 500;
    upper_limit = 999999;// 2500
    
    // Select CPG type:
    //  - Case 1: Standard SO2 with MI value
    cpg_type = 1;

    // Initial message
    infoMessage();

	// Select mode to operate : Full Robot movement or Single Leg Testing 
    full_robot = true;//false;
    if(full_robot == true)
    {
        cout << "   .   " << endl;
        cout << "   .   " << endl;
        cout << "   .   " << endl;
        cout << "FULL ROBOT CONTROL " << endl;
    }
    else
	{
        cout << "   .   " << endl;
        cout << "   .   " << endl;
        cout << "   .   " << endl;
        cout << "SINGLE LEG CONTROL" << endl;
    }

    // Create object pointer instances
	// ROS Communication Initialize
	rclcpp::init(argc, argv);
	std::cout << "Run db_beta_controller" << std::endl;

	realRos = std::make_shared<realRos2Class>(argc, argv);
	// auto realRos = std::enable_shared_from_this(realRos_shared_ptr);
	// realRos = new realRos2Class(argc, argv);

    // int rate = 60; //Hz
    // rclcpp::Rate loop_rate(60);

	std::cout << "init real ros2 class node" << std::endl;
	
	// ALPHA Omnidirectional Locomotion CPG
	for(int i = 0; i<6; i++){
        Delayline tmp(30);
        tauCF.push_back(tmp);
    	modularController * MC_temp = nullptr;	// initiate modular controller
		MC_temp = new modularController(cpg_type, true);
    	MC.push_back(MC_temp);	// initiate modular controller
    }
    cout << "set CPG Done" << endl;
	// Initialize Vrn Input Neuron
	for (int i=0; i<6; i++){
		MC[i]->setVrnInputNeurons(0, 1);
		// MC[0]->setVrnInputNeurons(1, 1);
		MC[i]->setVrnInputNeurons(2, 1);
		// MC[0]->setVrnInputNeurons(3, 1);
		// Initialize PSN Input Neuron
		MC[i]->setPsnInputNeurons(0,1);
		// MC[i]->cpg->setGamma(0.0);
	}
    cout << "set VRN PSN Input Done" << endl;
	// Step CPG to stable period Details
	// std::vector<float> delayCPG;
	std::vector<float> CPGindex = {0, 1, 5};
	// Generate rev_tripod gait
	// for (int j : CPGindex){
	// 	// delay 70 step
	// 	for(int i=0; i<70; i++){
	// 		MC[j]->step();
	// 	}
    // }

	// Delay CPG to get tripod gait
	for (int j=0; j<6; j=j+2){
		for(int i=0; i<70; i++){
			MC[j]->step();
		}
    }
    cout << "set inter-leg coordination Done" << endl;
	// //update to steady state
	// for (int j=0; j<6; j++){
    // 	for(int n=0; n<200; n++){
	// 		MC[j]->step();
	// 	}    
	// }
    cout << "set CPG stable Done" << endl;

	//--------------------------------
    so2 = new SO2CPG(); 
	//destabilize cpg to oscillate
	so2->setOutput(0, 0.1);
	so2->setOutput(1, 0.1);
	so2->setActivity(0, 0.1);
	so2->setActivity(1, 0.1);

	//set cpg weights22
	so2->setWeight(0, 0, 1.4);
	so2->setWeight(0, 1, 0.18 + 0.2); // MI = 0.01, 0.07 middle, 0.2 fast
	so2->setWeight(1, 0,-0.18 - 0.2); // MI = 0.01
	so2->setWeight(1, 1, 1.4);

	//set bias
	so2->setBias(0, 0.0); // cpg_bias = 0.0
	so2->setBias(1, 0.0);
	//----------------------------------

    cout << "setting modular control" << endl;

	// Step CPG to stable period
	for(int i=0; i<200; i++){
		so2->step();
    }

    //pd_c = new PDcontroller (25, 10);
	// Muscle controller
	// paper: beta=0.05 , a=0.2, b=5
	// original: beta=0.05 , a=0.5, b=5000
    complianceController = new AMC(0.05, 0.5, 5000);

    // Wait 3 secs
    sleep(3);

    cout << "   .   " << endl;
    cout << "   .   " << endl;
    cout << "   .   " << endl;
    cout << "Controller is up and running" << endl;

	// Initialize Vector
    positions.resize(21);
    positions_target.resize(21);
	oneleg_positions.resize(3);
	couple_positions.resize(6);
    torques.resize(21);
    feedback_positions.resize(21);
    feedback_velocities.resize(21);
    imu_signal.resize(13);
    previous_positions.resize(21);
    previous_velocities.resize(21);

	// Initialize couter
    initial_counter = 300;
    data_counter = 300;
    dt = 1/FREQ;

    // Delayline Neurons
	// tau_reflexChain.resize(6);
    reflex_delay = 5;
    for(int i = 0; i<6; i++){
    	Delayline tmp_delay(10);
    	tau_reflexChain.push_back(tmp_delay);
    }

//	tau_downsignal0 = new Delayline(20);
//	tau_downsignal1 = new Delayline(20);
//	tau_downsignal2 = new Delayline(20);
//	tau_downsignal3 = new Delayline(20);
//	tau_downsignal4 = new Delayline(20);
//	tau_downsignal5 = new Delayline(20);

    // Set PHI
//    if (cpg_type == 2)
//    {
//        MAX_NETWORK_OUTPUT = 0.2;
//        CPG->setPhii(phii);
//        cout << "PHI set to " << phii << endl;
//    }
//    else if (cpg_type == 1)
//    {
//        if(mi == 0.02)
//        {
//            MAX_NETWORK_OUTPUT = 1;
//        }
//        else
//        {
//            MAX_NETWORK_OUTPUT = 0.9;
//        }
//
//        CPG->setMI(mi);
//        cout << "MI set to " << mi << endl;
//    }
    
	// Open file for collect experiment data
    // Do not forget to change paths. 
    // General instalation space of gorobots: /home/user/workspace/gorobots
    // Maybe "/home/user/workspace/dung_beetle_experiments/*.csv"
    pos_feedback_csv.open("/home/binggwong/experiments/position_feedback_rad.csv");
    vel_feedback_csv.open("/home/binggwong/experiments/velocity_feedback_rad_s.csv");
    torque_csv.open("/home/binggwong/experiments/torque_Nm.csv");
    current_csv.open("/home/binggwong/experiments/current_mA_.csv");
    pos_desired_csv.open("/home/binggwong/experiments/position_desired_rad.csv");
    pos_error_csv.open("/home/binggwong/experiments/position_error_rad.csv");
    vel_desired_csv.open("/home/binggwong/experiments/velocity_desired_rad_s.csv");
    vel_error_csv.open("/home/binggwong/experiments/velocity_error_rad_s.csv");
    current_feedback_csv.open("/home/binggwong/experiments/current_feedback_mA.csv");
    cpg_signal_csv.open("/home/binggwong/experiments/cpg_signal_csv.csv");
    pcpg_signal_csv.open("/home/binggwong/experiments/pcpg_signal_csv.csv");
    vrn_signal_csv.open("/home/binggwong/experiments/vrn_signal_csv.csv");
    psn_signal_csv.open("/home/binggwong/experiments/psn_signal_csv.csv");
    legsearch_signal_csv.open("/home/binggwong/experiments/legsearch_signal_csv.csv");
    imu_csv.open("/home/binggwong/experiments/imu_csv.csv");
    rollControl_csv.open("/home/binggwong/experiments/rollControl.csv");
    ballDistanceControl_csv.open("/home/binggwong/experiments/ballDistanceControl.csv");
    pitchControl_csv.open("/home/binggwong/experiments/pitchControl.csv");

	// Controller Start Count Down
    // for(int i = 3; i > -1; i--){
    // 	usleep(1000000);
    // 	cout << "Controller start in " << i << endl;
    // }
    cout << "Finish initialize dung BeetleController" << endl;

	// Timer
	end = std::chrono::system_clock::now();

}


// Function that runs the controller and keeps it active
bool dungBeetleController::runController() 
{
	// Check ROS State
    if(rclcpp::ok()) 
    {
    	// Check experiment mode (Full ROBOT vs One Leg)
        if(full_robot == true)
        {
		//--------------------------------------Full Robot CONTROL ----------------------------------------
            if(initial_counter > experiment_end_step){
            	cout << "Full Robot Experiment finish" << endl;
            	actuateRobot_standPosition(1);
            	// actuateRobot_standTorque();
            	return false;
            }
            // Start timing
			start = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = abs(end-start);
			std::cout << "elapsed_seconds: " << elapsed_seconds.count() << std::endl;
			std::cout << "Running Freq: " << 1/elapsed_seconds.count() << std::endl;
			// Some computation here
			end = std::chrono::system_clock::now();
			// std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            
            // Get robot sensor feedback
            updateJointFeedback();
            updateImuFeedback();
            // std::cout << "test: " << endl;
        	// Read Keyboard input control
            if(useKeyboard){
                if(inputCounter == 5){
                    keyboard_input = getch();
                    inputCounter = 0;
                }
                else{
                	inputCounter += 1;
                }

                doMotion = keyboard_input;
            }
            // std::cout << "test: " << endl;

            // Set a file size limit
            if(data_counter > lower_limit && data_counter < upper_limit)
            {
				std::cout << "size: " << feedback_positions.size()  << endl;
				feedback_positions = jointPositions;
            	feedback_velocities =jointVelocities;
            	feedback_current = jointTorques;
				// if (data_counter < 502){
				// 	imu_signal = imuData;
				// 	imu_signal[10] = imu_base_x;
				// 	imu_signal[11] = imu_base_y;
				// 	imu_signal[12] = imu_base_z;
				// }
				// else
				// {
				imu_signal = imuData;
				// }
				
            	feedback_positions.insert(feedback_positions.begin(), data_counter);
            	feedback_velocities.insert(feedback_velocities.begin(), data_counter);
            	feedback_current.insert(feedback_current.begin(), data_counter);
            	imu_signal.insert(imu_signal.begin(), data_counter);

                complianceController->saveVector(feedback_positions, pos_feedback_csv, 1);
                complianceController->saveVector(feedback_velocities, vel_feedback_csv, 1);
                complianceController->saveVector(feedback_current, current_feedback_csv, 1);
                complianceController->saveVector(imu_signal, imu_csv, 1);

				// Try to visualize IMU data
				std::vector<float> plotdata;
				// plotdata.push_back(imu_signal[11]);
				// plotdata.push_back(imu_signal[12]);
				// plotdata.push_back(imu_signal[13]);
				// realRos->dataPlot(plotdata);

            }
            // std::cout << "test: " << endl;
			// else if (data_counter > 400)
			// {
			// 	imu_signal = imuData;
			// 	imu_base_x += imu_signal[10];
			// 	imu_base_y += imu_signal[11];
			// 	imu_base_z += imu_signal[12];
			// }
			

            // Actuation
            if(initial_counter > 100) // Skip the initial cycles until the driver stabilises
            {
            // std::cout << "test: " << endl;
            //     if(initial_counter == 101)
            //     {
            //         cout << "Starting actuation." << endl;
            //     }
            //     // perform fix set of target behaviour
			// 	if (initial_counter < 800){
			// 		doMotion = 'R';
			// 	}else if (initial_counter < 1100){
			// 		doMotion = 'Y';
			// 	}else if (initial_counter < 1400){
			// 		doMotion = 'R';
			// 	}
				// }else if (initial_counter < 1000){
				// 	doMotion = 'b';
				// }else if (initial_counter < 1200){
				// 	doMotion = 'a';
				// }else if (initial_counter < 1300){
				// 	doMotion = 'w';
				// }else if (initial_counter < 1500){
				// 	doMotion = 'd';
				// }else if (initial_counter < 1700){
				// 	doMotion = 'q';
				// }else if (initial_counter < 1900){
				// 	doMotion = 'e';
				// }else if (initial_counter < 2000){
				// 	doMotion = ' ';
				// }
				// Test real time adaptation
				// if (initial_counter > 1000){
				// 	activate_pitch_control = false;
				// 	activate_roll_control  = true;
				// }

                switch(doMotion){
                case 'w': // forward walking
                	l = 1;
                	r = 1;
                	objective = 1; // objective for visualize data only
                	standAndWalk();
                	break;
                case 'b': // backward walking
                	objective = 2;
                	standAndWalk();
                	break;
                case 'G': // activate grabing
                	objective = 7;
                	activate_walking_grab = true;
                	actuateRobot_standPosition(2);
					cout << "walking grab ON" << endl;
                	break;
                case 'g': // deactivate grabing
                	activate_walking_grab = false;
					cout << "walking grab off" << endl;
                	break;
                case 'a': // left curve walking
                	objective = 3;
                	standAndWalk();
                	break;
                case 'd': // right curve walking
                	objective = 4;
                	standAndWalk();
                	break;
                case 'q': // left turning
                	objective = 5;
                	standAndWalk();
                	break;
                case 'e': // right turning
                	objective = 6;
                	standAndWalk();
                	break;
                case 'm': // muscle model stance and walking
                	l = 1;
                	r = 1;
                	standAndWalkTorque();
                	break;
                case 'c': // deactivate ground searching system
					activate_fc_closeloop = false;
					FC_state = 0;
					cout << "FC off" << endl;
                	break;
                case 'C': // activate ground searching system
					activate_fc_closeloop = true;
					FC_state = 1;
					cout << "FC on" << endl;
                	break;
                case 's': // deactivate swing reflex system
                	activate_sw_avoid = false;
					cout << "SW off" << endl;
                	break;
                case 'S': // activate swing reflex system
                	activate_sw_avoid = true;
					cout << "SW on" << endl;
                	break;
                // case '4':
                // 	activate_sw_switch = false;
				// 	cout << "SW switch off" << endl;
                // 	break;
                // case '$':
                // 	activate_sw_switch = true;
				// 	cout << "SW switch on" << endl;
                // 	break;
                case 'r': // activate static rolling posture
//                    for(int i = 3; i > -1; i--){
//                    	usleep(1000000);
//                    	cout << "Rolling Controller start in " << i << endl;
//                    }
					cout << "RRRRR   Actuate Rolling Pose Position   RRRRR" << endl;
					// actuateRobot_Rolling_pose_Position();
					actuateRobot_standPosition(1);
                	break;
                case 'R': // activate rolling movement
//                    for(int i = 3; i > -1; i--){
//                    	usleep(1000000);
//                    	cout << "Rolling Controller start in " << i << endl;
//                    }
					cout << "RRRRR   Actuate Rolling Pose Position   RRRRR" << endl;
					standAndRoll();
                	break;
                case 'T': // activate rolling movement
					cout << "RRRRR   Actuate Rolling Pose Position Reverse   RRRRR" << endl;
					standAndRoll();
                	break;
                case 'Y': // activate rolling movement
					cout << "RRRRR   Actuate Rolling Pose turning   RRRRR" << endl;
					standAndRoll();
                	break;
                case 'z': // rotate on the ball testing (not waoking now)
                	standAndWalk();
                	break;
                case 'k': // knocking door (not working now)
                	knock();
                	break;
                case 'p': // knocking door (not working now)
					temp = MC[0]->getMI()+0.02;
					MC[0]->setMI(temp);
					cout << temp << endl;
                	break;
                case 'l': // knocking door (not working now)
					temp = MC[0]->getMI()-0.02;
					MC[0]->setMI(temp);
					cout << temp << endl;
                	break;
                default: // standing posture
                	objective = 0;
                	actuateRobot_standPosition(1);
					// actuateRobot_standTorque();
					cout << "TEST" << endl;
                	break;
//
                }
            }

			positions_target.insert(positions_target.begin(), data_counter);
			if(data_counter > lower_limit && data_counter < upper_limit)
            {
				complianceController->saveVector(positions_target, pos_desired_csv, 1);
			}
            // Update cycle counter
            if(initial_counter > 999999)
            {
                initial_counter = 100; // restart
//                CPG->setCpgOutput(0, 0.05);
//                CPG->setCpgOutput(0, 0.05);
            }
            else
            {
                initial_counter++; // advance    
            }
            cout << "initial_counter: " << initial_counter << endl;
            cout << "data_counter   : " << data_counter << endl;
            data_counter++;
			// std::cout << "test: " << endl;

			// ROS Spin
    		rclcpp::spin_some(realRos);
			realRos->ros2_spin_some();
			// rclcpp::Rate loop_rate(60);
			// loop_rate.sleep();
			std::cout << "testxxx: " << endl;

            return true;
        }
        //-------------------------------------------------------------- ONE LEG CONTROL --------------------------------------------------------------------------
        else
        {
            if(initial_counter > experiment_end_step){
            	cout << "One Leg Experiment finish" << endl;
            	return false;
            }
        	cout << "one_leg_test" << endl;
            // Get robot feedback
            updateJointFeedback();
            
        	// Use Keyborad input control
            if(useKeyboard){
                if(inputCounter == 5){
                    keyboard_input = getch();
                    inputCounter = 0;
                }
                else{
                	inputCounter += 1;
                }

                doMotion = keyboard_input;
            }

            // Set a file size limit
            if(data_counter > lower_limit && data_counter < upper_limit)
            {
            	feedback_positions = jointPositions;
            	feedback_velocities = jointVelocities;
            	feedback_current = jointTorques;

            	feedback_positions.insert(feedback_positions.begin(), data_counter);
            	feedback_velocities.insert(feedback_velocities.begin(), data_counter);
            	feedback_current.insert(feedback_current.begin(), data_counter);

                complianceController->saveVector(feedback_positions, pos_feedback_csv, 1);
                complianceController->saveVector(feedback_velocities, vel_feedback_csv, 1);
                complianceController->saveVector(feedback_current, current_feedback_csv, 1);

//                complianceController->saveVector(jointPositions, pos_feedback_csv, 1);
//                complianceController->saveVector(jointVelocities, vel_feedback_csv, 1);
//                complianceController->saveVector(jointTorques, current_feedback_csv, 1);
                
                //complianceController->printVector(jointTorques, "Current Feedback [mA]", 1);
                //cout << endl;
            }

            // Take step with CPGs
//            CPG->step();

            // Actuation
            cout << initial_counter << endl;
            if(initial_counter > 150) //&& initial_counter < 100000
            {
//                stand_one_leg();

                switch(doMotion){
                case '1':
                	l = 1;
                	r = 1;
                	standAndWalk();
                	// standAndWalkTorque();
                	break;
                case 'b':
                	standAndWalk();
                	break;
                case 'c':
					activate_fc_closeloop = false;
					FC_state = 0;
					cout << "FC off" << endl;
                	break;
                case 'C':
					activate_fc_closeloop = true;
					FC_state = 1;
					cout << "FC on" << endl;
                	break;
                case 's':
                	activate_sw_avoid = false;
					cout << "SW off" << endl;
                	break;
                case 'S':
                	activate_sw_avoid = true;
					cout << "SW on" << endl;
                	break;
                case '4':
                	activate_sw_switch = false;
					cout << "SW switch off" << endl;
                	break;
                case '$':
                	activate_sw_switch = true;
					cout << "SW switch on" << endl;
                	break;
                default:
                // If you want Just standing tests uncomment these lines
               		actuateRobot_standPosition(1);
               		// actuateRobot_standTorque();
                	break;

                }
            }
			// positions_target.insert(positions_target.begin(), data_counter);
			// if(data_counter > 500 && data_counter < 10000)
            // {
			// 	complianceController->saveVector(positions_target, pos_desired_csv, 1);
			// }
            // Update cycle counter
            if(initial_counter > 2000)
            {
                initial_counter = 100; // restart
//                CPG->setCpgOutput(0, 0.05);
//                CPG->setCpgOutput(0, 0.05);
            }
            else
            {
                initial_counter++; // advance
            }
            data_counter++;

            // ROS Spin
    		rclcpp::spin_some(realRos);
			realRos->ros2_spin_some();
			std::cout << "testxxx: " << endl;

            return true;
        }   
    }
    else
    {
    	for(int i = 0; i<6; i++){
        	delete MC[i];
		}
        // delete realRos;
        delete complianceController;
        pos_feedback_csv.close();
        vel_feedback_csv.close();
        torque_csv.close();
        current_csv.close();
        pos_desired_csv.close();
        vel_desired_csv.close();
        current_feedback_csv.close();
        cout << "   .   " << endl;
        cout << "   .   " << endl;
        cout << "   .   " << endl;
        cout << "Shutting down the controller." << endl;
        cout << "Joint data saved to /home/binggwong/experiments/*.csv" << endl;

		rclcpp::shutdown();

        return false;
    }
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


// PUBLIC METHODS


// Y-Axis Re-scaling function (Amplitude)
float dungBeetleController::rescale(float oldMax, float oldMin, float newMax, float newMin, float parameter)
{
    return (((newMax-newMin)*(parameter-oldMin))/(oldMax-oldMin))+newMin;
}


// Torque to Current conversion for Dynamixel MX340-W350 servo motors.
// Linear conversion formula:
//      Current [A] = m * Torque [N*m] + n
float dungBeetleController::convertTorque2Current(float torque)
{
    float m = 0.5828571429;
    float n = 0.072;

    float current = torque*m + n;
    return current; 
}


// Inverse conversion of previous function
float dungBeetleController::convertCurrent2Torque(float current)
{
    float m = 0.5828571429;
    float n = 0.072;

    float torque = (current - n)/m;
    return torque; 
}

vector<float> dungBeetleController::limitTorque(vector<float> torque)
{
    vector<float> new_torque;
    new_torque.resize(torque.size());
    for(size_t i=0; i<torque.size(); i++)
    {
        if(torque[i] > MAX_TORQUE)
        {
            new_torque.at(i) = MAX_TORQUE;
        }
        else if (torque[i] < -MAX_TORQUE)
        {
            new_torque.at(i) = -MAX_TORQUE;
        }
        else
        {
            new_torque.at(i) = torque[i];
        }
    }
    return new_torque;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


// TORQUE CONTROLLER - Standing
void dungBeetleController::actuateRobot_standTorque() 
{
    // 1. Set desired positions and velocities
    vector<float> pos_desired;
    for(int i=0; i<21; i++)
    {
        //pos_desired.push_back(home_position[i]);
        pos_desired.push_back(dung_beetle_pose[i]);
    }

    vector<float> vel_desired;
    for(int i=0; i<21; i++)
    {
        vel_desired.push_back(0);
    }

    // 2. Get feedback
    vector<float> pos_feedback;
    for(int i=0; i<21; i++)
    {
        pos_feedback.push_back(jointPositions[i]);
    }
    
    vector<float> vel_feedback;
    for(int i=0; i<21; i++)
    {
        vel_feedback.push_back(jointVelocities[i]);
    }

    // 3. Calculate torques:
    vector<float> taus = complianceController->approximateTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive Impedance PD controller
//    vector<float> taus = complianceController->porportionalTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive P controller
//    vector<float> taus = pd_c->calculateOutputTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Simple impedance PD controller
    vector<float> tau_ = limitTorque(taus);
    
    if(initial_counter < upper_limit)
    {
        complianceController->saveVector(pos_desired, pos_desired_csv, 1);
        complianceController->saveVector(vel_desired, vel_desired_csv, 1);
        complianceController->saveVector(tau_, torque_csv, 1);
    }

    // 4. Convert torque into current:
    vector<float> tau_ext;
    for(size_t i=0; i<tau_.size(); i++)
    {
        float amps = convertTorque2Current(tau_[i]);
        float mili_amps = amps*1000;
        tau_ext.push_back(mili_amps);
    }
    complianceController->saveVector(tau_ext, current_csv, 1);

    // 5. Set Torques
	// if(full_robot == true){
		torques.at(BC0) = tau_ext.at(BC0);
		torques.at(BC1) = tau_ext.at(BC1);
		torques.at(BC2) = tau_ext.at(BC2);
		torques.at(BC3) = tau_ext.at(BC3);
		torques.at(BC4) = tau_ext.at(BC4);
		torques.at(BC5) = tau_ext.at(BC5);

		torques.at(CF0) = tau_ext.at(CF0);
		torques.at(CF1) = tau_ext.at(CF1);
		torques.at(CF2) = tau_ext.at(CF2);
		torques.at(CF5) = tau_ext.at(CF5);
		torques.at(CF4) = tau_ext.at(CF4);
		torques.at(CF3) = tau_ext.at(CF3);

		torques.at(FT0) = tau_ext.at(FT0);
		torques.at(FT1) = tau_ext.at(FT1);
		torques.at(FT2) = tau_ext.at(FT2);
		torques.at(FT5) = tau_ext.at(FT5);
		torques.at(FT4) = tau_ext.at(FT4);
		torques.at(FT3) = tau_ext.at(FT3);

		torques.at(LONGITUDINAL) = tau_ext.at(LONGITUDINAL);
		torques.at(TRANSVERSAL) = tau_ext.at(TRANSVERSAL);
		torques.at(HEAD) = tau_ext.at(HEAD);

    	realRos->updateMotorState(home_names, pos_desired, home_velocity, torques);
	// }
	// else{
    // 	// cout << "set motor position" << endl;
    // 	// realRos->updateMotorState(one_leg_names, one_leg_names_test_pos, home_velocity, torques);
	// }


    // Set joint positions: Dynamixel Protocol 1.0
    /*std::vector<float> torquesNew = {11,positions.at(BC2),12,positions.at(CF2),13,positions.at(FT2),
                                       21,positions.at(BC5),22,positions.at(CF5),23,positions.at(FT5),
                                       31,positions.at(BC1),32,positions.at(CF1),33,positions.at(FT1),
                                       41,positions.at(BC4),42,positions.at(CF4),43,positions.at(FT4),
                                       51,positions.at(BC0),52,positions.at(CF0),53,positions.at(FT0),
                                       61,positions.at(BC3),62,positions.at(CF3),63,positions.at(FT3),
                                       71,positions.at(LONGITUDINAL),72,positions.at(TRANSVERSAL),73,positions.at(HEAD)};*/

    //realRos->setLegMotorTorques(torquesNew);
    // realRos->updateMotorState(home_names, pos_desired, home_velocity, torques);

    // Update values
    TC_0_previous = pos_desired.at(BC0);
    TC_3_previous = pos_desired.at(BC3);
    CF_0_previous = pos_desired.at(CF0);
    CF_3_previous = pos_desired.at(CF3);
    previous_positions.clear();
    previous_velocities.clear(); 
    previous_positions = complianceController->copyVector(pos_desired);
    previous_velocities = complianceController->copyVector(vel_desired);
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


// TORQUE CONTROLLER - Walking
void dungBeetleController::actuateRobot_walkingTorque()
{
	int cc = 0;
    // 1. Modular Neural Network

	////// Implementing db_neural controller
	///////////////////////////////////////////
	vector<float> pos_desired;
    pos_desired.resize(21);

	float c1  = MC[0]->getFinalNeuronOutput(0);
	float c1h = MC[0]->getFinalNeuronOutput(1);

	float c2  = MC[0]->getFinalNeuronOutput(2);
	float c2h = MC[0]->getFinalNeuronOutput(3);

	fac = 0.5;

	if (isDenmark_db_alpha == true){
		//// position = factor amplitude * oscillated signal(rad) + joint bias(rad) * conversion factor degree to radian
		pos_desired.at(BC0) =  -fac * c1h * 0.4 	   + ( targetBCl[0]  * degtoRad);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
		pos_desired.at(BC1) =  -fac * c2  * 0.3 	   + ( targetBCl[1]  * degtoRad);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
		pos_desired.at(BC2) =  -fac * c2h * 0.3 	   + ( targetBCl[2]  * degtoRad);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
		pos_desired.at(BC3) =   fac * c1  * 0.4 	   + ( targetBCr[0]  * degtoRad);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
		pos_desired.at(BC4) =   fac * c2h * 0.3 	   + ( targetBCr[1]  * degtoRad);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
		pos_desired.at(BC5) =   fac * c2  * 0.3 	   + ( targetBCr[2]  * degtoRad);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

		// Coxa-Femur Joint Position
		pos_desired.at(CF0) =   fac * c2h * 0.55      + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
		pos_desired.at(CF1) =  -fac * c1  * 0.5       + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
		pos_desired.at(CF2) =  -fac * c1h * 0.55      + ( targetCF[2]  * degtoRad);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
		pos_desired.at(CF3) =   fac * c2  * 0.55 	  + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
		pos_desired.at(CF4) =  -fac * c1h * 0.5       + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
		pos_desired.at(CF5) =  -fac * c1  * 0.55      + ( targetCF[2]  * degtoRad);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

		// Femur-Tibia Joint Position
		pos_desired.at(FT0) =  fac * c2h * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
		pos_desired.at(FT1) =  fac * c1  * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
		pos_desired.at(FT2) =  fac * c1h * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
		pos_desired.at(FT3) =  fac * c2  * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		pos_desired.at(FT4) =  fac * c1h * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
		pos_desired.at(FT5) =  fac * c1  * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		pos_desired.at(LONGITUDINAL) = dung_beetle_pose.at(LONGITUDINAL);
		pos_desired.at(TRANSVERSAL) = dung_beetle_pose.at(TRANSVERSAL);
		pos_desired.at(HEAD) = dung_beetle_pose.at(HEAD);
	}
	else{
		// Thailand db_alpha

		//// position = oscillated signal(rad) + joint bias(rad)
//		pos_desired.at(BC0) =   fac * c1h * 0.4 	   + ( targetBC[0]  * degtoRad);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
//		pos_desired.at(BC1) =   fac * c2  * 0.3 	   + ( targetBC[1]  * degtoRad);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
//		pos_desired.at(BC2) =   fac * c2h * 0.3 	   + ( targetBC[2]  * degtoRad);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
//		pos_desired.at(BC3) =   fac * c1  * 0.4 	   + ( targetBC[0]  * degtoRad);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
//		pos_desired.at(BC4) =   fac * c2h * 0.3 	   + ( targetBC[1]  * degtoRad);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
//		pos_desired.at(BC5) =   fac * c2  * 0.3 	   + ( targetBC[2]  * degtoRad);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];
//
//		// Coxa-Femur Joint Position
//		pos_desired.at(CF0) =   fac * c2h * 0.55      + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
//		pos_desired.at(CF1) =  -fac * c1  * 0.5       + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
//		pos_desired.at(CF2) =  -fac * c1h * 0.55      + ( targetCF[2]  * degtoRad);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
//		pos_desired.at(CF3) =   fac * c2  * 0.55 	  + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
//		pos_desired.at(CF4) =  -fac * c1h * 0.5       + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
//		pos_desired.at(CF5) =  -fac * c1  * 0.55      + ( targetCF[2]  * degtoRad);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];
//
//		// Femur-Tibia Joint Position
//		pos_desired.at(FT0) = -fac * c2h * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//		pos_desired.at(FT1) = -fac * c1  * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//		pos_desired.at(FT2) = -fac * c1h * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
//		pos_desired.at(FT3) = -fac * c2  * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
//		pos_desired.at(FT4) = -fac * c1h * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
//		pos_desired.at(FT5) = -fac * c1  * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		cout << "set desired pose" << endl;
		//// position = oscillated signal(rad) + joint bias(rad)
		pos_desired.at(BC0) =  l *  fac * c1h * fw * 0.4 	   + ( dung_beetle_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
		pos_desired.at(BC1) =  l *  fac * c2  * bw * 0.4 	   + ( dung_beetle_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
		pos_desired.at(BC2) =  l *  fac * c2h * bw * 0.4	   + ( dung_beetle_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
		pos_desired.at(BC3) =  r *  fac * c1  * fw * 0.4 	   + ( dung_beetle_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
		pos_desired.at(BC4) =  r *  fac * c2h * bw * 0.4	   + ( dung_beetle_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
		pos_desired.at(BC5) =  r *  fac * c2  * bw * 0.4	   + ( dung_beetle_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

		// Coxa-Femur Joint Position
		pos_desired.at(CF0) =  l *  fac * c2h * bw * 0.6     + ( dung_beetle_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
		pos_desired.at(CF1) =  l * -fac * c1  * fw * 0.6      + ( dung_beetle_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
		pos_desired.at(CF2) =  l * -fac * c1h * fw * 0.6     + ( dung_beetle_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
		pos_desired.at(CF3) =  r *  fac * c2  * bw * 0.6 	   + ( dung_beetle_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
		pos_desired.at(CF4) =  r * -fac * c1h * fw * 0.6      + ( dung_beetle_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
		pos_desired.at(CF5) =  r * -fac * c1  * fw * 0.6     + ( dung_beetle_pose[4]);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

		// Femur-Tibia Joint Position
		pos_desired.at(FT0) =  l * -fac * c2h * 0.4 *-1.0 + ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
		pos_desired.at(FT1) =  l * -fac * c1  * 0.3        + ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
		pos_desired.at(FT2) =  l * -fac * c1h * 0.4        + ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
		pos_desired.at(FT3) =  r * -fac * c2  * 0.4 *-1.0 + ( dung_beetle_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		pos_desired.at(FT4) =  r * -fac * c1h * 0.3        + ( dung_beetle_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
		pos_desired.at(FT5) =  r * -fac * c1  * 0.4        + ( dung_beetle_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		pos_desired.at(LONGITUDINAL) = dung_beetle_pose.at(LONGITUDINAL);
		pos_desired.at(TRANSVERSAL) = dung_beetle_pose.at(TRANSVERSAL);
		pos_desired.at(HEAD) = dung_beetle_pose.at(HEAD);

		pos_desired = trimJointMinMax(pos_desired);
	}
	///////////////////////////////////////////

    // Advance one step 
//    if (cc % 10 == 0){
//    	CPG->step();
//    }
//    cc++;
    for(int i = 0; i<6; i++){
		MC[i]->step();
	}
    tau_openl_eff3->Step();
    tau_openl_eff4->Step();

    // Forward walking: only CPG signals needed
//    float output_cpg_0 = CPG->getCpgOutput(0);
//    float output_cpg_1 = CPG->getCpgOutput(1);

    // Get CPG Values:
    /*output_mnn_0 = CPG->getFinalNeuronOutput(10);
    output_mnn_1 = CPG->getFinalNeuronOutput(11);
    output_mnn_2 = CPG->getFinalNeuronOutput(12);
    output_mnn_3 = CPG->getFinalNeuronOutput(13);
    output_mnn_4 = CPG->getFinalNeuronOutput(4);
    output_mnn_5 = CPG->getFinalNeuronOutput(5);
    output_mnn_6 = CPG->getFinalNeuronOutput(6);
    output_mnn_7 = CPG->getFinalNeuronOutput(7);
    output_mnn_8 = CPG->getFinalNeuronOutput(6);
    output_mnn_9 = CPG->getFinalNeuronOutput(7);*/
    
    // 2. Re-scale

    // REGULAR GAIT

    // TC
    /*float TC_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, -output_cpg_0); // front TC motors
    float TC_0 = TC_0_ref + dung_beetle_pose.at(BC0);
    float TC_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.25, -0.25, -output_cpg_1); // front TC motors
    TC_1 = TC_1 + dung_beetle_pose.at(BC1); 
    float TC_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, output_cpg_1); // front TC motors
    TC_2 = TC_2 + dung_beetle_pose.at(BC2);
    float TC_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, -output_cpg_0); // front TC motors
    float TC_3 = TC_3_ref + dung_beetle_pose.at(BC3);
    float TC_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.25, -0.25, -output_cpg_1); // front TC motors
    TC_4 = TC_4 + dung_beetle_pose.at(BC4); 
    float TC_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, output_cpg_1); // front TC motors
    TC_5 = TC_5 + dung_beetle_pose.at(BC5);

    // CF
    float CF_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.4, -0.4, -output_cpg_1); // front TC motors
    float CF_0 = CF_0_ref + dung_beetle_pose.at(CF0);
    float CF_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, output_cpg_0); // front TC motors
    CF_1 = CF_1 + dung_beetle_pose.at(CF1); 
    float CF_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.35, -0.35, -output_cpg_0); // front TC motors
    CF_2 = CF_2 + dung_beetle_pose.at(CF2);
    float CF_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.4, -0.4, output_cpg_1); // front TC motors
    float CF_3 = CF_3_ref + dung_beetle_pose.at(CF3);
    float CF_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, -output_cpg_0); // front TC motors
    CF_4 = CF_4 + dung_beetle_pose.at(CF4); 
    float CF_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.35, -0.35, output_cpg_0); // front TC motors
    CF_5 = CF_5 + dung_beetle_pose.at(CF5);

    // FT
    float FT_0 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, -output_cpg_0);
    FT_0 = FT_0 + dung_beetle_pose.at(FT0);
    float FT_3 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, output_cpg_0);
    FT_3 = FT_3 + dung_beetle_pose.at(FT3);*/

    // SMALL STEP GAIT
    
    // TC
//    float TC_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, -output_cpg_0); // front TC motors
//    float TC_0 = TC_0_ref + dung_beetle_pose.at(BC0);
//    float TC_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_1); // front TC motors
//    TC_1 = TC_1 + dung_beetle_pose.at(BC1);
//    float TC_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, output_cpg_1); // front TC motors
//    TC_2 = TC_2 + dung_beetle_pose.at(BC2);
//    float TC_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, -output_cpg_0); // front TC motors
//    float TC_3 = TC_3_ref + dung_beetle_pose.at(BC3);
//    float TC_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_1); // front TC motors
//    TC_4 = TC_4 + dung_beetle_pose.at(BC4);
//    float TC_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, output_cpg_1); // front TC motors
//    TC_5 = TC_5 + dung_beetle_pose.at(BC5);
//
//    // CF
//    float CF_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, -output_cpg_1); // front TC motors
//    float CF_0 = CF_0_ref + dung_beetle_pose.at(CF0);
//    float CF_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, output_cpg_0); // front TC motors
//    CF_1 = CF_1 + dung_beetle_pose.at(CF1);
//    float CF_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.2, -0.2, -output_cpg_0); // front TC motors
//    CF_2 = CF_2 + dung_beetle_pose.at(CF2);
//    float CF_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, output_cpg_1); // front TC motors
//    float CF_3 = CF_3_ref + dung_beetle_pose.at(CF3);
//    float CF_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_0); // front TC motors
//    CF_4 = CF_4 + dung_beetle_pose.at(CF4);
//    float CF_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.2, -0.2, output_cpg_0); // front TC motors
//    CF_5 = CF_5 + dung_beetle_pose.at(CF5);

    // 3. Adaptive Motor Control

    // 3.1. Set desired positions
//    vector<float> pos_desired;
//    pos_desired.resize(21);
//    pos_desired.at(BC0) = TC_0;//dung_beetle_pose.at(BC0);//TC_0;//
//    pos_desired.at(BC1) = TC_1;//dung_beetle_pose.at(BC1);//-TC_1;//
//    pos_desired.at(BC2) = TC_2;//dung_beetle_pose.at(BC2);//-TC_2;//
//    pos_desired.at(BC3) = TC_3;//dung_beetle_pose.at(BC3);//TC_3;//
//    pos_desired.at(BC4) = TC_4;//dung_beetle_pose.at(BC4);//-TC_4;//
//    pos_desired.at(BC5) = TC_5;//dung_beetle_pose.at(BC5);//-TC_5;//
//
//    pos_desired.at(CF0) = CF_0;//dung_beetle_pose.at(CF0);//CF_0;//
//    pos_desired.at(CF1) = CF_1;//dung_beetle_pose.at(CF1);//CF_1;//
//    pos_desired.at(CF2) = CF_2;//dung_beetle_pose.at(CF2);//CF_2;//
//    pos_desired.at(CF5) = CF_5;//dung_beetle_pose.at(CF5);//CF_5;//
//    pos_desired.at(CF4) = CF_4;//dung_beetle_pose.at(CF4);//CF_4;//
//    pos_desired.at(CF3) = CF_3;//dung_beetle_pose.at(CF3);//CF_3;//
//
//    pos_desired.at(FT0) = dung_beetle_pose.at(FT0);//FT_0;//
//    pos_desired.at(FT1) = dung_beetle_pose.at(FT1);
//    pos_desired.at(FT2) = dung_beetle_pose.at(FT2);
//    pos_desired.at(FT5) = dung_beetle_pose.at(FT5);
//    pos_desired.at(FT4) = dung_beetle_pose.at(FT4);
//    pos_desired.at(FT3) = dung_beetle_pose.at(FT3);//FT_3;//
//
//    pos_desired.at(LONGITUDINAL) = dung_beetle_pose.at(LONGITUDINAL);
//    pos_desired.at(TRANSVERSAL) = dung_beetle_pose.at(TRANSVERSAL);
//    pos_desired.at(HEAD) = dung_beetle_pose.at(HEAD);


    // 3.2. Set desired velocities
    vector<float> vel_desired;
    for(size_t i=0; i<pos_desired.size(); i++)
    {
        float dx_dt = (pos_desired[i] - previous_positions[i])/dt;
        vel_desired.push_back(dx_dt);
        
        // Constant velocity
        //vel_desired.push_back(1); // rad/s
    }
    vel_desired = complianceController->lowPassFilter(0.04, vel_desired, previous_velocities);

    // 3.3. Get feedback
    vector<float> pos_feedback;
    for(int i=0; i<21; i++)
    {
        pos_feedback.push_back(jointPositions[i]);
    }
    
    vector<float> vel_feedback;
    for(int i=0; i<21; i++)
    {
        vel_feedback.push_back(jointVelocities[i]);
    }

    // 4. Calculate torques:
//    vector<float> taus = complianceController->approximateTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive Impedance PD controller
//    vector<float> taus = complianceController->porportionalTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive P controller
    //vector<float> taus = pd_c->calculateOutputTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Simple impedance PD controller
    vector<float> taus = complianceController->getWalkingTorque(pos_feedback, pos_desired, vel_feedback, vel_desired);
    vector<float> tau_ = limitTorque(taus);

    // Save data
    if(initial_counter < upper_limit)
    {
        complianceController->saveVector(pos_desired, pos_desired_csv, 1);
        complianceController->saveVector(vel_desired, vel_desired_csv, 1);
        complianceController->saveVector(tau_, torque_csv, 1);
    }

    // 5. Convert torque into current:
    vector<float> tau_ext;
    for(size_t i=0; i<tau_.size(); i++)
    {
        float amps = convertTorque2Current(tau_[i]);
        float mili_amps = amps*1000;
        tau_ext.push_back(mili_amps);
    }

    complianceController->saveVector(tau_ext, current_csv, 1);

    // 6. Set positions
    torques.at(BC0) = tau_ext.at(BC0);
    torques.at(BC1) = tau_ext.at(BC1);
    torques.at(BC2) = tau_ext.at(BC2);
    torques.at(BC3) = tau_ext.at(BC3);
    torques.at(BC4) = tau_ext.at(BC4);
    torques.at(BC5) = tau_ext.at(BC5);

    torques.at(CF0) = tau_ext.at(CF0);
    torques.at(CF1) = tau_ext.at(CF1);
    torques.at(CF2) = tau_ext.at(CF2);
    torques.at(CF5) = tau_ext.at(CF5);
    torques.at(CF4) = tau_ext.at(CF4);
    torques.at(CF3) = tau_ext.at(CF3);

    torques.at(FT0) = tau_ext.at(FT0);
    torques.at(FT1) = tau_ext.at(FT1);
    torques.at(FT2) = tau_ext.at(FT2);
    torques.at(FT5) = tau_ext.at(FT5);
    torques.at(FT4) = tau_ext.at(FT4);
    torques.at(FT3) = tau_ext.at(FT3);

    torques.at(LONGITUDINAL) = tau_ext.at(LONGITUDINAL);
    torques.at(TRANSVERSAL) = tau_ext.at(TRANSVERSAL);
    torques.at(HEAD) = tau_ext.at(HEAD);

    // 7. Set joint positions
    realRos->updateMotorState(home_names, pos_desired, vel_desired, torques);

    // 8. Update values
//    TC_0_previous = TC_0_ref;
//    TC_3_previous = TC_3_ref;
//    CF_0_previous = CF_0_ref;
//    CF_3_previous = CF_3_ref;
    previous_positions.clear();
    previous_velocities.clear(); 
    previous_positions = complianceController->copyVector(pos_desired);
    previous_velocities = complianceController->copyVector(vel_desired);
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


// POSITION CONTROLLER - Standing
void dungBeetleController::actuateRobot_standPosition(int mode)
{

//	fc_sens_raw[0] = jointTorques[13];
//	fc_sens_raw[1] = jointTorques[7];
//	fc_sens_raw[2] = jointTorques[1];
//	fc_sens_raw[3] = jointTorques[16];
//	fc_sens_raw[4] = jointTorques[10];
//	fc_sens_raw[5] = jointTorques[4];

	cout << "Standing" << endl;
//	cout << "1: " << fc_sens_raw[0] << " 2: " << fc_sens_raw[1] <<
//			" 3: " << fc_sens_raw[2] << " 4: " << fc_sens_raw[3] <<
//			" 5: " << fc_sens_raw[4] << " 6: " << fc_sens_raw[5] << endl;
	if(doMotion == 'r' or doMotion == 'R' or doMotion == 'T'){
		cout << "Actuate Rolling Posture" << endl;
		for(size_t i=0; i<dung_beetle_rolling_pose.size(); i++)
		{
			positions.at(i) = dung_beetle_rolling_pose[i];
		}
	}
	else{
		for(size_t i=0; i<dung_beetle_pose.size(); i++)
		{
			positions.at(i) = dung_beetle_pose[i];
		}
	}
    if(mode == 2){
		positions.at(BC2) = dung_beetle_pose[0]+joint_pos_hind_grab[0];
		positions.at(CF2) = dung_beetle_pose[1]+joint_pos_hind_grab[1];
		positions.at(FT2) = dung_beetle_pose[2]+joint_pos_hind_grab[2];

		positions.at(BC5) = dung_beetle_pose[3]+joint_pos_hind_grab[0];
		positions.at(CF5) = dung_beetle_pose[4]+joint_pos_hind_grab[1];
		positions.at(FT5) = dung_beetle_pose[5]+joint_pos_hind_grab[2];
    }
    
//    vector<float> err = complianceController->getPosError(positions, jointPositions); //segmented fault error

    //realRos->setLegMotorPosition(positionsNew);
	if(activate_muscle_model == true){
		// 1. Set desired positions and velocities
		vector<float> pos_desired;
		for(int i=0; i<21; i++)
		{
			//pos_desired.push_back(home_position[i]);
			pos_desired.push_back(positions[i]);
		}
		cout << "pos_desired: " << pos_desired[2] << endl;

		vector<float> vel_desired;
		for(int i=0; i<21; i++)
		{
			vel_desired.push_back(0);
		}
		cout << "vel_desired: " << vel_desired[2] << endl;

		// 2. Get feedback
		vector<float> pos_feedback;
		for(int i=0; i<21; i++)
		{
			pos_feedback.push_back(jointPositions[i]);
		}
		cout << "pos_feedback: " << pos_feedback[2] << endl;
		
		vector<float> vel_feedback;
		for(int i=0; i<21; i++)
		{
			vel_feedback.push_back(jointVelocities[i]);
		}
		cout << "vel_feedback: " << vel_feedback[2] << endl;

		// 3. Calculate torques:
		vector<float> taus = complianceController->approximateTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive Impedance PD controller
	//    vector<float> taus = complianceController->porportionalTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive P controller
	//    vector<float> taus = pd_c->calculateOutputTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Simple impedance PD controller
		vector<float> tau_ = limitTorque(taus);
		
		// 4. Convert torque into current:
		vector<float> tau_ext;
		for(size_t i=0; i<tau_.size(); i++)
		{
			float amps = convertTorque2Current(tau_[i]);
			float mili_amps = amps*1000;
			tau_ext.push_back(mili_amps);
		}
		cout << "tau_ext: " << tau_ext[2] << endl;

		// 5. Set Torques
		// if(full_robot == true){
		torques.at(BC0) = tau_ext.at(BC0);
		torques.at(BC1) = tau_ext.at(BC1);
		torques.at(BC2) = tau_ext.at(BC2);
		torques.at(BC3) = tau_ext.at(BC3);
		torques.at(BC4) = tau_ext.at(BC4);
		torques.at(BC5) = tau_ext.at(BC5);

		torques.at(CF0) = tau_ext.at(CF0);
		torques.at(CF1) = tau_ext.at(CF1);
		torques.at(CF2) = tau_ext.at(CF2);
		torques.at(CF5) = tau_ext.at(CF5);
		torques.at(CF4) = tau_ext.at(CF4);
		torques.at(CF3) = tau_ext.at(CF3);

		torques.at(FT0) = tau_ext.at(FT0);
		torques.at(FT1) = tau_ext.at(FT1);
		torques.at(FT2) = tau_ext.at(FT2);
		torques.at(FT5) = tau_ext.at(FT5);
		torques.at(FT4) = tau_ext.at(FT4);
		torques.at(FT3) = tau_ext.at(FT3);

		torques.at(LONGITUDINAL) = tau_ext.at(LONGITUDINAL);
		torques.at(TRANSVERSAL) = tau_ext.at(TRANSVERSAL);
		torques.at(HEAD) = tau_ext.at(HEAD);

		realRos->updateMotorState(home_names, pos_desired, home_velocity, torques);

		// Update values
		TC_0_previous = pos_desired.at(BC0);
		TC_3_previous = pos_desired.at(BC3);
		CF_0_previous = pos_desired.at(CF0);
		CF_3_previous = pos_desired.at(CF3);
		previous_positions.clear();
		previous_velocities.clear();
		previous_positions = complianceController->copyVector(pos_desired);
		previous_velocities = complianceController->copyVector(vel_desired);

		// save AMC command data
		if(data_counter > lower_limit && data_counter < upper_limit)
		{
			vector<float> pos_err = complianceController->pos_error;
			vector<float> vel_err = complianceController->vel_error;
			vector<float> stiff   = complianceController->current_stiffness;
			vector<float> damp    = complianceController->current_damping;
			pos_desired.insert(pos_desired.begin(), data_counter);
			pos_err.insert(pos_err.begin(), data_counter);
			vel_err.insert(vel_err.begin(), data_counter);
			vel_desired.insert(vel_desired.begin(), data_counter);
			tau_.insert(tau_.begin(), data_counter);
			tau_ext.insert(tau_ext.begin(), data_counter);
			complianceController->saveVector(pos_desired, pos_desired_csv, 1);
			complianceController->saveVector(pos_err, pos_error_csv, 1);
			complianceController->saveVector(vel_desired, vel_desired_csv, 1);
			complianceController->saveVector(vel_err, vel_error_csv, 1);
			complianceController->saveVector(tau_, torque_csv, 1);
			complianceController->saveVector(tau_ext, current_csv, 1);
		}
	
	}
	else{
		if(full_robot == true){
			realRos->updateMotorState(home_names, positions, home_velocity, home_torques);
			positions_target = positions;
		}
		else{
			cout << "set motor position" << endl;
			realRos->updateMotorState(one_leg_names, one_leg_names_test_pos, home_velocity, home_torques);
		}
		// Update
		TC_0_previous = positions.at(BC0);
		TC_3_previous = positions.at(BC3);
		CF_0_previous = positions.at(CF0);
		CF_3_previous = positions.at(CF3);
	}
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


// POSITION CONTROLLER - Walking
void dungBeetleController::actuateRobot_walkingPosition() 
{
    // 1. Modular Neural Network

	////// Implementing db_neural controller
	///////////////////////////////////////////

//	cout << "New loop" << endl;
//	cout << plotdata[0] << "  " << plotdata[1] << endl;
	// for sending to ROS topic for visualize data
	std::vector<float> plotdata;
//	cout << plotdata[0] << "  " << plotdata[1] << endl;

	// Collect Data for Logging
	cpg_signal.clear();
	pcpg_signal.clear();
	vrn_signal.clear();
	psn_signal.clear();
	ballDistanceControlData.clear();
	rollControlData.clear();
	pitchControlData.clear();
	legsearch_signal.clear();

	/////// Joystick
	if(useJoy)
	{
//		printf("joy input \n");
//		cout << " axes [0] : "  << axes[0]
//			 << " axes [1] : "  << axes[1]
//			 << " axes [2] : "  << axes[2]
//			 << " axes [3] : "  << axes[3]<< endl;
//
//		//Buttons
//		cout << " button [0] : "  << buttons[0]
//			 << " button [1] : "  << buttons[1]
//			 << " button [2] : "  << buttons[2]
//			 << " button [3] : "  << buttons[3]<< endl;
	}

	// Rolling system
	//////////////////////////////////////////////////////////////
	// IMU Sensory input signal preprocessing 
	//////////////////////////////////////////////////////////////
	omega = omega;
	accel = std::vector<float> (imuData.begin()+3, imuData.begin() + 6); // (roll, pitch, yaw) convention
	angle = angle;

	// Lowpass filter the IMU data
	for (int i=0; i<3; i++){
		angle_lowpass[i] = lowpass_input_w*angle[i] + lowpass_self_w*old_angle[i];
		old_angle[i] = angle_lowpass[i];
	}

	// Visulize angle data
	// plotdata.push_back(angle[0]);
	// plotdata.push_back(angle[1]);
	// plotdata.push_back(angle[2]);
	plotdata.push_back(angle_lowpass[0]);
	plotdata.push_back(angle_lowpass[1]);
	plotdata.push_back(angle_lowpass[2]);
	// Visulize omega data
	// plotdata.push_back(omega[0]);
	// plotdata.push_back(omega[1]);
	// plotdata.push_back(omega[2]);


	//////////////////////////////////////////////////////////////
	// Pitch control 
	//////////////////////////////////////////////////////////////
	// pitch FC Modulation
	// Sensory input
	pitch = angle_lowpass[1];
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

		// /////////////
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
		// //////////////////////////////
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
		// Integral Learner : frequency modulation from pitch error
		// if(activate_pitch_learner){
		// 	if (pitch > pitch_upper_bound or pitch < pitch_lower_bound){
		// 		cout << "Activate Integral Learner : " << endl;
		// 		pitchIntegralError += pitch_error;
		// 		slowLearnerP = (Asp*slowLearnerP) + (Bsp*pitch_error) + (Bsi*pitchIntegralError);
		// 		dualLearner = slowLearnerP*dil_gamma;
		// 		pitch_front_adapt_bias += dualLearner;
		// 		pitch_back_adapt_bias -= dualLearner;
		// 	}
		// 	else{
		// 		cout << "Integral Learner Idle" << endl;
		// 		pitchIntegralError = 0;
		// 		slowLearnerP *= 0.99;
		// 	}
		// 	cout << "pitch Front adapt_bias : " << pitch_front_adapt_bias << endl;
		// 	cout << "pitch Back  adapt_bias : " << pitch_back_adapt_bias << endl;
		// }
		// else{
		// 	pitch_front_adapt_bias = 0;
		// 	pitch_back_adapt_bias  = 0;

		// }
		// Leg Amplitude Modulation
		pitch_front_amp_gain = (0.5*pitch_error)/pitch_range + pitch_front_adapt_bias;
		pitch_front_amp_gain = limitRangeFunction(pitch_front_amp_gain, 0.0, 1);
		
		// pitch_front_amp_gain = 1.0;

		pitch_back_amp_gain = (-0.5*pitch_error)/pitch_range + pitch_back_adapt_bias;
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
	roll_cpg_mod_l = relu(-angle_lowpass[0]-10);
	roll_cpg_mod_r = relu( angle_lowpass[0]-10);
	roll_mod_l_old = roll_cpg_mod_l;
	roll_mod_r_old = roll_cpg_mod_r;
	// roll_cpg_mod_l = sigmoid(omega[0],  1,  5, 5);
	// roll_cpg_mod_r = sigmoid(omega[0], -1, -5, 5);
	// roll_cpg_mod_l = 0.0;
	// roll_cpg_mod_r = 0.0;
	cout << "roll_cpg_mod Left  : " << roll_cpg_mod_l << endl;
	cout << "roll_cpg_mod Right : " << roll_cpg_mod_r << endl;

	//////////////////////////////////////////////////////////////


	//Walking speed
	joySpeed = axes[1];
    // CPG->setMI(0.03); //walking MI=0.05,  //0.00001
    // if(joySpeed > 0.2){
    // 	CPG->setMI(joySpeed/20);
    // }
	if (doMotion == 'R' or doMotion == 'T' or doMotion == 'Y'){
		for(const int i : {1,2, 4,5}){
			MC[i]->setMI(back_MI);
		}
		for(const int i : {0,3}){
			MC[i]->setMI(front_MI);
			// cout << "MC" << i << " MI : " << MC[i]->getMI() << endl;
		}
	}
	else{
		for(int i=0; i<6; i++){
			MC[i]->setMI(default_MI);
		}
	}
	cout << "set MI" << endl;


	// Turning direction factor
	joyTurn = axes[3];
	if (abs(joyTurn) > 0){
		if (joyTurn > 0){
			l = 1 - joyTurn;
			r = 1;
		}
		else if (joyTurn < 0){
			l = 1;
			r = 1 + joyTurn;
		}
	}
	else if (doMotion == 'a'){
		l = 0.5;
		r = 1.0;
	}
	else if (doMotion == 'd'){
		l = 1.0;
		r = 0.5;
	}
	int backward = buttons[1];
	///////////////////////

	// set PSN parameter for forward walking
	for (int i=0; i<6; i++){
		MC[i]->setPsnInputNeurons(0,0);
	}

//    float c1  = MC1->getpmnOutput(3);
//    float c1h = MC1->getpmnOutput(0);
////            float cpg = MC1->getCpgOutput(0);
//
//    float c2  = MC1->getpmnOutput(2);
//	float c2h = MC1->getpmnOutput(1);

	// signal from motor neurons
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
	cout << "get pmn Output" << endl;

//	std::cout << "down0 : " << down0 << std::endl;
//	    std::cout << "down : " << vrn3->getOutput(0) << std::endl;

//	tau_downsignal0->Write(down0);
//	tau_downsignal1->Write(down1);
//	tau_downsignal2->Write(down2);
//	tau_downsignal3->Write(down3);
//	tau_downsignal4->Write(down4);
//	tau_downsignal5->Write(down5);
//
//	int tau_down = 3;
//	down0 = tau_downsignal0->Read(tau_down);
//	down1 = tau_downsignal1->Read(tau_down);
//	down2 = tau_downsignal2->Read(tau_down);
//	down3 = tau_downsignal3->Read(tau_down);
//	down4 = tau_downsignal4->Read(tau_down);
//	down5 = tau_downsignal5->Read(tau_down);

//	float c1  = CPG->getFinalNeuronOutput(0);
//	float c1h = CPG->getFinalNeuronOutput(1);
//
//	float c2  = CPG->getFinalNeuronOutput(2);
//	float c2h = CPG->getFinalNeuronOutput(3);


//	cout << c1 << "\n";
//	plotdata.push_back(c2);
//	cout << plotdata[0] << endl;

//	tauCF[0].Write(c2);
//	tauCF[1].Write(c2h);
//	c2 = tauCF[0].Read(30);
//	c2h = tauCF[1].Read(30);

	//amplitude for the signal for group of leg
	fac = 1.5; // 0.6
	fw   = 1.0;
	bw   = 1.0;
	l = 1.0; // left leg
	r = 1.0; // right leg
//	bl = 1.0; 
//	plotdata.push_back(c2);
//	cout << plotdata[0] << "  " << plotdata[1] << endl;


	///////// Read open loop Efference copy ////////
	for (int i=0; i<6; i++){
		eff[i] = MC[i]->getVrnOutput3(6);
		tauCF[i].Write(eff[i]);
		openlsignal[i] = tauCF[i].Read(tauEff_JointFeedback);
	}
	// cout << "Read open loop Efference copy" << endl;

//	std::cout << "eff_vrn3 : " << CPG->getVrnOutput3(6) << "  " << eff_vrn3 << "  " << eff_vrn3_delay << std::endl;

	if(activate_rolling_grab){
//		fcphase = {c2h, 1, 1, c2, 1, 1};
		cout << "activate grab" << endl;
	}
	else{

		for (int i = 0; i < 6; i++){
			if (openlsignal[i] > old_openlsignal[i]){
				fcphase[i] = 0;
			}
			else{
				fcphase[i] = 1;
			}
		}

		// legsearch_signal.push_back(data_counter);
		// legsearch_signal.push_back(objective);
		// legsearch_signal.push_back(FC_state);
		// legsearch_signal.push_back(openlsignal.at(0));
		// legsearch_signal.push_back(openlsignal.at(1));
		// legsearch_signal.push_back(openlsignal.at(2));
		// legsearch_signal.push_back(openlsignal.at(3));
		// legsearch_signal.push_back(openlsignal.at(4));
		// legsearch_signal.push_back(openlsignal.at(5));
		// legsearch_signal.push_back(old_openlsignal.at(0));
		// legsearch_signal.push_back(old_openlsignal.at(1));
		// legsearch_signal.push_back(old_openlsignal.at(2));
		// legsearch_signal.push_back(old_openlsignal.at(3));
		// legsearch_signal.push_back(old_openlsignal.at(4));
		// legsearch_signal.push_back(old_openlsignal.at(5));
		old_openlsignal = openlsignal;
	}

	///////// Read open loop Efference copy ////////

	///////// Leg state neuron ////////////////////


	///////////////////////////////////////////////

	/////////  Read Sensor data ///////////////////
	// Read torque from motors
	sw_sens_raw[0] = jointTorques[12]; //BC0-BC5
	sw_sens_raw[1] = jointTorques[6];
	sw_sens_raw[2] = jointTorques[0];
	sw_sens_raw[3] = jointTorques[15];
	sw_sens_raw[4] = jointTorques[9];
	sw_sens_raw[5] = jointTorques[3];

	fc_sens_raw[0] = jointTorques[13]; // CF0-CF5
	fc_sens_raw[1] = jointTorques[7];
	fc_sens_raw[2] = jointTorques[1];
	fc_sens_raw[3] = jointTorques[16];
	fc_sens_raw[4] = jointTorques[10];
	fc_sens_raw[5] = jointTorques[4];

	FT_torque[0] = jointTorques[14]; // FT0-FT5
	FT_torque[1] = jointTorques[8];
	FT_torque[2] = jointTorques[2];
	FT_torque[3] = jointTorques[17];
	FT_torque[4] = jointTorques[11];
	FT_torque[5] = jointTorques[5];

	// normalize torque signal (reading from current of dynamixel motor)
	for(int i = 0; i < 6; i++){
		sw_sens_raw[i] /= 1000.0;
		fc_sens_raw[i] /= 1000.0;
		FT_torque  [i] /= 1000.0;

		sw_sens_raw[i] = input_w*sw_sens_raw[i] + self_w*old_sw_sens_raw[i];
		fc_sens_raw[i] = input_w*fc_sens_raw[i] + self_w*old_fc_sens_raw[i];
		FT_torque  [i] = input_w*FT_torque  [i] + self_w*old_FT_torque  [i];
	}
	// store past signal (t-1 timestep)
	old_sw_sens_raw = sw_sens_raw;
	old_fc_sens_raw = fc_sens_raw;
	old_FT_torque = FT_torque;
	cout << "Read Sensor Data" << endl;

	/////////  Read Sensor data ///////////////////

	//////////////////////////////////////////////////////////////
	// Ball Control (Robot Ball distance Control) 
	//////////////////////////////////////////////////////////////
	vector<float> current_positions = jointPositions;
	// target_robot_ball_distance = -0.40;
	represent_joint_pos_ball_contact_sens = 0.7*0.25*(joint_pos_ball_contact_sens[1]+joint_pos_ball_contact_sens[4]
												+joint_pos_ball_contact_sens[7]+joint_pos_ball_contact_sens[10])
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
				fc_sens[i] = 0;

				fc_error_st[i] = 0;
				oldfc_error_st[i] = 0;

				swingToStance[i] = false;
//					cout << " stanceToSwing" << endl;
				stanceToSwing[i] = true;

				// Contact Manipulation
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
				LS_act_old[i] = 0;
				LS_out[i] = 0;
				LS_out_old[i] = 0;
				sw_activation = false;
//				if(max_sw_error[i] > 0.4){
//					cout << "Swing error --> FC_error" << endl;
//					fc_error_st[i] = -max_sw_error[i]*0.5;
//					oldfc_error_st[i] = -oldsw_error[i]*0.5;
//				}
//				else{
				fc_error_st[i] = max_fc_error[i];
				oldfc_error_st[i] = max_fc_error[i];
//				}
				continue;
			}
		}

//	}
	/////////StanceToSwing and swingToStance setup/////////


	////////   Stance Phase Foot Extend System ////////////////
	//// threshold foot contact sensor signal /////
//	for(int i = 0; i < 6; i++){
		if (fc_sens_raw[i] > fc_threshold){
			fc_sens[i] = 1;
		}
		else {
			fc_sens[i] = 0;
		}

		if(fcphase[i] == 1){

			if(true or activate_fc_closeloop or activate_rolling_grab){
				//// calculate error ////
				fc_error_st[i] = fc_selfw * oldfc_error_st[i] +
							(1.0-fc_selfw) * (fcphase[i] - fc_sens[i]);
				// if (fc_sens[i] == 0){
				// 	fc_error_st[i] = fc_selfw * oldfc_error_st[i] +
				// 				(1.0-fc_selfw) * (fcphase[i] - fc_sens[i]);
				// 	// // Contact Manipulation
				// 	// if(doMotion=='R'){
				// 	// 	// ball_push_sig[i] -= 0.01;
				// 	// 	// ground_push_sig[i] -= 0.01;
				// 	// 	ball_push_sig[i] *= 0.98;
				// 	// 	ground_push_sig[i] *= 0.98;
				// 	// 	ball_push_sig[i] = lowerLimitFunction(ball_push_sig[i], 0.0);
				// 	// 	ground_push_sig[i] = lowerLimitFunction(ball_push_sig[i], 0.0);
				// 	// }
				// }
				// else if (fc_sens[i] == 1){
				// 	// dynamic of st_error from simulation
				// 	// fc_error_st[i] -= 0.01;
				// 	// fc_error_st[i] = lowerLimitFunction(fc_error_st[i], 0.0);
				// 	// // Contact Manipulation
				// 	// if(doMotion=='R'){
				// 	// 	// ball_push_sig[i] = tanh(0.05 * fc_sens[i]+ 0.95 * old_ball_push_sig[i]);
				// 	// 	// ground_push_sig[i] = tanh(0.05 * fc_sens[i]+ 0.95 * old_ground_push_sig[i]);
				// 	// 	ball_push_sig[i] = tanh(0.1 * ball_push_input+ 0.9 * old_ball_push_sig[i]);
				// 	// 	ground_push_sig[i] = tanh(0.05 * ball_push_input+ 0.95 * old_ground_push_sig[i]);
				// 	// }
				// 	// joint_pos_ball_contact_sens[i] = current_positions[i+6];
				// 	// // cout << "Collect joint_pos_ball_contact_sens Data !!!" << endl;
				// 	// // cout << "current_positions " << i << " : " << current_positions[i+6] << endl;

				// }

				//// Memory term for max value of leg Extend with decay ////////
				//// for extend in the begin of stance phase /////
				if(max_fc_error[i] < fc_error_st[i]){
					max_fc_error[i] = fc_error_st[i];
				}
				max_fc_error[i] *= 0.95;
			}
			////////////////////////////////////////////////
		}
//	}
//	oldfc_error_st = fc_error_st;
	////////   Stance Phase Foot Extend System ////////////////


	////////   Swing Phase Foot Retract System for Obstacle ////////////////
//	for(int i = 0; i < 6; i++){

		if(fcphase[i] == 0){
//			cout << "Leg " << i << " swing" << endl;
			/////// Threshold BC joint Torque ///////
			if (doMotion == 'b'){
				if (-sw_sens_raw[i] > sw_threshold[i]){
					sw_sens[i] = 1;
				}
				else {
					sw_sens[i] = 0;
				}
			}
			else{
				if (sw_sens_raw[i] > sw_threshold[i]){
					sw_sens[i] = 1;
				}
				else {
					sw_sens[i] = 0;
				}
			}
			///////// Leg state neuron ////////////////////
			// LS_act[i] = LS_input_w * (1-fcphase[i]) - (CF_tq_w * fc_sens[i]) + (LS_self_w * LS_act_old[i]);
			LS_act[i] = LS_input_w * (1-fc_sens[i]) + (LS_self_w * LS_out[i]);
			LS_out[i] = tanh(LS_act[i]);
			LS_out_old[i] = LS_out[i];
			///////////////////////////////////////////////
			if (LS_out[i] > 0.9 or sw_activation){
				sw_activation = true;
				if(true or activate_sw_avoid){

					//// calculate error
					sw_error[i] = sw_selfw * oldsw_error[i] +
							(1.0-sw_selfw) * sw_sens[i];

					//// Memory term for max value of leg Extend with decay ////////
					//// for extend in the begin of stance phase /////
					// if(sw_error[i] > max_sw_error[i]){
					// 	max_sw_error[i] = sw_error[i];
					// }
					// max_sw_error[i] *= 0.95;
					
					sw_error_mem[i] += sw_error[i]/50; // default
					// sw_error_mem[i] += sw_error[i]/50; // gamma = 0.01, 0.02

					//// activate spiking error when sw_error exceed threshold
					// if(sw_error[i] > sw_err_threshold and activate_sw_leg_avoid[i] == 0){
					// 	activate_sw_leg_avoid[i] = 1;-
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
				if(activate_sw_switch == true ){
					if (FT_torque[i] > sw_switch_threshold[i]){
						sw_switch_sens[i] = 1;
					}
					else {
						sw_switch_sens[i] = 0;
					}
					//// calculate error
					sw_switch_err[i] = sw_selfw * oldsw_switch_err[i] +
							(1.0-sw_selfw) * sw_switch_sens[i];

					//////
					if(sw_switch_err[i] > sw_switch_err_threshold){
						sw_switch_spike[i] = 2.0;
						cout << "FT Movement Switch Leg" << i << endl;
					}
				}
			}
			//////////////////////////////////////////
		}
	}
	oldfc_error_st = fc_error_st;
	oldsw_switch_err = sw_switch_err;
	oldsw_error = sw_error;
	sw_err_fac = sw_error_mem;
	cout << "process Local Leg Control" << endl;
	
	plotdata.push_back(eff[4]);
	plotdata.push_back(openlsignal[4]);
	plotdata.push_back(fcphase[4]);
	// plotdata.push_back(sw_error_mem[4]);

//	cout << "Sw Sens Raw BC" << sw_sens_raw[0] << endl;
//	cout << "SW sens          " << sw_sens[0] << endl;
//	cout << "SW Error              " << sw_error[0] << endl;
//	cout << "activate leg 0              " << activate_sw_leg_avoid[0] << endl;

//	cout << "Sw switch Raw FT      " << FT_torque[3] << endl;
//	cout << "SW switch          " << sw_switch_sens[3] << endl;
//	cout << "SW Error    " << sw_switch_err[3] << endl;
//	cout << "SW switch Fac    " << sw_switch_fac[3] << endl;
//	cout << fcphase[5] << endl;
//	cout << fc_error_st[5] << endl;


	////////   Swing Phase Foot Retract System for Obstacle ////////////////

	// setup parameter for each behaviour
	if (backward or doMotion == 'b' or doMotion == 'g')
	{
		cout << "Backward" << endl;
//		float temp = c1;
//		c1 = c2;
//		c2 = temp;
//		float temph = c1h;
//		c1h = c2h;
//		c2h = temph;
//		fac = -0.6;
//		fw = 1.0;
//		bw = 1.7;
		for (int i=0; i<6; i++){
			MC[i]->setPsnInputNeurons(0,1);
		}
	}
	else if (doMotion == 'q' or doMotion == 'z'){
		cout << "Left Turn" << endl;
		for (int i=0; i<3; i++){
			MC[i]->setPsnInputNeurons(0,1);
		}
//		l = -1.0;
	}
	else if (doMotion == 'e'){
		cout << "Right Turn" << endl;
		for (int i=3; i<6; i++){
			MC[i]->setPsnInputNeurons(0,1);
		}

//		r = -1.0;
	}
	else if (doMotion == 'a'){
		cout << "Left Curve" << endl;
		l = 0.5;
		r = 1.0;
	}
	else if (doMotion == 'd'){
		cout << "Right Curve" << endl;
		l = 1.0;
		r = 0.5;
	}

	// past denmark version ALPHA Configuration Testing (Have not been tested yet)
	if (isDenmark_db_alpha == true){
		//// position = factor amplitude * oscillated signal(rad) + joint bias(rad) * conversion factor degree to radian
//		positions.at(BC0) =  -fac * c1h * 0.4 	   + ( targetBCl[0]  * degtoRad);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
//		positions.at(BC1) =  -fac * c2  * 0.3 	   + ( targetBCl[1]  * degtoRad);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
//		positions.at(BC2) =  -fac * c2h * 0.3 	   + ( targetBCl[2]  * degtoRad);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
//		positions.at(BC3) =   fac * c1  * 0.4 	   + ( targetBCr[0]  * degtoRad);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
//		positions.at(BC4) =   fac * c2h * 0.3 	   + ( targetBCr[1]  * degtoRad);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
//		positions.at(BC5) =   fac * c2  * 0.3 	   + ( targetBCr[2]  * degtoRad);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];
//
//		// Coxa-Femur Joint Position
//		positions.at(CF0) =   fac * c2h * 0.55     + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
//		positions.at(CF1) =  -fac * c1  * 0.5      + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
//		positions.at(CF2) =  -fac * c1h * 0.55     + ( targetCF[2]  * degtoRad);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
//		positions.at(CF3) =   fac * c2  * 0.55 	   + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
//		positions.at(CF4) =  -fac * c1h * 0.5      + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
//		positions.at(CF5) =  -fac * c1  * 0.55     + ( targetCF[2]  * degtoRad);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];
//
//		// Femur-Tibia Joint Position
//		positions.at(FT0) =  fac * c2h * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//		positions.at(FT1) =  fac * c1  * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//		positions.at(FT2) =  fac * c1h * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
//		positions.at(FT3) =  fac * c2  * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
//		positions.at(FT4) =  fac * c1h * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
//		positions.at(FT5) =  fac * c1  * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
//
//		positions.at(LONGITUDINAL) = dung_beetle_pose.at(LONGITUDINAL);
//		positions.at(TRANSVERSAL) = dung_beetle_pose.at(TRANSVERSAL);
//		positions.at(HEAD) = dung_beetle_pose.at(HEAD);
	}
	else if (doMotion == 'R' or doMotion == 'T' or doMotion == 'Y'){ // rolling
		cout << doMotion << endl;

		MC[0]->setPsnInputNeurons(0,1);
		MC[3]->setPsnInputNeurons(0,1);

		//// position = oscillated signal(rad) + joint bias(rad)
		// fac = 1.0;
		fac = 0.5; // low frequency: MI = 0.05
		float tmp_back_fac = 0.1;
		// float posture_bias = (pitch_error)*degtoRad;
		float posture_bias = 0.0;

		if (abs(angle_lowpass[0]) < 10){

			// middle & hind legs movement
			positions.at(BC1) =  l  * down1 * bw * BC_Roll_fac[1] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[6] - posture_bias);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
			positions.at(BC2) =  l  * down2 * bw * BC_Roll_fac[2] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[0] - posture_bias);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
			positions.at(BC4) =  r  * down4 * bw * BC_Roll_fac[1] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[9] - posture_bias);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
			positions.at(BC5) =  r  * down5 * bw * BC_Roll_fac[2] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[3] - posture_bias);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

			// Coxa-Femur Joint Position
			positions.at(CF1) =  l * -fac * lift1 * fw * CF_Roll_fac[1]  + ( dung_beetle_rolling_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
			positions.at(CF2) =  l * -fac * lift2 * fw * CF_Roll_fac[2]  + ( dung_beetle_rolling_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
			positions.at(CF4) =  r * -fac * lift4 * fw * CF_Roll_fac[1]  + ( dung_beetle_rolling_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
			positions.at(CF5) =  r * -fac * lift5 * fw * CF_Roll_fac[2]  + ( dung_beetle_rolling_pose[4]);//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

			// Femur-Tibia Joint Position
			positions.at(FT1) =  l *  fac * lift1 * FT_Roll_fac[1] 	  + ( dung_beetle_rolling_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
			positions.at(FT2) =  l * -fac * lift2 * FT_Roll_fac[2] 	  + ( dung_beetle_rolling_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
			positions.at(FT4) =  r *  fac * lift4 * FT_Roll_fac[1] 	  + ( dung_beetle_rolling_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
			positions.at(FT5) =  r * -fac * lift5 * FT_Roll_fac[2] 	  + ( dung_beetle_rolling_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

			// // front leg backwards movement
			positions.at(BC0) =  l *  fac * lift0 * fw * BC_Roll_fac[0] * pitch_front_amp_gain 	+ ( dung_beetle_rolling_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
			positions.at(BC3) =  r *  fac * lift3 * fw * BC_Roll_fac[0] * pitch_front_amp_gain    + ( dung_beetle_rolling_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];

			positions.at(CF0) =  l *  fac * down0 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
			positions.at(CF3) =  r *  fac * down3 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];

			positions.at(FT0) =  l *  fac * down0 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
			positions.at(FT3) =  r *  fac * down3 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		}
		else{
			//  Fix legs
			// Front Legs
			positions.at(BC0) =  l *  fac * lift0 * fw * BC_Roll_fac[0] * pitch_front_amp_gain 	+ ( dung_beetle_rolling_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
			positions.at(BC3) =  r *  fac * lift3 * fw * BC_Roll_fac[0] * pitch_front_amp_gain    + ( dung_beetle_rolling_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];

			positions.at(CF0) =  l *  fac * down0 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
			positions.at(CF3) =  r *  fac * down3 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];

			positions.at(FT0) =  l *  fac * down0 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
			positions.at(FT3) =  r *  fac * down3 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
			// positions.at(BC0) =  ( dung_beetle_rolling_pose[12]);
			// positions.at(BC3) =  ( dung_beetle_rolling_pose[15]);
			// positions.at(CF0) =  ( dung_beetle_rolling_pose[13]);
			// positions.at(CF3) =  ( dung_beetle_rolling_pose[16]);
			// positions.at(FT0) =  ( dung_beetle_rolling_pose[14]);
			// positions.at(FT3) =  ( dung_beetle_rolling_pose[17]);
			// BC joints
			// Hind legs
			positions.at(BC1) =  ( dung_beetle_rolling_pose[6]);
			positions.at(BC2) =  ( dung_beetle_rolling_pose[0]);
			positions.at(BC4) =  ( dung_beetle_rolling_pose[9]);
			positions.at(BC5) =  ( dung_beetle_rolling_pose[3]);
			// // Coxa-Femur Joint Position
			positions.at(CF1) =  ( dung_beetle_rolling_pose[7]);
			positions.at(CF2) =  ( dung_beetle_rolling_pose[1]);
			positions.at(CF4) =  ( dung_beetle_rolling_pose[10]);
			positions.at(CF5) =  ( dung_beetle_rolling_pose[4]);
			// // Femur-Tibia Joint Position
			positions.at(FT1) =  ( dung_beetle_rolling_pose[8]);
			positions.at(FT2) =  ( dung_beetle_rolling_pose[2]);
			positions.at(FT4) =  ( dung_beetle_rolling_pose[11]);
			positions.at(FT5) =  ( dung_beetle_rolling_pose[5]);

		}
		/////////////////////////////////////////////////////////////
		// Test Low frequency ///////////////////////
		////////////////////////////////////////////////////////

		// if (abs(angle[0]) < 10){

		// 	// middle & hind legs movement
		// 	positions.at(BC1) =  l  * down1 * bw * BC_Roll_fac[1] * pitch_back_amp_gain*0.5  + ( dung_beetle_rolling_pose[6] - posture_bias);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
		// 	positions.at(BC2) =  l  * down2 * bw * BC_Roll_fac[2] * pitch_back_amp_gain*0.5  + ( dung_beetle_rolling_pose[0] - posture_bias);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
		// 	positions.at(BC4) =  r  * down4 * bw * BC_Roll_fac[1] * pitch_back_amp_gain*0.5  + ( dung_beetle_rolling_pose[9] - posture_bias);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
		// 	positions.at(BC5) =  r  * down5 * bw * BC_Roll_fac[2] * pitch_back_amp_gain*0.5  + ( dung_beetle_rolling_pose[3] - posture_bias);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

		// 	// Coxa-Femur Joint Position
		// 	positions.at(CF1) =  l * -fac * lift1 * fw * CF_Roll_fac[1]*0.5  + ( dung_beetle_rolling_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
		// 	positions.at(CF2) =  l * -fac * lift2 * fw * CF_Roll_fac[2]*0.5  + ( dung_beetle_rolling_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
		// 	positions.at(CF4) =  r * -fac * lift4 * fw * CF_Roll_fac[1]*0.5  + ( dung_beetle_rolling_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
		// 	positions.at(CF5) =  r * -fac * lift5 * fw * CF_Roll_fac[2]*0.5  + ( dung_beetle_rolling_pose[4]);//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

		// 	// Femur-Tibia Joint Position
		// 	positions.at(FT1) =  l *  fac * lift1 * FT_Roll_fac[1] 	  + ( dung_beetle_rolling_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
		// 	positions.at(FT2) =  l * -fac * lift2 * FT_Roll_fac[2] 	  + ( dung_beetle_rolling_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
		// 	positions.at(FT4) =  r *  fac * lift4 * FT_Roll_fac[1] 	  + ( dung_beetle_rolling_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
		// 	positions.at(FT5) =  r * -fac * lift5 * FT_Roll_fac[2] 	  + ( dung_beetle_rolling_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		// 	// // front leg backwards movement
		// 	positions.at(BC0) =  l *  fac * lift0 * fw * BC_Roll_fac[0] * pitch_front_amp_gain 	+ ( dung_beetle_rolling_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
		// 	positions.at(BC3) =  r *  fac * lift3 * fw * BC_Roll_fac[0] * pitch_front_amp_gain    + ( dung_beetle_rolling_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];

		// 	positions.at(CF0) =  l *  fac * down0 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
		// 	positions.at(CF3) =  r *  fac * down3 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];

		// 	positions.at(FT0) =  l *  fac * down0 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
		// 	positions.at(FT3) =  r *  fac * down3 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		// }
		// else{
		// 	//  Fix legs
		// 	// Front Legs
		// 	positions.at(BC0) =  l *  fac * lift0 * fw * BC_Roll_fac[0] * pitch_front_amp_gain 	+ ( dung_beetle_rolling_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
		// 	positions.at(BC3) =  r *  fac * lift3 * fw * BC_Roll_fac[0] * pitch_front_amp_gain    + ( dung_beetle_rolling_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];

		// 	positions.at(CF0) =  l *  fac * down0 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
		// 	positions.at(CF3) =  r *  fac * down3 * bw * CF_Roll_fac[0]*1.0   + ( dung_beetle_rolling_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];

		// 	positions.at(FT0) =  l *  fac * down0 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
		// 	positions.at(FT3) =  r *  fac * down3 * FT_Roll_fac[0]*1.0   		+ ( dung_beetle_rolling_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		// positions.at(BC0) =  ( dung_beetle_rolling_pose[12]);
		// positions.at(BC3) =  ( dung_beetle_rolling_pose[15]);
		// positions.at(CF0) =  ( dung_beetle_rolling_pose[13]);
		// positions.at(CF3) =  ( dung_beetle_rolling_pose[16]);
		// positions.at(FT0) =  ( dung_beetle_rolling_pose[14]);
		// positions.at(FT3) =  ( dung_beetle_rolling_pose[17]);
		// // BC joints
		// // Hind legs
		// 	positions.at(BC1) =  ( dung_beetle_rolling_pose[6]);
		// 	positions.at(BC2) =  ( dung_beetle_rolling_pose[0]);
		// 	positions.at(BC4) =  ( dung_beetle_rolling_pose[9]);
		// 	positions.at(BC5) =  ( dung_beetle_rolling_pose[3]);
		// 	// // Coxa-Femur Joint Position
		// 	positions.at(CF1) =  ( dung_beetle_rolling_pose[7]);
		// 	positions.at(CF2) =  ( dung_beetle_rolling_pose[1]);
		// 	positions.at(CF4) =  ( dung_beetle_rolling_pose[10]);
		// 	positions.at(CF5) =  ( dung_beetle_rolling_pose[4]);
		// 	// // Femur-Tibia Joint Position
		// 	positions.at(FT1) =  ( dung_beetle_rolling_pose[8]);
		// 	positions.at(FT2) =  ( dung_beetle_rolling_pose[2]);
		// 	positions.at(FT4) =  ( dung_beetle_rolling_pose[11]);
		// 	positions.at(FT5) =  ( dung_beetle_rolling_pose[5]);

		// }
		// End test low frequencu
		//////////////////////////////////////////////////////////////////////////


		//  Fix legs
		// Front Legs
		// positions.at(BC0) =  ( dung_beetle_rolling_pose[12]);
		// positions.at(BC3) =  ( dung_beetle_rolling_pose[15]);
		// positions.at(CF0) =  ( dung_beetle_rolling_pose[13]);
		// positions.at(CF3) =  ( dung_beetle_rolling_pose[16]);
		// positions.at(FT0) =  ( dung_beetle_rolling_pose[14]);
		// positions.at(FT3) =  ( dung_beetle_rolling_pose[17]);
		// BC joints
		// // Hind legs
		// positions.at(BC1) =  ( dung_beetle_rolling_pose[6]);
		// positions.at(BC2) =  ( dung_beetle_rolling_pose[0]);
		// positions.at(BC4) =  ( dung_beetle_rolling_pose[9]);
		// positions.at(BC5) =  ( dung_beetle_rolling_pose[3]);
		// // // Coxa-Femur Joint Position
		// positions.at(CF1) =  ( dung_beetle_rolling_pose[7]);
		// positions.at(CF2) =  ( dung_beetle_rolling_pose[1]);
		// positions.at(CF4) =  ( dung_beetle_rolling_pose[10]);
		// positions.at(CF5) =  ( dung_beetle_rolling_pose[4]);
		// // // Femur-Tibia Joint Position
		// positions.at(FT1) =  ( dung_beetle_rolling_pose[8]);
		// positions.at(FT2) =  ( dung_beetle_rolling_pose[2]);
		// positions.at(FT4) =  ( dung_beetle_rolling_pose[11]);
		// positions.at(FT5) =  ( dung_beetle_rolling_pose[5]);

		if (doMotion == 'T'){
		positions.at(BC1) =  l  * down1 * bw * BC_Roll_fac[1] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
			positions.at(BC2) =  l  * down2 * bw * BC_Roll_fac[2] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
			positions.at(BC4) =  r  * down4 * bw * BC_Roll_fac[1] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
			positions.at(BC5) =  r  * down5 * bw * BC_Roll_fac[2] * pitch_back_amp_gain  + ( dung_beetle_rolling_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];
		}
		if (doMotion == 'Y'){
			positions.at(BC0) =  l *  0.2 * lift0 * fw * BC_Roll_fac[0] * pitch_front_amp_gain 	+ ( dung_beetle_rolling_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
			positions.at(BC3) =  r *  0.2 * lift3 * fw * BC_Roll_fac[0] * pitch_front_amp_gain    + ( dung_beetle_rolling_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];

			positions.at(FT0) =  l *  fac * down0 * FT_Roll_fac[0]*2.0   		+ ( dung_beetle_rolling_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
			positions.at(FT3) =  r *  fac * down3 * FT_Roll_fac[0]*-2.0   		+ ( dung_beetle_rolling_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
			
			// Hind legs
			// positions.at(BC1) =  ( dung_beetle_rolling_pose[6]);
			// positions.at(BC2) =  ( dung_beetle_rolling_pose[0]);
			// positions.at(BC4) =  ( dung_beetle_rolling_pose[9]);
			// positions.at(BC5) =  ( dung_beetle_rolling_pose[3]);
			// // // Coxa-Femur Joint Position
			// positions.at(CF1) =  ( dung_beetle_rolling_pose[7]);
			// positions.at(CF2) =  ( dung_beetle_rolling_pose[1]);
			// positions.at(CF4) =  ( dung_beetle_rolling_pose[10]);
			// positions.at(CF5) =  ( dung_beetle_rolling_pose[4]);
			// // // Femur-Tibia Joint Position
			// positions.at(FT1) =  ( dung_beetle_rolling_pose[8]);
			// positions.at(FT2) =  ( dung_beetle_rolling_pose[2]);
			// positions.at(FT4) =  ( dung_beetle_rolling_pose[11]);
			// positions.at(FT5) =  ( dung_beetle_rolling_pose[5]);

		}
		//// position = oscillated signal(rad) + joint bias(rad)
		//// Rolling walk
//		positions.at(BC1) =  l *  fac * c2  * bw * 0.4 	   + ( dung_beetle_rolling_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
//		positions.at(BC2) =  l *  fac * c2h * bw * 0.4	   + ( dung_beetle_rolling_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
//		positions.at(BC4) =  r *  fac * c2h * bw * 0.4	   + ( dung_beetle_rolling_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
//		positions.at(BC5) =  r *  fac * c2  * bw * 0.4	   + ( dung_beetle_rolling_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];
//
//		// Coxa-Femur Joint Position
//		positions.at(CF1) =  l * -fac * c1  * fw * 0.7     + ( dung_beetle_rolling_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
//		positions.at(CF2) =  l * -fac * c1h * fw * 0.55     + ( dung_beetle_rolling_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
//		positions.at(CF4) =  r * -fac * c1h * fw * 0.7     + ( dung_beetle_rolling_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
//		positions.at(CF5) =  r * -fac * c1  * fw * 0.55     + ( dung_beetle_rolling_pose[4]);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];
//
//		// Femur-Tibia Joint Position
//		positions.at(FT1) =  l * -fac * c1  * 1.2        + ( dung_beetle_rolling_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//		positions.at(FT2) =  l * -fac * c1h * 1.2        + ( dung_beetle_rolling_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
//		positions.at(FT4) =  r * -fac * c1h * 1.2        + ( dung_beetle_rolling_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
//		positions.at(FT5) =  r * -fac * c1  * 1.2        + ( dung_beetle_rolling_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];


//		float temp = c1;
//		c1 = c2;
//		c2 = temp;
//		float temph = c1h;
//		c1h = c2h;
//		c2h = temph;
//		fac = -0.4;

		// normal pose
		float target_TA0 = dung_beetle_rolling_pose.at(LONGITUDINAL);
		float target_TA1 = dung_beetle_rolling_pose.at(TRANSVERSAL);
		// adaptive pose
		// posture_bias = (pitch_error)*degtoRad;
		// float target_TA0 = dung_beetle_rolling_pose.at(LONGITUDINAL) - posture_bias;
		// float target_TA1 = dung_beetle_rolling_pose.at(TRANSVERSAL)  - posture_bias;
		positions.at(LONGITUDINAL) = target_TA0;
		positions.at(TRANSVERSAL) = target_TA1;
		positions.at(HEAD) = dung_beetle_rolling_pose.at(HEAD);

	}
	else if (doMotion == 'z'){ // turning on ball
		cout << doMotion << endl;

		//// position = oscillated signal(rad) + joint bias(rad)
//		positions.at(BC0) =  l *  fac * c1h * fw * 0.6 	   + ( dung_beetle_ball_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
//		positions.at(BC1) =  l *  fac * c2  * bw * 0.4 	   + ( dung_beetle_ball_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
//		positions.at(BC2) =  l *  fac * c2h * bw * 0.4	   + ( dung_beetle_ball_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
		positions.at(BC3) =  r *  fac * lift3  * fw * 0.6 	   + ( dung_beetle_ball_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
		positions.at(BC4) =  r *  fac * down4 * bw * 0.4	   + ( dung_beetle_ball_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
		positions.at(BC5) =  r *  fac * down5  * bw * 0.4	   + ( dung_beetle_ball_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

		// Coxa-Femur Joint Position
//		positions.at(CF0) =  l *  fac * c2h * bw * 0.55     + ( dung_beetle_ball_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
//		positions.at(CF1) =  l * -fac * c1  * fw * 0.7     + ( dung_beetle_ball_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
//		positions.at(CF2) =  l * -fac * c1h * fw * 0.55     + ( dung_beetle_ball_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
		positions.at(CF3) =  r *  fac * down3  * bw * 0.55	   + ( dung_beetle_ball_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
		positions.at(CF4) =  r * -fac * lift4 * fw * 0.7     + ( dung_beetle_ball_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
		positions.at(CF5) =  r * -fac * lift5  * fw * 0.55     + ( dung_beetle_ball_pose[4]);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];


//		float temp = c1;
//		c1 = c2;
//		c2 = temp;
//		float temph = c1h;
//		c1h = c2h;
//		c2h = temph;
//		mleg = 0.2;
//			fw = 0.7;
//			bw = 1.1;
		MC[0]->setPsnInputNeurons(0,1);
		MC[1]->setPsnInputNeurons(0,1);
		MC[2]->setPsnInputNeurons(0,1);

//			fac = 0.5;
		positions.at(BC0) =  l *  fac * lift0 * fw * 0.6 	   + ( dung_beetle_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
		positions.at(BC1) =  l *  fac * down1  * bw * 0.4 	   + ( dung_beetle_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
		positions.at(BC2) =  l *  fac * down2 * bw * 0.4	   + ( dung_beetle_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];

		// Coxa-Femur Joint Position
		positions.at(CF0) =  l *  fac * down0 * bw * 0.55     	   + ( dung_beetle_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
		positions.at(CF1) =  l * -fac * lift1  * fw * 0.7  * mleg     + ( dung_beetle_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
		positions.at(CF2) =  l * -fac * lift2 * fw * 0.55    		   + ( dung_beetle_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];

//		if(doMotion == 'z'){
//			positions.at(CF1) =  l * -fac * c1  * fw * 0.7  * mleg     + ( dung_beetle_ball_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
//		}
		// Femur-Tibia Joint Position
//			positions.at(FT0) =  l * -fac * c2h * 1.3 *-1.0 + ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//			positions.at(FT1) =  l * -fac * c1  * 1.2        + ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//			positions.at(FT2) =  l * -fac * c1h * 1.2        + ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];

		positions.at(FT0) =  ( dung_beetle_ball_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
		positions.at(FT1) =  ( dung_beetle_ball_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
		positions.at(FT2) =  ( dung_beetle_ball_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
		positions.at(FT3) =  ( dung_beetle_ball_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		positions.at(FT4) =  ( dung_beetle_ball_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
		positions.at(FT5) =  ( dung_beetle_ball_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		positions.at(LONGITUDINAL) = dung_beetle_ball_pose.at(LONGITUDINAL);
		positions.at(TRANSVERSAL) = dung_beetle_ball_pose.at(TRANSVERSAL);
		positions.at(HEAD) = dung_beetle_ball_pose.at(HEAD);


	}
	else
	{
		// Thailand db_alpha

		//// position = oscillated signal(rad) + joint bias(rad)
//		positions.at(BC0) =  l *  fac * c1h * 0.4 	   + ( targetBC[0]  * degtoRad);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
//		positions.at(BC1) =  l *  fac * c2  * 0.3 	   + ( targetBC[1]  * degtoRad);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
//		positions.at(BC2) =  l *  fac * c2h * 0.3 	   + ( targetBC[2]  * degtoRad);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
//		positions.at(BC3) =  r *  fac * c1  * 0.4 	   + ( targetBC[0]  * degtoRad);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
//		positions.at(BC4) =  r *  fac * c2h * 0.3 	   + ( targetBC[1]  * degtoRad);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
//		positions.at(BC5) =  r *  fac * c2  * 0.3 	   + ( targetBC[2]  * degtoRad);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];
//
//		// Coxa-Femur Joint Position
//		positions.at(CF0) =  l *  fac * c2h * 0.55     + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
//		positions.at(CF1) =  l * -fac * c1  * 0.5      + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
//		positions.at(CF2) =  l * -fac * c1h * 0.55     + ( targetCF[2]  * degtoRad);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
//		positions.at(CF3) =  r *  fac * c2  * 0.55 	   + ( targetCF[0]  * degtoRad);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
//		positions.at(CF4) =  r * -fac * c1h * 0.5      + ( targetCF[1]  * degtoRad);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
//		positions.at(CF5) =  r * -fac * c1  * 0.55     + ( targetCF[2]  * degtoRad);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];
//
//		// Femur-Tibia Joint Position
//		positions.at(FT0) =  l * -fac * c2h * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//		positions.at(FT1) =  l * -fac * c1  * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//		positions.at(FT2) =  l * -fac * c1h * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
//		positions.at(FT3) =  r * -fac * c2  * 0.45 *-1.0 + ( targetFT[0]  * degtoRad);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
//		positions.at(FT4) =  r * -fac * c1h * 0.3        + ( targetFT[1]  * degtoRad);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
//		positions.at(FT5) =  r * -fac * c1  * 0.3        + ( targetFT[2]  * degtoRad);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];
//
//		positions.at(LONGITUDINAL) = dung_beetle_pose.at(LONGITUDINAL);
//		positions.at(TRANSVERSAL) = dung_beetle_pose.at(TRANSVERSAL);
//		positions.at(HEAD) = dung_beetle_pose.at(HEAD);

		//// position = oscillated signal(rad) + joint bias(rad)
		float test = 0.0;
		if (doMotion == 'b' and activate_walking_grab == true){
			test = 0.1;
			cout << "Tune!!!" << endl;
			cout << doMotion << endl;
		}
		positions.at(BC0) =  l *  fac * lift0 * fw * 0.9 	   + ( dung_beetle_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
		positions.at(BC1) =  l *  fac * down1 * bw * 0.4 	   + ( dung_beetle_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1]; // locomotion 0.4
		positions.at(BC2) =  l *  fac * down2 * bw * 0.4	   + ( dung_beetle_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
		positions.at(BC3) =  r *  fac * lift3 * fw * 0.9 	   + ( dung_beetle_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
		positions.at(BC4) =  r *  fac * down4 * bw * 0.4	   + ( dung_beetle_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
		positions.at(BC5) =  r *  fac * down5 * bw * 0.4	   + ( dung_beetle_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

		// Coxa-Femur Joint Position
		positions.at(CF0) =  l *  fac * down0 * bw * 0.55     + ( dung_beetle_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
		positions.at(CF1) =  l * -fac * lift1 * fw * 0.7      + ( dung_beetle_pose[7]-test);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1]; // locomotion 0.7
		positions.at(CF2) =  l * -fac * lift2 * fw * 0.7      + ( dung_beetle_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
		positions.at(CF3) =  r *  fac * down3 * bw * 0.55	  + ( dung_beetle_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
		positions.at(CF4) =  r * -fac * lift4 * fw * 0.7      + ( dung_beetle_pose[10]-test);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
		positions.at(CF5) =  r * -fac * lift5 * fw * 0.7      + ( dung_beetle_pose[4]);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

//		if(doMotion == 'z'){
//			positions.at(CF4) =  r * -fac * c1h * fw * 0.7     + ( dung_beetle_ball_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
//		}
		// Femur-Tibia Joint Position
		positions.at(FT0) =  l * -fac * down0 * 1.1 *-1.0  + ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
		positions.at(FT1) =  l * -fac * lift1 * 0.7        + ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
		positions.at(FT2) =  l * -fac * lift2 * 0.7        + ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
		positions.at(FT3) =  r * -fac * down3 * 1.1 *-1.0  + ( dung_beetle_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		positions.at(FT4) =  r * -fac * lift4 * 0.7        + ( dung_beetle_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
		positions.at(FT5) =  r * -fac * lift5 * 0.7        + ( dung_beetle_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		// optimize parameters for Object transportation
		// positions.at(BC0) =  l *  fac * lift0 * fw * 0.6 	   + ( dung_beetle_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
		// positions.at(BC1) =  l *  fac * down1 * bw * 0.7 	   + ( dung_beetle_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1]; // locomotion 0.4
		// positions.at(BC2) =  l *  fac * down2 * bw * 0.4	   + ( dung_beetle_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
		// positions.at(BC3) =  r *  fac * lift3 * fw * 0.6 	   + ( dung_beetle_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
		// positions.at(BC4) =  r *  fac * down4 * bw * 0.7	   + ( dung_beetle_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
		// positions.at(BC5) =  r *  fac * down5 * bw * 0.4	   + ( dung_beetle_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];
		// // Coxa-Femur Joint Position
		// positions.at(CF0) =  l *  fac * down0 * bw * 0.6     + ( dung_beetle_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
		// positions.at(CF1) =  l * -fac * lift1 * fw * 0.7      + ( dung_beetle_pose[7]-test);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1]; // locomotion 0.7
		// positions.at(CF2) =  l * -fac * lift2 * fw * 0.7      + ( dung_beetle_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
		// positions.at(CF3) =  r *  fac * down3 * bw * 0.6	  + ( dung_beetle_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
		// positions.at(CF4) =  r * -fac * lift4 * fw * 0.7      + ( dung_beetle_pose[10]-test);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
		// positions.at(CF5) =  r * -fac * lift5 * fw * 0.7      + ( dung_beetle_pose[4]);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];
		// // Femur-Tibia Joint Position
		// positions.at(FT0) =  l * -fac * down0 * 0.7 *-1.0  + ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
		// positions.at(FT1) =  l * -fac * lift1 * 0.7        + ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
		// positions.at(FT2) =  l * -fac * lift2 * 0.7        + ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
		// positions.at(FT3) =  r * -fac * down3 * 0.7 *-1.0  + ( dung_beetle_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
		// positions.at(FT4) =  r * -fac * lift4 * 0.7        + ( dung_beetle_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
		// positions.at(FT5) =  r * -fac * lift5 * 0.7        + ( dung_beetle_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		if (doMotion == 'q'){ // left turn
			cout << doMotion << endl;

//			float temp = c1;
//			c1 = c2;
//			c2 = temp;
//			float temph = c1h;
//			c1h = c2h;
//			c2h = temph;
//			mleg = 0.2;
//			fw = 0.7;
//			bw = 1.1;
			MC[0]->setPsnInputNeurons(0,1);
			MC[1]->setPsnInputNeurons(0,1);
			MC[2]->setPsnInputNeurons(0,1);


//			fac = 0.5;
			positions.at(BC0) =  l *  fac * lift0 * fw * 0.9 	   + ( dung_beetle_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
			positions.at(BC1) =  l *  fac * down1 * bw * 0.4 	   + ( dung_beetle_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
			positions.at(BC2) =  l *  fac * down2 * bw * 0.4	   + ( dung_beetle_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];

			// Coxa-Femur Joint Position
			positions.at(CF0) =  l *  fac * down0 * bw * 0.55      + ( dung_beetle_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
			positions.at(CF1) =  l * -fac * lift1 * fw * 0.7       + ( dung_beetle_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
			positions.at(CF2) =  l * -fac * lift2 * fw * 0.7      + ( dung_beetle_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];

			// Femur-Tibia Joint Position
//			positions.at(FT0) =  l * -fac * c2h * 1.3 *-1.0 + ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//			positions.at(FT1) =  l * -fac * c1  * 1.2        + ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//			positions.at(FT2) =  l * -fac * c1h * 1.2        + ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];

			positions.at(FT0) =  ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
			positions.at(FT1) =  ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
			positions.at(FT2) =  ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
			positions.at(FT3) =  ( dung_beetle_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
			positions.at(FT4) =  ( dung_beetle_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
			positions.at(FT5) =  ( dung_beetle_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		}
		else if (doMotion == 'e'){ // right turn

//			float temp = c1;
//			c1 = c2;
//			c2 = temp;
//			float temph = c1h;
//			c1h = c2h;
//			c2h = temph;
//			fw = 1.0;
//			bw = 1.5;
			MC[3]->setPsnInputNeurons(0,1);
			MC[4]->setPsnInputNeurons(0,1);
			MC[5]->setPsnInputNeurons(0,1);


//			fac = 0.6;
			positions.at(BC3) =  r *  fac * lift3  * fw * 0.9 	   + ( dung_beetle_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
			positions.at(BC4) =  r *  fac * down4  * bw * 0.4	   + ( dung_beetle_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
			positions.at(BC5) =  r *  fac * down5  * bw * 0.4	   + ( dung_beetle_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];

			// Coxa-Femur Joint Position
			positions.at(CF3) =  r *  fac * down3  * bw * 0.55	   + ( dung_beetle_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
			positions.at(CF4) =  r * -fac * lift4  * fw * 0.7      + ( dung_beetle_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
			positions.at(CF5) =  r * -fac * lift5  * fw * 0.7	   + ( dung_beetle_pose[4]);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];

			// Femur-Tibia Joint Position
//			positions.at(FT3) =  r * -fac * c2  * 1.3 *-1.0 + ( dung_beetle_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
//			positions.at(FT4) =  r * -fac * c1h * 1.2        + ( dung_beetle_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
//			positions.at(FT5) =  r * -fac * c1  * 1.2        + ( dung_beetle_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

			// Femur-Tibia Joint Position
//			positions.at(FT0) =  l * -fac * c2h * 1.3 *-1.0 + ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//			positions.at(FT1) =  l * -fac * c1  * 1.2        + ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
//			positions.at(FT2) =  l * -fac * c1h * 1.2        + ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];

			positions.at(FT0) =  ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
			positions.at(FT1) =  ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
			positions.at(FT2) =  ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
			positions.at(FT3) =  ( dung_beetle_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
			positions.at(FT4) =  ( dung_beetle_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
			positions.at(FT5) =  ( dung_beetle_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

		}

		positions.at(LONGITUDINAL) = dung_beetle_pose.at(LONGITUDINAL);
		positions.at(TRANSVERSAL) = dung_beetle_pose.at(TRANSVERSAL);
		positions.at(HEAD) = dung_beetle_pose.at(HEAD);

	}
	cout << "finish process behavior" << endl;
	// ground searching system
	if (activate_fc_closeloop == true) 
	{
		/// Leg trajectory with foot contact error correction
	//	float fc_error_lt = (fc_error[0]+fc_error[2]+fc_error[4])/3.0;
	//	float fc_error_rt = (fc_error[1]+fc_error[3]+fc_error[5])/3.0;

	//	cout << fc_err_fac << "\n";

		if(doMotion == 'z'){
			positions.at(BC0) -= fc_error_st[0] * fc_err_fac;
			positions.at(BC3) -= fc_error_st[3] * fc_err_fac;
			positions.at(BC2) += fc_error_st[2] * fc_err_fac;
			positions.at(BC5) += fc_error_st[5] * fc_err_fac;

			positions.at(CF1) += fc_error_st[1] * fc_err_fac;
			positions.at(CF4) += fc_error_st[4] * fc_err_fac;

		}
		else if(doMotion == 'R'){
			// positions.at(CF0) += fc_error_st[0] * fc_err_fac;
			// positions.at(CF3) += fc_error_st[3] * fc_err_fac;
			positions.at(CF1) += fc_error_st[1] * fc_err_fac;
			positions.at(CF2) += fc_error_st[2] * fc_err_fac;
			positions.at(CF4) += fc_error_st[4] * fc_err_fac;
			positions.at(CF5) += fc_error_st[5] * fc_err_fac;

			// positions.at(FT0) -= fc_error_st[0] * fc_err_fac;
			// positions.at(FT3) -= fc_error_st[3] * fc_err_fac;
			positions.at(FT1) -= fc_error_st[1] * fc_err_fac;
			positions.at(FT2) -= fc_error_st[2] * fc_err_fac;
			positions.at(FT4) -= fc_error_st[4] * fc_err_fac;
			positions.at(FT5) -= fc_error_st[5] * fc_err_fac;

			// // Contact Manipulation
			// // positions.at(BC0) += ball_push_sig[0] * ball_push_gain;
			// // positions.at(BC3) += ball_push_sig[3] * ball_push_gain;

			// positions.at(BC1) += ball_push_sig[1] * ball_push_gain;
			// positions.at(BC2) += ball_push_sig[2] * ball_push_gain;
			// positions.at(BC4) += ball_push_sig[4] * ball_push_gain;
			// positions.at(BC5) += ball_push_sig[5] * ball_push_gain;

			// // positions.at(CF0) += ball_push_sig[0] * ball_push_gain;
			// // positions.at(CF3) += ball_push_sig[3] * ball_push_gain;

			// positions.at(CF1) += ball_push_sig[1] * ball_push_gain;
			// positions.at(CF2) += ball_push_sig[2] * ball_push_gain;
			// positions.at(CF4) += ball_push_sig[4] * ball_push_gain;
			// positions.at(CF5) += ball_push_sig[5] * ball_push_gain;

			// // positions.at(FT0) += ball_push_sig[0] * ball_push_gain;
			// // positions.at(FT3) += ball_push_sig[3] * ball_push_gain;

			// positions.at(FT1) += ball_push_sig[1] * ball_push_gain;
			// positions.at(FT2) += ball_push_sig[2] * ball_push_gain;
			// positions.at(FT4) += ball_push_sig[4] * ball_push_gain;
			// positions.at(FT5) += ball_push_sig[5] * ball_push_gain;
		}
		else{
//			positions.at(BC0) -= fc_error_st[0] * fc_err_fac;
//			positions.at(BC3) -= fc_error_st[3] * fc_err_fac;
//			positions.at(BC1) += fc_error_st[1] * fc_err_fac;
//			positions.at(BC2) += fc_error_st[2] * fc_err_fac;
//			positions.at(BC4) += fc_error_st[4] * fc_err_fac;
//			positions.at(BC5) += fc_error_st[5] * fc_err_fac;

			positions.at(CF0) += fc_error_st[0] * fc_err_fac;
			positions.at(CF3) += fc_error_st[3] * fc_err_fac;
			positions.at(CF1) += fc_error_st[1] * fc_err_fac;
			positions.at(CF2) += fc_error_st[2] * fc_err_fac;
			positions.at(CF4) += fc_error_st[4] * fc_err_fac;
			positions.at(CF5) += fc_error_st[5] * fc_err_fac;

			positions.at(FT0) += fc_error_st[0] * fc_err_fac;
			positions.at(FT3) += fc_error_st[3] * fc_err_fac;
			positions.at(FT1) += fc_error_st[1] * fc_err_fac;
			positions.at(FT2) += fc_error_st[2] * fc_err_fac;
			positions.at(FT4) += fc_error_st[4] * fc_err_fac;
			positions.at(FT5) += fc_error_st[5] * fc_err_fac;
		}
	}

	if (activate_roll_control){
		// roll velocity --> leg extension reflex
		cout << "activate_roll_control " << endl;
		// angular velocity regulate leg movement reflex
		positions.at(CF0) += roll_cpg_mod_l * roll_leg_fac;
		positions.at(CF3) += roll_cpg_mod_r * roll_leg_fac;

		positions.at(FT0) += roll_cpg_mod_l * roll_leg_fac;
		positions.at(FT3) += roll_cpg_mod_r * roll_leg_fac;
		////////////////

		// positions.at(FT0) -= tilt_err[0] * roll_leg_fac;
		// positions.at(FT3) += tilt_err[3] * roll_leg_fac;
		
		// Roll Angle to modulate front Leg configuration
		// positions.at(CF0) += relu(-angle[0]) * degtoRad *0.5;
		// positions.at(CF3) += relu( angle[0]) * degtoRad *0.5;
		
		// positions.at(FT0) += relu(-angle[0]) * degtoRad *0.5;
		// positions.at(FT3) += relu( angle[0]) * degtoRad *0.5;
		// positions.at(FT0) += sigmoid(relu(-angle[0]), -0.1, 26, -65);
		// positions.at(FT3) += sigmoid(relu( angle[0]), -0.1, 26, -65);
	}

	// swing reflex system (only for forward walking condition)
	if ((doMotion == 'w' or doMotion == '1') and activate_sw_avoid){ 
//		positions.at(BC0) -= activate_sw_leg_avoid[0] * sw_err_fac;
//		positions.at(BC3) -= activate_sw_leg_avoid[3] * sw_err_fac;
//		positions.at(BC1) -= activate_sw_leg_avoid[1] * sw_err_fac;
//		positions.at(BC2) -= activate_sw_leg_avoid[2] * sw_err_fac;
//		positions.at(BC4) -= activate_sw_leg_avoid[4] * sw_err_fac;
//		positions.at(BC5) -= activate_sw_leg_avoid[5] * sw_err_fac;
//
//		positions.at(CF0) -= activate_sw_leg_avoid[0] * sw_err_fac;
//		positions.at(CF3) -= activate_sw_leg_avoid[3] * sw_err_fac;
//		positions.at(CF1) -= activate_sw_leg_avoid[1] * sw_err_fac;
//		positions.at(CF2) -= activate_sw_leg_avoid[2] * sw_err_fac;
//		positions.at(CF4) -= activate_sw_leg_avoid[4] * sw_err_fac;
//		positions.at(CF5) -= activate_sw_leg_avoid[5] * sw_err_fac;
//
//		positions.at(FT0) += activate_sw_leg_avoid[0] * sw_err_fac * 4.0;
//		positions.at(FT3) += activate_sw_leg_avoid[3] * sw_err_fac * 2.0;
//		positions.at(FT1) += activate_sw_leg_avoid[1] * sw_err_fac * 2.0;
//		positions.at(FT2) += activate_sw_leg_avoid[2] * sw_err_fac * 4.0;
//		positions.at(FT4) += activate_sw_leg_avoid[4] * sw_err_fac * 2.0;
//		positions.at(FT5) += activate_sw_leg_avoid[5] * sw_err_fac * 2.0;

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


//		positions.at(CF0) -= sw_error[3] * sw_err_fac * 0.2;
//		positions.at(CF3) -= sw_error[0] * sw_err_fac * 0.2;o

//		if(positions.at(FT0) < -1.5){ positions.at(FT0) = -1.3;}
//		if(positions.at(FT1) < -1.5){ positions.at(FT1) = -1.3;}
//		if(positions.at(FT2) < -1.5){ positions.at(FT2) = -1.3;}
//		if(positions.at(FT3) < -1.5){ positions.at(FT3) = -1.3;}
//		if(positions.at(FT4) < -1.5){ positions.at(FT4) = -1.3;}
//		if(positions.at(FT5) < -1.5){ positions.at(FT5) = -1.3;}
//		}
	}
	else if (doMotion == 'b' and activate_sw_avoid){
		for(int c = 0; c < 6; c++){
			sw_err_fac[c] = 0.7;
		}

		positions.at(BC0) += sw_error[0] * sw_err_fac[0]*1.0 - tau_reflexChain[0].Read(reflex_delay)* sw_err_fac[0]*1.5;
		positions.at(BC3) += sw_error[3] * sw_err_fac[3]*1.0 - tau_reflexChain[3].Read(reflex_delay)* sw_err_fac[3]*1.5;
		positions.at(BC1) -= sw_error[1] * sw_err_fac[1]*1.5 ;
		positions.at(BC2) -= sw_error[2] * sw_err_fac[2]*1.5 ;
		positions.at(BC4) -= sw_error[4] * sw_err_fac[4]*1.5 ;
		positions.at(BC5) -= sw_error[5] * sw_err_fac[5]*1.5 ;

		// cout << "tau_reflexChain  :  " << tau_reflexChain[3].Read(5) << endl;

		positions.at(CF0) -= sw_error[0] * sw_err_fac[0] * 1.5;
		positions.at(CF3) -= sw_error[3] * sw_err_fac[3] * 1.5;
		positions.at(CF1) -= sw_error[1] * sw_err_fac[1] * 1.5;//tau_reflexChain[1].Read(reflex_delay)* sw_err_fac[1] * 1.5;//sw_error[1] * sw_err_fac * 1.5;
		positions.at(CF2) -= sw_error[2] * sw_err_fac[2] * 1.5;//tau_reflexChain[2].Read(reflex_delay)* sw_err_fac[2] * 1.5;//sw_error[2] * sw_err_fac * 1.5;
		positions.at(CF4) -= sw_error[4] * sw_err_fac[4] * 1.5;//tau_reflexChain[4].Read(reflex_delay)* sw_err_fac[4] * 1.5;//sw_error[4] * sw_err_fac * 1.5;
		positions.at(CF5) -= sw_error[5] * sw_err_fac[5] * 1.5;//tau_reflexChain[5].Read(reflex_delay)* sw_err_fac[5] * 1.5;//sw_error[5] * sw_err_fac * 1.5;

//		for(int i=0; i<6; i++){
		positions.at(FT0) +=  (sw_error[0] * sw_err_fac[0] * 1.5 );// - sw_switch_err[0];
		positions.at(FT3) +=  (sw_error[3] * sw_err_fac[3] * 1.5 );// - sw_switch_err[3];
		positions.at(FT1) -=  (sw_error[1] * sw_err_fac[1] * 1.5 );// - sw_switch_err[1];
		positions.at(FT2) -=  (sw_error[2] * sw_err_fac[2] * 1.5 );// - sw_switch_err[2];
		positions.at(FT4) -=  (sw_error[4] * sw_err_fac[4] * 1.5 );// - sw_switch_err[4];
		positions.at(FT5) -=  (sw_error[5] * sw_err_fac[5] * 1.5 );// - sw_switch_err[5];

	}

	// ground searching system for ball rolling
	if(activate_rolling_grab){ 
		/// grab ball by extend CF joint
		positions.at(CF0) += fc_error_st[0] * fc_err_fac;
		positions.at(CF3) += fc_error_st[3] * fc_err_fac;
		positions.at(CF1) += fc_error_st[1] * fc_err_fac;
		positions.at(CF2) += fc_error_st[2] * fc_err_fac;
		positions.at(CF4) += fc_error_st[4] * fc_err_fac;
		positions.at(CF5) += fc_error_st[5] * fc_err_fac;
//		cout << fc_error_st[5] << endl;
	}
	// choosing 1 Single leg for testing
	if(not full_robot){

		// Test Left Front Leg L1
//		oneleg_positions.at(0) = positions.at(BC0);
//		oneleg_positions.at(1) = positions.at(CF0);
//		oneleg_positions.at(2) = positions.at(FT0);

		// Test Left Middle Leg L2
//		oneleg_positions.at(0) = positions.at(BC1);
//		oneleg_positions.at(1) = positions.at(CF1);
//		oneleg_positions.at(2) = positions.at(FT1);

		// Test Left Hind Leg L2
		oneleg_positions.at(0) = positions.at(BC2);
		oneleg_positions.at(1) = positions.at(CF2);
		oneleg_positions.at(2) = positions.at(FT2);

		// Test Right Front Leg R1
		// oneleg_positions.at(0) = positions.at(BC3);
		// oneleg_positions.at(1) = positions.at(CF3);
		// oneleg_positions.at(2) = positions.at(FT3);

		// Test Right Middle Leg R2
		// oneleg_positions.at(0) = positions.at(BC4);
		// oneleg_positions.at(1) = positions.at(CF4);
		// oneleg_positions.at(2) = positions.at(FT4);

		// Test Right Hind Leg R3
		// oneleg_positions.at(0) = positions.at(BC5);
		// oneleg_positions.at(1) = positions.at(CF5);
		// oneleg_positions.at(2) = positions.at(FT5);

		// Couple Front Legs
//		couple_positions.at(0) = positions.at(BC0);
//		couple_positions.at(1) = positions.at(CF0);
//		couple_positions.at(2) = positions.at(FT0);
//
//		couple_positions.at(3) = positions.at(BC3);
//		couple_positions.at(4) = positions.at(CF3);
//		couple_positions.at(5) = positions.at(FT3);


	}
	// grabing small pallet at hind leg posture
	if(activate_walking_grab){
		positions.at(BC2) = dung_beetle_pose[0]+joint_pos_hind_grab[0];
		positions.at(CF2) = dung_beetle_pose[1]+joint_pos_hind_grab[1];
		positions.at(FT2) = dung_beetle_pose[2]+joint_pos_hind_grab[2];

		positions.at(BC5) = dung_beetle_pose[3]+joint_pos_hind_grab[0];
		positions.at(CF5) = dung_beetle_pose[4]+joint_pos_hind_grab[1];
		positions.at(FT5) = dung_beetle_pose[5]+joint_pos_hind_grab[2];
		
		// oscillate hind leg
		if(activate_hingleg_oscillate == true){
			positions.at(BC2) -= 0.3*so2->getOutput(0);
			positions.at(BC5) -= 0.3*so2->getOutput(0);

			positions.at(CF2) -= 0.3*so2->getOutput(0);
			positions.at(CF5) -= 0.3*so2->getOutput(0);

		}
	}
	cout << "finish add on modules" << endl;

//	//// position = oscillated signal(rad) + joint bias(rad)
//	positions.at(BC0) =  ( dung_beetle_pose[12]);//( MC1.getFinalNeuronOutput(0) + biasBC0)*degtoRad*rangeBC[0];
//	positions.at(BC1) =  ( dung_beetle_pose[6]);//( MC1.getFinalNeuronOutput(1) + biasBC1)*degtoRad*rangeBC[1];
////		positions.at(BC2) =  l *  fac * c2h * bw * 0.4	   + ( dung_beetle_pose[0]);//( MC1.getFinalNeuronOutput(2) + biasBC2)*degtoRad*rangeBC[2];
//	positions.at(BC3) =  ( dung_beetle_pose[15]);//( MC1.getFinalNeuronOutput(3) + biasBC3)*degtoRad*rangeBC[0];
//	positions.at(BC4) =  ( dung_beetle_pose[9]);//( MC1.getFinalNeuronOutput(4) + biasBC4)*degtoRad*rangeBC[1];
//	positions.at(BC5) =  ( dung_beetle_pose[3]);//( MC1.getFinalNeuronOutput(5) + biasBC5)*degtoRad*rangeBC[2];
//
//	// Coxa-Femur Joint Position
//	positions.at(CF0) =  ( dung_beetle_pose[13]);//( MC1.getFinalNeuronOutput(6)  + biasCF0)*degtoRad*rangeCF[0];
//	positions.at(CF1) =  ( dung_beetle_pose[7]);//( MC1.getFinalNeuronOutput(7)  + biasCF1)*degtoRad*rangeCF[1];
////		positions.at(CF2) =  l * -fac * c1h * fw * 0.55     + ( dung_beetle_pose[1]);//( MC1.getFinalNeuronOutput(8)  + biasCF2)*degtoRad*rangeCF[2];
//	positions.at(CF3) =  ( dung_beetle_pose[16]);//( MC1.getFinalNeuronOutput(9)  + biasCF3)*degtoRad*rangeCF[0];
//	positions.at(CF4) =  ( dung_beetle_pose[10]);//( MC1.getFinalNeuronOutput(10) + biasCF4)*degtoRad*rangeCF[1];
//	positions.at(CF5) =  ( dung_beetle_pose[4]);;//( MC1.getFinalNeuronOutput(11) + biasCF5)*degtoRad*rangeCF[2];
//
//	// Femur-Tibia Joint Position
//	positions.at(FT0) =  ( dung_beetle_pose[14]);//( MC1.getFinalNeuronOutput(12) + biasFT0)*degtoRad*rangeFT[0];
//	positions.at(FT1) =  ( dung_beetle_pose[8]);//( MC1.getFinalNeuronOutput(13) + biasFT1)*degtoRad*rangeFT[1];
////		positions.at(FT2) =  l * -fac * c1h * 1.2        + ( dung_beetle_pose[2]);//( MC1.getFinalNeuronOutput(14) + biasFT2)*degtoRad*rangeFT[2];
//	positions.at(FT3) =  ( dung_beetle_pose[17]);//( MC1.getFinalNeuronOutput(15) + biasFT3)*degtoRad*rangeFT[0];
//	positions.at(FT4) =  ( dung_beetle_pose[11]);//( MC1.getFinalNeuronOutput(16) + biasFT4)*degtoRad*rangeFT[1];
//	positions.at(FT5) =  ( dung_beetle_pose[5]);//( MC1.getFinalNeuronOutput(17) + biasFT5)*degtoRad*rangeFT[2];

	////////////////////////////////////////
	//// Collect CPG signal to CSV /////////
	////////////////////////////////////////
	// pcpg_signal.clear();
	// vrn_signal.clear();
	// psn_signal.clear();
	// // cout << "Clear" << endl;

	// cpg_signal.push_back(data_counter);
	// cpg_signal.push_back(MC[0]->getCpgOutput(0));
	// cpg_signal.push_back(MC[0]->getCpgOutput(1));
	// // cout << "CPG" << endl;

	// pcpg_signal.push_back(data_counter);
	// pcpg_signal.push_back(MC[0]->getpcpgOutput1(0));
	// pcpg_signal.push_back(MC[0]->getpcpgOutput1(1));
	// pcpg_signal.push_back(MC[0]->getpcpgOutput2(0));
	// pcpg_signal.push_back(MC[0]->getpcpgOutput2(1));
	// // cout << "pcpg" << endl;

	// vrn_signal.push_back(data_counter);
	// vrn_signal.push_back(MC[0]->getVrnOutput1(6));
	// vrn_signal.push_back(MC[0]->getVrnOutput3(6));
	// // cout << "vrn" << endl;

	// psn_signal.push_back(data_counter);
	// psn_signal.push_back(objective);
	// psn_signal.push_back(MC[0]->getVrnOutput1(6));
	// psn_signal.push_back(MC[0]->getVrnOutput3(6));
	// psn_signal.push_back(MC[0]->getPsnOutput0(10));
	// psn_signal.push_back(MC[0]->getPsnOutput0(11));

	// legsearch_signal.push_back(int(activate_sw_avoid));
	// legsearch_signal.push_back(int(activate_sw_switch));
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fcphase.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fc_sens_raw.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fc_sens.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fc_error_st.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(max_fc_error.at(c)); }

// //	legsearch_signal.push_back(int(activate_sw_avoid));
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_sens_raw.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_sens.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_error.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(max_sw_error.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(activate_sw_leg_avoid.at(c)); }
// //	legsearch_signal.push_back(down0);

// //	legsearch_signal.push_back(int(activate_sw_switch));
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(FT_torque.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_switch_sens.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_switch_err.at(c)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_error[c] * sw_err_fac[c] * 2.0); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_switch_spike[c]); }

// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(LS_out[c]); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(tau_reflexChain[c].Read(reflex_delay)); }
// 	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(sw_error_mem[c]); }
// 	cout << "finish Collect Data" << endl;

//	cout << "cpg_signal :" << cpg_signal.at(0) << " " << cpg_signal.at(1) << " " << cpg_signal.at(2) << endl;

	// CPG interconnection 
	// hypo 1 : purturbation to CPG self stance inhibition
	plotdata.push_back(MC[0]->getCpgOutput(0));
	// plotdata.push_back(MC[1]->getCpgOutput(1));
	plotdata.push_back(MC[3]->getCpgOutput(0));
	// plotdata.push_back(MC[2]->getCpgOutput(1));
	// cout << "MC0 C0" << MC[0]->getCpgOutput(0);
	// cout << "MC0 C1" << MC[0]->getCpgOutput(1);

	// cpg_input_mod[0] = -abs(fc_sens_raw[0])*cpg_input_w_f; //- abs(fc_sens_raw[3])*cpg_input_w;
	// cpg_input_mod[3] = -abs(fc_sens_raw[3])*cpg_input_w_f; //- abs(fc_sens_raw[0])*cpg_input_w;
	// plotdata.push_back(fc_sens_raw[0]);
	// plotdata.push_back(fc_sens_raw[3]);
	cout << "fc_sens_raw 0: " << fc_sens_raw[0] << endl;
	cout << "fc_sens_raw 1: " << fc_sens_raw[3] << endl;
	// plotdata.push_back(cpg_input_mod[0]);
	// plotdata.push_back(cpg_input_mod[3]);
	cout << "cpg_input_mod 0: " << cpg_input_mod[0] << endl;
	cout << "cpg_input_mod 1: " << cpg_input_mod[3] << endl;

	// cpg_input_mod[1] = -abs(-fc_sens_raw[1])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[2])*cpg_input_w - abs(fc_sens_raw[4])*cpg_input_w;
	// cpg_input_mod[2] = -abs(-fc_sens_raw[2])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[1])*cpg_input_w - abs(fc_sens_raw[5])*cpg_input_w;
	// cpg_input_mod[4] = -abs(-fc_sens_raw[4])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[5])*cpg_input_w - abs(fc_sens_raw[1])*cpg_input_w;
	// cpg_input_mod[5] = -abs(-fc_sens_raw[5])*cpg_input_w_b*0.1; //- abs(fc_sens_raw[4])*cpg_input_w - abs(fc_sens_raw[2])*cpg_input_w;

	// hypo 2 : left-right interleg swing inhibition
	// cpg_input_mod[0] = -abs(1-fc_sens[3])*cpg_input_w_f; 
	// cpg_input_mod[3] = -abs(1-fc_sens[0])*cpg_input_w_f;

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
	// cpg_input_mod[0] = roll_cpg_mod_l*cpg_input_w_f*0.5; //- abs(fc_sens_raw[3])*cpg_input_w;
	// cpg_input_mod[3] = roll_cpg_mod_r*cpg_input_w_f*0.5; //- abs(fc_sens_raw[0])*cpg_input_w;
	// oppose side tilt excite swing
	// cpg_input_mod[0] = roll_cpg_mod_r*cpg_input_w_f; //- abs(fc_sens_raw[3])*cpg_input_w;
	// cpg_input_mod[3] = roll_cpg_mod_l*cpg_input_w_f; //- abs(fc_sens_raw[0])*cpg_input_w;

	// cpg_input_mod[3] -= roll_cpg_mod_r*cpg_input_w_f*0.2; //- abs(fc_sens_raw[3])*cpg_input_w;
	// cpg_input_mod[0] -= roll_cpg_mod_l*cpg_input_w_f*0.2; //- abs(fc_sens_raw[0])*cpg_input_w;

	for (int i=0; i<6; i++){
		// phase modulation force to cpg self inhibitiom
		// cpg_input_mod[i] = -abs(-fc_sens_raw[i])*cpg_input_w*0.1;

		// MC[i]->setCpgInputNeuron(cpg_input_mod[i]);
	}
	
	// pcpg_signal.clear();
	// vrn_signal.clear();
	// psn_signal.clear();
	// CPG signals
	cpg_signal.push_back(data_counter);
	for(int c = 0; c < 6; c++){ cpg_signal.push_back(cpg_input_mod[c]); }
	for(int c = 0; c < 6; c++){ cpg_signal.push_back(MC[c]->getCpgOutput(0)); }
	for(int c = 0; c < 6; c++){ cpg_signal.push_back(MC[c]->getCpgOutput(1)); }
	for(int c = 0; c < 6; c++){ cpg_signal.push_back(MC[c]->getpmnOutput(0)); }
	for(int c = 0; c < 6; c++){ cpg_signal.push_back(MC[c]->getpmnOutput(1)); }

	pcpg_signal.push_back(data_counter);
	pcpg_signal.push_back(MC[0]->getpcpgOutput1(0));
	pcpg_signal.push_back(MC[0]->getpcpgOutput1(1));
	pcpg_signal.push_back(MC[0]->getpcpgOutput2(0));
	pcpg_signal.push_back(MC[0]->getpcpgOutput2(1));
	pcpg_signal.push_back(MC[0]->getOutputtau_vrn3(0));

	// cout << "pcpg" << endl;

	vrn_signal.push_back(data_counter);
	vrn_signal.push_back(MC[0]->getVrnOutput1(6));
	vrn_signal.push_back(MC[0]->getVrnOutput3(6));
	// cout << "vrn" << endl;

	psn_signal.push_back(data_counter);
	psn_signal.push_back(objective);
	psn_signal.push_back(MC[0]->getVrnOutput1(6));
	psn_signal.push_back(MC[0]->getVrnOutput3(6));
	psn_signal.push_back(MC[0]->getPsnOutput0(10));
	psn_signal.push_back(MC[0]->getPsnOutput0(11));

	// Ball Control System
	ballDistanceControlData.push_back(data_counter);
	ballDistanceControlData.push_back(ball_push_gain);
	ballDistanceControlData.push_back(target_robot_ball_distance);
	ballDistanceControlData.push_back(e_ball);
	ballDistanceControlData.push_back(ball_push_input);
	for(int c = 0; c < 6; c++){ ballDistanceControlData.push_back(joint_pos_ball_contact_sens[c]); }
	for(int c = 0; c < 6; c++){ ballDistanceControlData.push_back(ball_push_sig[c]); }
	
	// // Local Leg Control System
	legsearch_signal.push_back(data_counter);
	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fcphase[c]); }
	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fc_sens_raw[c]); }
	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fc_sens[c]); }
	for(int c = 0; c < 6; c++){ legsearch_signal.push_back(fc_error_st[c]); }
	// // Roll Control System
	rollControlData.push_back(data_counter);
	rollControlData.push_back(activate_roll_control);
	for(int c = 0; c < 3; c++){ rollControlData.push_back(omega[c]); }
	for(int c = 0; c < 3; c++){ rollControlData.push_back(angle_lowpass[c]); }
	rollControlData.push_back(roll_cpg_mod_l);
	rollControlData.push_back(roll_cpg_mod_r);
	rollControlData.push_back(roll_leg_fac);
	// // Pitch Control System
	pitchControlData.push_back(data_counter);
	pitchControlData.push_back(activate_pitch_control);
	pitchControlData.push_back(pitch);
	pitchControlData.push_back(pitch_error);
	pitchControlData.push_back(pitch_freq_mod);
	pitchControlData.push_back(front_MI);
	pitchControlData.push_back(back_MI);
	pitchControlData.push_back(pitch_front_amp_gain);
	pitchControlData.push_back(pitch_back_amp_gain);
	for (int i=0;i<12;i++){pitchControlData.push_back(trackingError[i]); }		
	
	
	///////////////////////////////////////////
    // Take step with CPGs & value Step
	cout << "cpg_step" << endl;
	for (int i=0; i<6; i++){
		MC[i]->step();				// update all neuron activity
		tauCF[i].Step();
	}
	so2->step();
	// tau_openl_eff3->Step();
	// tau_openl_eff4->Step();
	for(int c = 0; c < 6; c++){
		tau_reflexChain[c].Write(sw_error[c]);
		tau_reflexChain[c].Step();
	}
	// cout << "finish cpg_step" << endl;
//	tau_downsignal0->Step();
//	tau_downsignal1->Step();
//	tau_downsignal2->Step();
//	tau_downsignal3->Step();
//	tau_downsignal4->Step();
//	tau_downsignal5->Step();
	//////////////////////////////////////////
    // Forward walkng: Only CPG signals needed
//    float output_cpg_0 = CPG->getCpgOutput(0);
//    float output_cpg_1 = CPG->getCpgOutput(1);

    // Get CPG Values:
    /*output_mnn_0 = CPG->getFinalNeuronOutput(10);
    output_mnn_1 = CPG->getFinalNeuronOutput(11);
    output_mnn_2 = CPG->getFinalNeuronOutput(12);
    output_mnn_3 = CPG->getFinalNeuronOutput(13);
    output_mnn_4 = CPG->getFinalNeuronOutput(4);
    output_mnn_5 = CPG->getFinalNeuronOutput(5);
    output_mnn_6 = CPG->getFinalNeuronOutput(6);
    output_mnn_7 = CPG->getFinalNeuronOutput(7);
    output_mnn_8 = CPG->getFinalNeuronOutput(6);
    output_mnn_9 = CPG->getFinalNeuronOutput(7);*/
    
    // 2. Re-scale

    // REGULAR GAIT

    // TC
    /*float TC_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, -output_cpg_0); // front TC motors
    float TC_0 = TC_0_ref + dung_beetle_pose.at(BC0);
    float TC_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.25, -0.25, -output_cpg_1); // front TC motors
    TC_1 = TC_1 + dung_beetle_pose.at(BC1); 
    float TC_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, output_cpg_1); // front TC motors
    TC_2 = TC_2 + dung_beetle_pose.at(BC2);
    float TC_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, -output_cpg_0); // front TC motors
    float TC_3 = TC_3_ref + dung_beetle_pose.at(BC3);
    float TC_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.25, -0.25, -output_cpg_1); // front TC motors
    TC_4 = TC_4 + dung_beetle_pose.at(BC4); 
    float TC_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, output_cpg_1); // front TC motors
    TC_5 = TC_5 + dung_beetle_pose.at(BC5);

    // CF
    float CF_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.4, -0.4, -output_cpg_1); // front TC motors
    float CF_0 = CF_0_ref + dung_beetle_pose.at(CF0);
    float CF_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.2, -0.2, output_cpg_0); // front TC motors
    CF_1 = CF_1 + dung_beetle_pose.at(CF1); 
    float CF_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.35, -0.35, -output_cpg_0); // front TC motors
    CF_2 = CF_2 + dung_beetle_pose.at(CF2);
    float CF_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.4, -0.4, output_cpg_1); // front TC motors
    float CF_3 = CF_3_ref + dung_beetle_pose.at(CF3);
    float CF_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.2, -0.2, -output_cpg_0); // front TC motors
    CF_4 = CF_4 + dung_beetle_pose.at(CF4); 
    float CF_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.35, -0.35, output_cpg_0); // front TC motors
    CF_5 = CF_5 + dung_beetle_pose.at(CF5);

    float FT_0 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, -output_cpg_0);
    FT_0 = FT_0 + dung_beetle_pose.at(FT0);
    float FT_3 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.3, -0.3, output_cpg_0);
    FT_3 = FT_3 + dung_beetle_pose.at(FT3);*/

    // SMALL STEP GAIT

    // TC
//    float TC_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, -output_cpg_0); // front TC motors
//    float TC_0 = TC_0_ref + dung_beetle_pose.at(BC0);
//    float TC_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_1); // front TC motors
//    TC_1 = TC_1 + dung_beetle_pose.at(BC1);
//    float TC_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, output_cpg_1); // front TC motors
//    TC_2 = TC_2 + dung_beetle_pose.at(BC2);
//    float TC_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, -output_cpg_0); // front TC motors
//    float TC_3 = TC_3_ref + dung_beetle_pose.at(BC3);
//    float TC_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_1); // front TC motors
//    TC_4 = TC_4 + dung_beetle_pose.at(BC4);
//    float TC_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, output_cpg_1); // front TC motors
//    TC_5 = TC_5 + dung_beetle_pose.at(BC5);
//
//    // CF
//    float CF_0_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, -output_cpg_1); // front TC motors
//    float CF_0 = CF_0_ref + dung_beetle_pose.at(CF0);
//    float CF_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, output_cpg_0); // front TC motors
//    CF_1 = CF_1 + dung_beetle_pose.at(CF1);
//    float CF_2 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.2, -0.2, -output_cpg_0); // front TC motors
//    CF_2 = CF_2 + dung_beetle_pose.at(CF2);
//    float CF_3_ref = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.15, -0.15, output_cpg_1); // front TC motors
//    float CF_3 = CF_3_ref + dung_beetle_pose.at(CF3);
//    float CF_4 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_0); // front TC motors
//    CF_4 = CF_4 + dung_beetle_pose.at(CF4);
//    float CF_5 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.2, -0.2, output_cpg_0); // front TC motors
//    CF_5 = CF_5 + dung_beetle_pose.at(CF5);

    /*float FT_0 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_0);
    FT_0 = FT_0 + dung_beetle_pose.at(FT0);
    float FT_3 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, output_cpg_0);
    FT_3 = FT_3 + dung_beetle_pose.at(FT3);
    float FT_hind_0 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_1);
    float FT_hind_2 = FT_hind_0 + dung_beetle_pose.at(FT2);
    float FT_hind_4 = FT_hind_0 + dung_beetle_pose.at(FT4);
    float FT_hind_1 = rescale(MAX_NETWORK_OUTPUT, -MAX_NETWORK_OUTPUT, 0.1, -0.1, -output_cpg_1);
    FT_hind_1 = FT_hind_1 + dung_beetle_pose.at(FT1);
    float FT_hind_5 = FT_hind_1 + dung_beetle_pose.at(FT5);*/

    // Set positions
//    positions.at(BC0) = TC_0;//dung_beetle_pose.at(BC0);//TC_0;//
//    positions.at(BC1) = TC_1;//dung_beetle_pose.at(BC1);//TC_1;//
//    positions.at(BC2) = TC_2;//dung_beetle_pose.at(BC2);//TC_2;//
//    positions.at(BC3) = TC_3;//dung_beetle_pose.at(BC3);//TC_3;//
//    positions.at(BC4) = TC_4;//dung_beetle_pose.at(BC4);//TC_4;//
//    positions.at(BC5) = TC_5;//dung_beetle_pose.at(BC5);//TC_5;//
//
//    positions.at(CF0) = CF_0;//dung_beetle_pose.at(CF0);//CF_0;//
//    positions.at(CF1) = CF_1;//dung_beetle_pose.at(CF1);//CF_1;//
//    positions.at(CF2) = CF_2;//dung_beetle_pose.at(CF2);//CF_2;//
//    positions.at(CF5) = CF_5;//dung_beetle_pose.at(CF5);//CF_5;//
//    positions.at(CF4) = CF_4;//dung_beetle_pose.at(CF4);//CF_4;//
//    positions.at(CF3) = CF_3;//dung_beetle_pose.at(CF3);//CF_3;//
//
//    positions.at(FT0) = dung_beetle_pose.at(FT0);
//    positions.at(FT1) = dung_beetle_pose.at(FT1);
//    positions.at(FT2) = dung_beetle_pose.at(FT2);
//    positions.at(FT5) = dung_beetle_pose.at(FT5);
//    positions.at(FT4) = dung_beetle_pose.at(FT4);
//    positions.at(FT3) = dung_beetle_pose.at(FT3);
//
//    positions.at(LONGITUDINAL) = dung_beetle_pose.at(LONGITUDINAL);
//    positions.at(TRANSVERSAL) = dung_beetle_pose.at(TRANSVERSAL);
//    positions.at(HEAD) = dung_beetle_pose.at(HEAD);

    complianceController->saveVector(cpg_signal, cpg_signal_csv, 1);
    complianceController->saveVector(pcpg_signal, pcpg_signal_csv, 1);
    complianceController->saveVector(vrn_signal, vrn_signal_csv, 1);
    complianceController->saveVector(psn_signal, psn_signal_csv, 1);
    complianceController->saveVector(legsearch_signal, legsearch_signal_csv, 1);
    complianceController->saveVector(ballDistanceControlData, ballDistanceControl_csv, 1);
    complianceController->saveVector(rollControlData, rollControl_csv, 1);
    complianceController->saveVector(pitchControlData, pitchControl_csv, 1);
	// cout << "finish save compliant data" << endl;

	positions_target = positions;
	// positions_target.insert(positions_target.begin(), data_counter);
    // complianceController->saveVector(positions_target, pos_desired_csv, 1);

    std::vector<float> err = complianceController->getPosError(positions_target, feedback_positions);
    err.insert(err.begin(), data_counter);
    positions = trimJointMinMax(positions);
	// cout << "before muscle model" << endl;
	if(activate_muscle_model == true){
		// 1. Set desired positions and velocities
		vector<float> pos_desired;
		cout << "positions" << endl;
		for(int i=0; i<21; i++)
		{
			pos_desired.push_back(positions[i]);
			// cout << positions[i] << endl;
		}
		cout << "pos: " << positions[0] << endl;
		cout << "previous_positions: " << previous_positions[0] << endl;

		// 3.2. Set desired velocities
		vector<float> vel_desired;
		cout << "velocity" << endl;
		for(size_t i=0; i<pos_desired.size(); i++)
		{
			float dx_dt = (pos_desired[i] - previous_positions[i])/dt;
			// vel_desired.push_back(dx_dt);
			// cout << dx_dt << endl;
			
			// Constant velocity
			vel_desired.push_back(1); // rad/s
		}
		vel_desired = complianceController->lowPassFilter(0.04, vel_desired, previous_velocities);
		cout << "vel_desired: " << vel_desired[0] << endl;
		cout << "previous_velocities: " << previous_velocities[0] << endl;

		// 3.3. Get feedback
		vector<float> pos_feedback;
		for(int i=0; i<21; i++)
		{
			pos_feedback.push_back(jointPositions[i]);
		}
		
		vector<float> vel_feedback;
		for(int i=0; i<21; i++)
		{
			vel_feedback.push_back(jointVelocities[i]);
		}

		// 4. Calculate torques:
	   vector<float> taus = complianceController->approximateTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive Impedance PD controller
	//    vector<float> taus = complianceController->porportionalTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Adaptive P controller
		//vector<float> taus = pd_c->calculateOutputTorque(pos_feedback, pos_desired, vel_feedback, vel_desired); // Simple impedance PD controller
		// vector<float> taus = complianceController->getWalkingTorque(pos_feedback, pos_desired, vel_feedback, vel_desired);
		vector<float> tau_ = limitTorque(taus);

		// 5. Convert torque into current:
		// cout << "tau_ext" << endl;
		vector<float> tau_ext;
		for(size_t i=0; i<tau_.size(); i++)
		{
			float amps = convertTorque2Current(tau_[i]);
			float mili_amps = amps*1000;
			tau_ext.push_back(mili_amps);
			// cout << tau_[i] << endl;
		}
		cout << "torque" << endl;
		cout << tau_ext[0] << endl;

		// 6. Set positions
		torques.at(BC0) = tau_ext.at(BC0);
		torques.at(BC1) = tau_ext.at(BC1);
		torques.at(BC2) = tau_ext.at(BC2);
		torques.at(BC3) = tau_ext.at(BC3);
		torques.at(BC4) = tau_ext.at(BC4);
		torques.at(BC5) = tau_ext.at(BC5);

		torques.at(CF0) = tau_ext.at(CF0);
		torques.at(CF1) = tau_ext.at(CF1);
		torques.at(CF2) = tau_ext.at(CF2);
		torques.at(CF5) = tau_ext.at(CF5);
		torques.at(CF4) = tau_ext.at(CF4);
		torques.at(CF3) = tau_ext.at(CF3);

		torques.at(FT0) = tau_ext.at(FT0);
		torques.at(FT1) = tau_ext.at(FT1);
		torques.at(FT2) = tau_ext.at(FT2);
		torques.at(FT5) = tau_ext.at(FT5);
		torques.at(FT4) = tau_ext.at(FT4);
		torques.at(FT3) = tau_ext.at(FT3);

		torques.at(LONGITUDINAL) = tau_ext.at(LONGITUDINAL);
		torques.at(TRANSVERSAL) = tau_ext.at(TRANSVERSAL);
		torques.at(HEAD) = tau_ext.at(HEAD);
		// cout << torques.at(HEAD) << endl;

		// 7. Set joint positions
		realRos->updateMotorState(home_names, pos_desired, vel_desired, torques);

		// 8. Update values
	//    TC_0_previous = TC_0_ref;
	//    TC_3_previous = TC_3_ref;
	//    CF_0_previous = CF_0_ref;
	//    CF_3_previous = CF_3_ref;
		previous_positions.clear();
		previous_velocities.clear(); 
		previous_positions = complianceController->copyVector(pos_desired);
		previous_velocities = complianceController->copyVector(vel_desired);

		// save AMC command data
		if(data_counter > lower_limit && data_counter < upper_limit)
		{
			// cout << "save AMC data" << endl;
			vector<float> pos_err = complianceController->pos_error;
			vector<float> vel_err = complianceController->vel_error;
			vector<float> stiff   = complianceController->current_stiffness;
			vector<float> damp    = complianceController->current_damping;
			pos_desired.insert(pos_desired.begin(), data_counter);
			pos_err.insert(pos_err.begin(), data_counter);
			vel_err.insert(vel_err.begin(), data_counter);
			vel_desired.insert(vel_desired.begin(), data_counter);
			tau_.insert(tau_.begin(), data_counter);
			tau_ext.insert(tau_ext.begin(), data_counter);
			complianceController->saveVector(pos_desired, pos_desired_csv, 1);
			complianceController->saveVector(pos_err, pos_error_csv, 1);
			complianceController->saveVector(vel_desired, vel_desired_csv, 1);
			complianceController->saveVector(vel_err, vel_error_csv, 1);
			complianceController->saveVector(tau_, torque_csv, 1);
			complianceController->saveVector(tau_ext, current_csv, 1);
		}
	}
	else{
		if(full_robot){
	//    	cout << "fullrobot publish joint angle" << endl;
	   		cout << "Front_MI : " << front_MI << endl;
			realRos->updateMotorState(home_names, positions, home_velocity, home_torques);
		}
		else{
			realRos->updateMotorState(one_leg_names, oneleg_positions, home_velocity, home_torques);
	//        realRos->updateMotorState(couple_leg_names, couple_positions, home_velocity, home_torques);
		}
	}
	cout << "finish updateMotorState" << endl;
	// Plot Data for visualization
//    plotdata.push_back(fc_sens_raw[0]);
    // plotdata.push_back(down0);
//    plotdata.push_back(oneleg_positions[2]);
	// debudding swing error memory

//    plotdata.push_back(-sw_error[3] * sw_err_fac[3]*1.0 + tau_reflexChain[3].Read(reflex_delay)* sw_err_fac[3]*1.5);
//    plotdata.push_back(sw_err_fac[3]);
//    plotdata.push_back(positions.at(BC3));
//    plotdata.push_back(positions.at(BC3));
	// for(int c = 0; c < 6; c++){ plotdata.push_back(sw_err_fac[c]); }
	// for(int c = 0; c < 6; c++){ plotdata.push_back(fc_error_st[c]); }
	
//    plotdata.push_back(eff[2]);
//    plotdata.push_back(openlsignal[2]);
//    plotdata.push_back(positions.at(CF2));
//    plotdata.push_back(feedback_positions[1]);
//    plotdata.push_back(fcphase[2]);
	
	realRos->dataPlot(plotdata);

    // Update
//    TC_0_previous = TC_0_ref;
//    TC_3_previous = TC_3_ref;
//    CF_0_previous = CF_0_ref;
//    CF_3_previous = CF_3_ref;
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


// ONE-LEG Standing
void dungBeetleController::stand_one_leg()
{
    // 1. Set desired positions and velocities
    vector<float> pos_desired;
    pos_desired.resize(2);
    pos_desired.at(0) = dung_beetle_pose.at(CF5);//jointPositions[0];//dung_beetle_pose.at(CF5);
    pos_desired.at(1) = dung_beetle_pose.at(FT5);//jointPositions[1];//dung_beetle_pose.at(FT5);

    vector<float> vel_desired;
    for(int i=0; i<2; i++)
    {
        vel_desired.push_back(0);
    }

    // 2. Get feedback
    vector<float> pos_feedback;
    for(int i=0; i<2; i++)
    {
        pos_feedback.push_back(jointPositions[i]);
    }
    
    vector<float> vel_feedback;
    for(int i=0; i<2; i++)
    {
        vel_feedback.push_back(jointVelocities[i]);
    }

    // 3. Calculate torques:
    vector<float> taus = complianceController->approximateTorque(pos_feedback, pos_desired, vel_feedback, vel_desired);
    //vector<float> taus = pd_c->calculateOutputTorque(pos_feedback, pos_desired, vel_feedback, vel_desired);
    vector<float> tau_ = limitTorque(taus);
    
    if(initial_counter < upper_limit)
    {
        complianceController->saveVector(pos_desired, pos_desired_csv, 1);
        complianceController->saveVector(vel_desired, vel_desired_csv, 1);
        complianceController->saveVector(taus, torque_csv, 1);
    }

    // 4. Convert torque into current:
    vector<float> tau_ext;
    for(size_t i=0; i<tau_.size(); i++)
    {
        float amps = convertTorque2Current(tau_[i]);
        float mili_amps = amps*1000;
        tau_ext.push_back(mili_amps);
    }

    // 5. Set output vector
    vector<float> torques;
    torques.resize(2);
    torques.at(0) = tau_ext.at(0);
    torques.at(1) = tau_ext.at(1);

    //realRos->setLegMotorTorques(torquesNew);
    realRos->updateMotorState(one_leg_names, pos_desired, vel_desired, torques);
}


//---------------------------------------------------------------------------------------------------------------------------------------------


void dungBeetleController::infoMessage()
{
    printf("------------------------------------------------------------------------------------------------------\n");
    printf("\n"
           "     ______            _        _______    ______   _______  _______ _________ _        _______ \n"
           "    (  __  \\ |\\     /|( (    /|(  ____ \\  (  ___ \\ (  ____ \\(  ____ \\__   __/( \\      (  ____ \\ \n"
           "    | (  \\  )| )   ( ||  \\  ( || (    \\/  | (   ) )| (    \\/| (    \\/   ) (   | (      | (    \\/ \n"
           "    | |   ) || |   | ||   \\ | || |        | (__/ / | (__    | (__       | |   | |      | (__     \n"
           "    | |   | || |   | || (\\ \\) || | ____   |  __ (  |  __)   |  __)      | |   | |      |  __)    \n"
           "    | |   ) || |   | || | \\   || | \\_  )  | (  \\ \\ | (      | (         | |   | |      | (       \n"
           "    | (__/  )| (___) || )  \\  || (___) |  | )___) )| (____/\\| (____/\\   | |   | (____/\\| (____/\\ \n"
           "    (______/ (_______)|/    )_)(_______)  |/ \\___/ (_______/(_______/   )_(   (_______/(_______/ \n"
           "\n"                                                                                 
           "     _______  _______  _       _________ _______  _______  _        _        _______  _______     \n"
           "    (  ____ \\(  ___  )( (    /|\\__   __/(  ____ )(  ___  )( \\      ( \\      (  ____ \\(  ____ )   \n"
           "    | (    \\/| (   ) ||  \\  ( |   ) (   | (    )|| (   ) || (      | (      | (    \\/| (    )|   \n"
           "    | |      | |   | ||   \\ | |   | |   | (____)|| |   | || |      | |      | (__    | (____)|   \n"
           "    | |      | |   | || (\\ \\) |   | |   |     __)| |   | || |      | |      |  __)   |     __)   \n"
           "    | |      | |   | || | \\   |   | |   | (\\ (   | |   | || |      | |      | (      | (\\ (      \n" 
           "    | (____/\\| (___) || )  \\  |   | |   | ) \\ \\__| (___) || (____/\\| (____/\\| (____/\\| ) \\ \\__   \n" 
           "    (_______/(_______)|/    )_)   )_(   |/   \\__/(_______)(_______/(_______/(_______/|/   \\__/   \n");
    printf("------------------------------------------------------------------------------------------------------\n");

    printf("\n"
        "**************************     ADAPTIVE COMPLIANCE CONTROL      ************************** \n"
        "\n");
    
    printf("Joint data is saved to *.csv files for all robot joints. \n");
    printf("Use ROS' rqt_plot pkg for live plots of data. \n");
    printf("Example: \n");
    printf("rosrun rqt_plot rqtplot /db_ROS_dynamixel_driver/hexapod_joint_feedback/position[11]");
    printf("Joints are numbered as follows: \n");
    printf("0:TC2   1: CF2  2:FT2   3:TC5   4: CF5  5:FT5   6:TC1   7: CF1  8:FT1 \n");
    printf("9:TC4   10: CF4  11:FT4   12:TC0   13: CF0  14:FT0   15:TC3   16: CF3  17:FT3 \n");
    printf("18: REAR ABDOMEN  19: BACKBONE  20:HEAD   \n");
}

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

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Looping standing and walking when running the controller
// modifies lower_limit and upper_limit variables for tuning stanidn and walking cycle
void dungBeetleController::standAndWalk(){
	// Standing + walking
	if(initial_counter > lower_limit && initial_counter < upper_limit)
	{
		if(initial_counter == lower_limit+1)
		{
			cout << "Walking for " << upper_limit-lower_limit << " cycles." << endl;
		}
		cout << "actuate walking" << endl;
		actuateRobot_walkingPosition(); // If you want Just standing tests comment This  line ***
		//actuateRobot_walkingTorque();
	}
	else
	{
		cout << "actuate stand" << endl;
		actuateRobot_standPosition(1);
		//actuateRobot_standTorque();
		if(initial_counter == lower_limit-50)
		{
			complianceController->setConstantCoefficients();
			cout << "Set K and F." << endl;
		}
	}// *** Until here */
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


void dungBeetleController::standAndWalkTorque(){
	cout << "Actuate Stand and Walk Torque Controller" << endl;
	// Standing + walking
	// If you want Just standing tests comment these lines ***
	if(initial_counter > lower_limit && initial_counter < upper_limit)
	{
		if(initial_counter == lower_limit+1)
		{
			cout << "Walking for " << upper_limit-lower_limit << " cycles." << endl;
		}
//		actuateRobot_walkingPosition();
		actuateRobot_walkingTorque();
	}
	else
	{
//		actuateRobot_standPosition();
		actuateRobot_standTorque();
		if(initial_counter == lower_limit-50)
		{
			complianceController->setConstantCoefficients();
			cout << "Set K and F." << endl;
		}
	}// *** Until here */
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


// POSITION CONTROLLER - Standing
void dungBeetleController::actuateRobot_Rolling_pose_Position()
{

	fc_sens_raw[0] = jointTorques[13];
	fc_sens_raw[1] = jointTorques[7];
	fc_sens_raw[2] = jointTorques[1];
	fc_sens_raw[3] = jointTorques[16];
	fc_sens_raw[4] = jointTorques[10];
	fc_sens_raw[5] = jointTorques[4];

	cout << "Standing" << endl;
	cout << "1: " << fc_sens_raw[0] << " 2: " << fc_sens_raw[1] <<
			" 3: " << fc_sens_raw[2] << " 4: " << fc_sens_raw[3] <<
			" 5: " << fc_sens_raw[4] << " 6: " << fc_sens_raw[5] << endl;

    for(size_t i=0; i<dung_beetle_rolling_pose.size(); i++)
    {
        positions.at(i) = dung_beetle_rolling_pose[i];
    }

//    complianceController->saveVector(positions, pos_desired_csv, 1);
//    vector<float> err = complianceController->getPosError(positions, jointPositions);

    //realRos->setLegMotorPosition(positionsNew);
    if(full_robot){
    	realRos->updateMotorState(home_names, positions, home_velocity, home_torques);
    }
    else{
    	//pass
    }
    // Update
//    TC_0_previous = positions.at(BC0);
//    TC_3_previous = positions.at(BC3);
//    CF_0_previous = positions.at(CF0);
//    CF_3_previous = positions.at(CF3);
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


void dungBeetleController::standAndRoll(){
	// Standing + walking
	// If you want Just standing tests comment these lines ***
	if(initial_counter > lower_limit && initial_counter < upper_limit)
	{
		if(initial_counter == lower_limit+1)
		{
			cout << "Rolling for " << upper_limit-lower_limit << " cycles." << endl;
		}
		cout << "actuate rolling" << endl;
		actuateRobot_walkingPosition();
		//actuateRobot_walkingTorque();
	}
	else
	{
		cout << "actuate stand" << endl;
		// actuateRobot_Rolling_pose_Position();
		// actuateRobot_standTorque();
		actuateRobot_standPosition(1);
		if(initial_counter == lower_limit-50)
		{
			complianceController->setConstantCoefficients();
			cout << "Set K and F." << endl << "   ready to Roll   " << endl;
		}
	}// *** Until here */
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


void dungBeetleController::knock(){
	// Standing + walking
	// If you want Just standing tests comment these lines ***
	for(int i = 0; i < 100; i++){
//		if(i == 0 or i == 20 or i == 40){
			err_spike = 2;
//		}
		err_sp = 0.9 * olderr_sp + 0.1 * err_spike;
		olderr_sp = err_sp;
		cout << err_sp << endl;

		oneleg_knock.resize(3);

		oneleg_knock.at(0) = dung_beetle_pose.at(BC3) + err_sp;
		oneleg_knock.at(1) = dung_beetle_pose.at(CF3) - err_sp;
		oneleg_knock.at(2) = dung_beetle_pose.at(FT3) + err_sp;

		realRos->updateMotorState(one_leg_names_knock, oneleg_knock, home_velocity, home_torques);
	}
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

vector<float> dungBeetleController::trimJointMinMax(vector<float> array){

	for (int i=0; i<array.size(); i++){
		if (array.at(i) < joint_min[i]){
			array.at(i) = joint_min[i];
		}
		else if (array.at(i) > joint_max[i]){
			array.at(i) = joint_max[i];
		}
	}
	return array;
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

void dungBeetleController::updateJointFeedback(){
	joint_state = realRos->getJointFeedback();

    jointPositions.clear();
    jointVelocities.clear();
    jointTorques.clear();

    int index = 0;
    for(std::string name : joint_state.name)
    {
        //jointIDs.push_back(name); 
        jointPositions.push_back(joint_state.position[index]);
        jointVelocities.push_back(joint_state.velocity[index]);
        jointTorques.push_back(joint_state.effort[index]);
	    std::cout << "joint pos motor" << index <<  ": " << jointPositions[index] << std::endl;
	    std::cout << "joint vel motor" << index <<  ": " << jointVelocities[index] << std::endl;
	    std::cout << "joint Torque motor" << index <<  ": " << jointTorques[index] << std::endl;
        index++;
    }
    // std::cout << "joint feedback Position motor_1 : " << jointTorques[0] << std::endl;
}

void dungBeetleController::updateImuFeedback(){
	imuData = realRos->getImuFeedback();
	angle[0] = imuData[10];
	angle[1] = imuData[11];
	angle[2] = imuData[12];

	omega[0] = imuData[0];
	omega[1] = imuData[1];
	omega[2] = imuData[2];
}
