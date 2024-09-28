# Integrate Modular Neural Control with Adaptive Muscle-Model Controller 

This project are modified from Carlos Viescas Huerta's project muscle\_control project. And Thank you for a very nice interface.

## 1. Simulation robot project 

#### 1.1. V-REP Simulation set up.

##### 1.2.1. Compilation and running.

Standard installation path:

```
$ cd /home/user/
$ mkdir workspace
$ git clone <gorobots_repository>
```

To compile:

```sh
$ cd gorobots/projects/db_alpha/walking_neural_control/sim/catkin_ws
$ catkin clean # just the first time after cloning
$ catkin build
```

To run it:
 - Open V-REP and select the ALPHA\_muscle\_test.ttt scene in utils/v-rep\_simulations/alpha\_dungbeetle\_robot. 
 - Then, open the child script of the Alpha\_Base (the icon that resembles a paper sheet) and set up the path (_sim.include()_) pointing to the LUA script walking\_neural\_control\_script.lua. 
 - After that, open the walking\_neural\_control\_script.lua script; go to the line with the function call:

```lua
-- Start the client application (c++ node)
result=sim.launchExecutable('path/to/executable/', MotorTopicName.." "..simulationTimeTopicName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..CPGOutputName.." "..jointPositionsName.." "..testParametersName.." "..SimulationType,0)
```

And add the path to the controller's executable, that was generated after calling _catking build_ in the compilation step. The executable should have been named **walking\_neural\_controller**.

 - Check that in **_dungBeetleController.h_** the variable **_isSimulation_** is set to true (line 96).
 - Finally, just press the play button in V-REP.

##### 1.2.2. Dynamic Simulation.

To make the robot walk on the floor (not just magically floating on the air), first open the .ttt scene. 

Them, on the left menu, go to the **Scene hierarchy** and double click on the icon of the **Alpha\_Base**. A pop-up window will appear. On it, press the button **Show dynamic properties dialog**. Finally, on the new window opened, activate the checkbox that says **Body is dynamic**. 

##### 1.2.3. Saving feedback data.

In the **_AMC_** and **_dungBeetleController_** source files there are some file streaming functions which require changing the paths to the files. 
 

## 2. REAL ROBOT

#### 2.1. Dynamixel ROS drivers.

There are 2 versions of the dynamixel drivers:

 - If you use Protocol 1.0 follow [Mathias' guide](https://github.com/MathiasThor/my_dynamixel_workbench/wiki/MORF-Software-Installation-Guide) (easier set up of the position controller).
 - If you use Protocol 2.0 follow [this guide](https://github.com/CVH95/db_alpha_interface) to download, install and set up the drivers (improved for torque control).

 Depending on the protocol the controller would need either _std\_msgs::Float34MultiArray_ or _sensor\_msgs::JointState_ ROS messages. This requires commenting and uncommenting some lines in both **_realRosClass_** source files and the source files of the [db\_alpha\_controller drivers](https://github.com/CVH95/db_alpha_interface).  

#### 2.2. Running the controller:

Add the following alias to `~/.bashrc`:

```sh
# Remember to change the paths
$ echo "alias dungBeetle_controller='cd /home/user/workspace/gorobots/projects/db_alpha/walking_neural_control/real/catkin_ws/src/walking_neural_controller/bin && ./walking_neural_controller'" >> ~/.bashrc
$ echo "alias dungBeetle_build='cd /home/user/workspace/gorobots/projects/db_alpha/walking_neural_control/real/catkin_ws/ && catkin build'" >> ~/.bashrc
$ echo "alias dungBeetle_RQTplot='cd /home/user/workspace/gorobots/projects/db_alpha/walking_neural_control/real/catkin_ws/src/walking_neural_controller/bin && ./online_ros_plot.sh'" >> ~/.bashrc
```
If Running the Denmark version of db_alpha robot 
please go to controllers/db_alpha/real/dungBeetleController.h and set the bool isDenmark_db_alpha = true;
Please also check the dungbeetle_pose in dungBeetleController.h

To make a robot perform stand and walking, in controllers/db_alpha/real/dungBeetleController.cpp in the runController 
by comment or uncomment the actuateRobot_walkingPosition();, actuateRobot_walkingTorque();, actuateRobot_standTorque();, actuateRobot_standPosition();
The robot can Stand or walking in a sequence.  each cycle robot will stand and walk for a while.

To tune the amplitude of the modular neural control
goto the actuateRobot_walking Position or Torque
modify the variable *fac* 

Start the dynamixel driver interface. Once you are connected to the robot:

```sh
$ dungBeetle_build # to compile
$ dungBeetle_controller # to run the controller
$ dungBeetle_RQTplot # to run RQT plot and see live plots of data
```


## 3. V-REP Simulation 

**IMPORTANT NOTE:**

By default, the .ttt V-REP model of the dung beetle comes in Position Control. Always keep a backup of this original file. If configuration changes are made (like changing the control mode or joint mode or enabling dynamic bodies), **NEVER SAVE THOSE CHANGES**. Create a new .ttt file. 

#### 3.1. Controllers.

Two modalities: 

 - Position Control (PID Contoller).
 - Torque Control (muscle model - Adaptive Motor Controller). 

To select the control mode comment/uncomment in lines 10-12 in _**walking\_neural\_control\_script.lua**_ script.

```lua
-- SET UP POSITION / TORQUE CONTROL MODE 
PorT=false -- Closed-loop Torque Control (muscle model) implementing the Adaptive Motor Controller
--PorT=true -- Closed-Loop CPG-driven Position PID Controller adapted for the dung beetle.
```

#### 3.2. Control Modes in V-REP simulations. 

##### 3.2.1. Joint Modes:

 - **Passive mode**: in this mode the joint is not directly controlled and will act as a fixed link. The user can however change the joint's position with appropriate API function calls (e.g. sim.setJointPositon or sim.setSphericalJointMatrix).
 - **Inverse kinematics mode**: in this mode, the joint acts as a passive joint, but is used (adjusted) during inverse kinematics calculations.
 - **Dependent mode**: in this mode, the joint position is directly linked (dependent) to another joint position through a linear equation.
 - **Motion mode**: _this mode is deprecated_ and should not be used anymore. A similar and more flexible behaviour can be obtained with the passive mode and a child script appropriately updating the joint.
 - **Torque or force mode**: in this mode, the joint is simulated by the dynamics module, if and only if it is dynamically enabled (refer to the section on designing dynamic simulations for more information). When dynamically enabled, a joint can be free or controlled in Force/torque, in velocity or in position. Screws cannot be operated in torque or force mode (however it is possible to obtain a similar behaviour by linking a revolute and prismatic joint programmatically), and spherical joints can only be free in torque or force mode.
    - When the **joint motor is disabled**, the joint is free and only constrained by its limits.
    - When the **joint motor is enabled** and the **control loop is disabled**, then the joint will try to reach the desired target velocity given the maximum torque/force it is capable to deliver. When that maximum torque/force is very high, the target velocity is instantaneously reached and the joint operates in velocity control, otherwise it operates at the specified torque/force until the desired target velocity is reached (torque/force control).
    - When both the **joint motor and the control loop are enabled**, then the user has 3 control modes available:
        - _Custom control_: a joint callback function will be in charge of controlling the dynamic behaviour of the joint, allowing you to control the joint with any imaginable algorithm.
        - _Position control (PID)_: the joint will be controlled in position via a PID controller that will adjust the joint velocity and enforces a constante Force/Torque.
        - _Spring-damper mode_: the joint will act like a spring-damper system via a force/torque modulation and constant velocity.

##### 3.2.2. Joint Control Modes:

If the joint operates in **force/torque mode** and is **dynamically enabled**, then it will be indirectly handled by the physics engine. If your joint's motor is not enabled, then your joint is not controlled (i.e. it will be free). Otherwise, your joint can be in following two dynamic modes:

 - **The joint motor is enabled, but the control loop is disabled**. Use this mode when you want to precisely custom control your joint **from an external application** (e.g. force/torque control, PID, etc.). Use this mode also, when you want to loosely control your joint in force/torque mode, or for velocity control (e.g. robot wheel motors).
 - **The joint motor is enabled, and the control loop is enabled**. Use this mode when your joint needs to act as a spring/damper, or if you want to precisely custom control your joint from within V-REP, or if you want to loosely control your joint in position control from an external application.

**If your joint motor is enabled, but the control loop is disabled**, then the physics engine will apply the **specified Maximum force/torque**, and accelerate the joint **until the target velocity is reached**. If the load is small and/or the maximum force/torque high, that target velocity will be reached quickly. Otherwise, it will take some time, or, if the force/torque is not large enough, the target velocity will never be reached. You can programmatically adjust the target velocity with **_sim.setJointTargetVelocity_** , and the maximum force/torque with **_sim.setJointForce_**. You should be very careful before writing a precise joint controller for a joint in force/torque mode from a child script for following reason:

By default, the simulation loop runs with a time step of 50ms (in simulation time). But the physics engine will run with a time step of 5ms, i.e. 10 times more often. A child script will be called in each simulation step, but not in each physics engine calculation step. This means that if you control a joint from a child script in a regular way, you will only be able to provide new control values once for 10 physics engine calculation steps: you will be missing 9 steps. One way to overcome this would be to change the default simulation settings and to specify a simulation time step of 5ms, instead of 50ms. This works fine, but remember that all other calculations (e.g. vision sensors, proximity sensors, distance calculations, IK, etc.) will also run 10 times more often, and finally slow down your simulation (most of the time you won't need such a high refresh rate for the other calculation modules. But the physics engine requires such a high refresh rate). Another, much better option, would be to use a joint callback function (or a dynamics callback function) as will be explained further down.

If, one the other hand, you want to run a precise and regular joint controller externally (e.g. from a remote API client, a ROS node or a BlueZero node), then you have no other option than to set the simulation loop to the same rate as the physics engine rate, then run V-REP in synchronous mode, and the external controller (e.g. the remote API client) will have to trigger each simulation step explicitely. 

Notes taken from [Coppelia Robotics' Manual on V-REP](http://www.coppeliarobotics.com/helpFiles/en/jointDescription.htm#controller).

#### 3.3. Setting up the Position Controller in V-REP.

In `utils/v-rep_simulations/db_alpha/`, the simulation **_db_position_default.ttt_** comes with the default settings to run the position controller. Make sure that the main **LUA** script is in position control before running it. 

##### 3.3.1. Setting joints in Force/Torque mode for position control.

Double click on the joint icon (icon next to TC/CF/FT joint name). A window will appear; select **Force/Torque mode** in the tab inside mode section in the window. Then, click on **Show dynamic properties dialog** and check the box that says **_control loop enabled_**. Then, select **_Position Control (PID)_** as controller model. 

#### 3.4. Setting up the Torque Controller in V-REP.

##### 3.4.1. Setting joints in Force/Torque mode.

Double click on the joint icon (icon next to TC/CF/FT joint name). A window will appear; select **Force/Torque mode** in the tab inside mode section in the window. Then, click on **Show dynamic properties dialog** and uncheck the box that says _**control loop enabled**_.

##### 3.4.2. Lowering the simulation time step.



##### 3.4.3. Using target velocities for torque control.

Torque control of joints is not straight forward in V-REP. Joints admit a **maximum torque** value, which represents the maximum torque applied to that joint; and a **target velocity** value, which is the velocity that the joint is expected to reach. If the current velocity of the joint is below the target velocity, the maximum torque will be applied. Therefore, to control joints by torque, it is required to **set the target velocity to a very high (unreachable) value** and modulate the joint with the maximum torque applied. This is done in the LUA interface script with the functions:

```lua
sim.setJointTargetVelocity(motor_id,V_value)
sim.setJointJointForce(motor_id,T_value)
```

It is important to remark that V-REP does not allow to use negative torque values. Therefore, before sending torques and velocities, it is necessary to implement a function that evaluates the sign of the torque to be applied. If it is negative, then the target velocity sent must be negative, and the torque sent must be its absolute value.