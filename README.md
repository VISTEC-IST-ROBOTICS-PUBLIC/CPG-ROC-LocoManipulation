# CPG-ROC-Locomanipulation
CPG-based Loco-manipulation control with robot orientation control of dung beetle-like robot

- In order to run the simulation, it's required to have ROS2, CoppeliaSim.
- The code is written in C++.
- We have tested pur system on Ubuntu 24.04.1. The older Version such as 22, 20 should also work. 

- Clone the repository and navigate to the folder **CPG-ROC-LocoManipulation_main/projects/db_alpha/walking_neural_control/sim/ros2_ws**

use **colcon build** to build the package
```
colcon build
```

- open the simulation scenes in folder scene/dbAlpha_Rolling_Spere_small_ball

- run the simulation

# Quadruped Cylinder Rolling

- To run the Quadruped Cylinder Rolling Scene, it is required to use the Coppelia Simulation. The simulation is tested on Ubuntu 24.04.1 with Coppelia Version 4.7.

- The code is written in the scene file with Lua programming language.

- Open the Quadruped Cylinder Rolling Scene scene file located in the scenes folder.

# Throubleshooting
- if there is an error to run the scene consider install some dependencies as follows:

```
apt install pyzmq python3-cbor2
```