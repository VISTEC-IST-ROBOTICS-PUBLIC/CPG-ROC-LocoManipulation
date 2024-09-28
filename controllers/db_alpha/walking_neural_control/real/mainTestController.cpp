
#include <cstdio>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "realRos2Class.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "Run db_beta_controller" << std::endl;

  auto db_beta_controller = std::make_shared<realRos2Class>();

  int ros_rate = 60; // Hz
  rclcpp::Rate loop_rate(ros_rate);
  std::cout << "init real ros2 class node" << std::endl;
  // auto end = std::chrono::system_clock::now();
  // rclcpp::spin(db_beta_controller);
  float o1 = 0.01;
  float o2 = 0.01;
  
  while(rclcpp::ok()){
    o1 = o1*1.4 + 0.18*o2;
    o2 = o2*1.4 - 0.18*o1;
    o1 = tanh(o1);
    o2 = tanh(o2);
    
    std::cout << "o1: " << o1 << "   o2: " << o2 << std::endl;

    // Start timing
    // auto start = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds = abs(end-start);
    // std::cout << "elapsed_seconds: " << elapsed_seconds.count() << std::endl;
    // // Some computation here
    // auto end = std::chrono::system_clock::now();
    // std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    // Subscribe jointstate
    db_beta_controller->getRobotFeedback();

    // Publish jointstate
    std::vector<std::string> names;
    std::vector<float> positions;
    std::vector<float> velocities;
    std::vector<float> torques;
    std::vector<float> dataPlot;
    for(int i =0; i<1; i++){
      velocities.push_back(0);
      torques.push_back(0);
    }
    names.push_back("motor_21");
    positions.push_back(o1);
    db_beta_controller->updateMotorState(names, positions, velocities, torques);
    dataPlot.push_back(db_beta_controller->angle[0]);
    dataPlot.push_back(db_beta_controller->angle[1]);
    dataPlot.push_back(db_beta_controller->angle[2]);
    db_beta_controller->dataPlot(dataPlot);
    // rclcpp::spin(db_beta_controller);
    rclcpp::spin_some(db_beta_controller);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
