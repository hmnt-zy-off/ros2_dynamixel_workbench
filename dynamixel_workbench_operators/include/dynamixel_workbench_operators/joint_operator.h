/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */
/* Modified by: Hemand Sunny  */

#ifndef DYNAMIXEL_WORKBENCH_OPERATORS_H
#define DYNAMIXEL_WORKBENCH_OPERATORS_H

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <yaml-cpp/yaml.h>

// #include <trajectory_msgs/JointTrajectory.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// #include <trajectory_msgs/JointTrajectoryPoint.h>
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// #include <std_srvs/Trigger.h>
#include "std_srvs/srv/trigger.hpp"

class JointOperator : public rclcpp::Node
{
 private:
  // ROS NodeHandle
//   ros::NodeHandle node_handle_;
//   ros::NodeHandle priv_node_handle_;

  // ROS Parameters
  bool is_loop_;

  // ROS Topic Publisher
//   ros::Publisher joint_trajectory_pub_;
rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_trajectory_pub_;

  // ROS Topic Subscriber

  // ROS Service Server
//   ros::ServiceServer move_command_server_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_command_server_;


  // ROS Service Client

//   trajectory_msgs::JointTrajectory *jnt_tra_msg_;
 std::shared_ptr<trajectory_msgs::msg::JointTrajectory> jnt_tra_msg_;


 public:
  JointOperator();
  ~JointOperator();

  bool isLoop(void){ return is_loop_;}


 bool getTrajectoryInfo(
    const std::string & yaml_file,
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> jnt_tra_msg);

  void moveCommandMsgCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);



  };

#endif // DYNAMIXEL_WORKBENCH_OPERATORS_H
