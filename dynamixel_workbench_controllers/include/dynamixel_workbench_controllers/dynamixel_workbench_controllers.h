/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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
/* Modified by: Hemand Sunny */

#ifndef DYNAMIXEL_WORKBENCH_CONTROLLERS_H
#define DYNAMIXEL_WORKBENCH_CONTROLLERS_H

#include <memory>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <yaml-cpp/yaml.h>

// #include <sensor_msgs/JointState.h>
#include <sensor_msgs/msg/joint_state.hpp> 

// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/msg/twist.hpp>

// #include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// #include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

// #include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/msg/dynamixel_state_list.hpp>

// #include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/srv/dynamixel_command.hpp>

#include <dynamixel_workbench_controllers/trajectory_generator.h>

#include <mutex>
std::mutex dxl_mutex_;

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define DEBUG

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController : public rclcpp::Node
{
 private:
 
 // ROS Topic Publisher
  // ros::Publisher dynamixel_state_list_pub_;
  rclcpp::Publisher<dynamixel_workbench_msgs::msg::DynamixelStateList>::SharedPtr dynamixel_state_list_pub_;

  // ros::Publisher joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // ROS Topic Subscriber
  // ros::Subscriber cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // ros::Subscriber trajectory_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;

  // ROS Service Server
  // ros::ServiceServer dynamixel_command_server_;
  rclcpp::Service<dynamixel_workbench_msgs::srv::DynamixelCommand>::SharedPtr dynamixel_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  // DynamixelWorkbench *dxl_wb_;
  std::unique_ptr<DynamixelWorkbench> dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;

  dynamixel_workbench_msgs::msg::DynamixelStateList dynamixel_state_list_;
  sensor_msgs::msg::JointState joint_state_msg_;
  std::vector<WayPoint> pre_goal_;

  bool is_joint_state_topic_;
  bool is_cmd_vel_topic_;
  bool use_moveit_;

  double wheel_separation_;
  double wheel_radius_;

  // JointTrajectory *jnt_tra_;
  std::unique_ptr<JointTrajectory> jnt_tra_;
  trajectory_msgs::msg::JointTrajectory jnt_tra_msg_;

  double read_period_;
  double write_period_;
  double pub_period_;

  bool is_moving_;

 public:
  DynamixelController();
  ~DynamixelController();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod(){return read_period_;}
  double getWritePeriod(){return write_period_;}
  double getPublishPeriod(){return pub_period_;}

  void initPublisher(void);
  void initSubscriber(void);

  void initServer();

  void readCallback();
  void writeCallback();
  void publishCallback();

  void commandVelocityCallback(std::shared_ptr<const geometry_msgs::msg::Twist> msg); //const geometry_msgs::msg::Twist::SharedPtr &msg
  void trajectoryMsgCallback(
  std::shared_ptr<const trajectory_msgs::msg::JointTrajectory> msg);
  bool dynamixelCommandMsgCallback( const std::shared_ptr<dynamixel_workbench_msgs::srv::DynamixelCommand::Request> req, std::shared_ptr<dynamixel_workbench_msgs::srv::DynamixelCommand::Response> res);
};

#endif //DYNAMIXEL_WORKBENCH_CONTROLLERS_H
