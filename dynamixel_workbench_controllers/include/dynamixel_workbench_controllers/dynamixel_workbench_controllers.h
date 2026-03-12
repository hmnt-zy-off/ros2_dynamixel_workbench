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
#include <string>
#include <mutex>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"


#include <yaml-cpp/yaml.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "lifecycle_msgs/msg/transition.hpp"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/msg/dynamixel_state_list.hpp>
#include <dynamixel_workbench_msgs/srv/dynamixel_command.hpp>

#include <dynamixel_workbench_controllers/trajectory_generator.h>

#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

using LCL_RET = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController : public rclcpp_lifecycle::LifecycleNode
{
private:
  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<dynamixel_workbench_msgs::msg::DynamixelStateList>::SharedPtr dynamixel_state_list_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;

  // Service
  rclcpp::Service<dynamixel_workbench_msgs::srv::DynamixelCommand>::SharedPtr dynamixel_command_server_;

  // Timers
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Dynamixel
  std::unique_ptr<DynamixelWorkbench> dxl_wb_;
  std::unique_ptr<JointTrajectory> jnt_tra_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;

  dynamixel_workbench_msgs::msg::DynamixelStateList dynamixel_state_list_;
  sensor_msgs::msg::JointState joint_state_msg_;
  trajectory_msgs::msg::JointTrajectory jnt_tra_msg_;
  std::vector<WayPoint> pre_goal_;
  
  bool auto_start_;
  bool is_joint_state_topic_;
  bool is_cmd_vel_topic_;
  bool use_moveit_;
  bool is_moving_;

  double wheel_separation_;
  double wheel_radius_;

  double read_period_;
  double write_period_;
  double pub_period_;

  std::mutex dxl_mutex_;
  std::atomic<bool> is_shutdown_{false};

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr read_cb_group_;
  rclcpp::CallbackGroup::SharedPtr write_cb_group_;
  rclcpp::CallbackGroup::SharedPtr pub_cb_group_;

public:
  DynamixelController();
  ~DynamixelController();

  // Lifecycle
  LCL_RET on_configure(const rclcpp_lifecycle::State & state) override;
  LCL_RET on_activate(const rclcpp_lifecycle::State & state) override;
  LCL_RET on_deactivate(const rclcpp_lifecycle::State & state) override;
  LCL_RET on_cleanup(const rclcpp_lifecycle::State & state) override;
  LCL_RET on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Init
  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod()    { return read_period_; }
  double getWritePeriod()   { return write_period_; }
  double getPublishPeriod() { return pub_period_; }

  void initPublisher(void);
  void initSubscriber(void);
  void initServer(void);

  // Callbacks
  void readCallback(void);
  void writeCallback(void);
  void publishCallback(void);

  void commandVelocityCallback(std::shared_ptr<const geometry_msgs::msg::Twist> msg);
  void trajectoryMsgCallback(std::shared_ptr<const trajectory_msgs::msg::JointTrajectory> msg);
  bool dynamixelCommandMsgCallback(
    const std::shared_ptr<dynamixel_workbench_msgs::srv::DynamixelCommand::Request> req,
    std::shared_ptr<dynamixel_workbench_msgs::srv::DynamixelCommand::Response> res);
};

#endif  // DYNAMIXEL_WORKBENCH_CONTROLLERS_H