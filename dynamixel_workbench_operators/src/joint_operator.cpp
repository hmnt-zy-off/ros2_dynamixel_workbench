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
/* Modified by: Hemand Sunny */

#include "dynamixel_workbench_operators/joint_operator.h"
using std::placeholders::_1;
using std::placeholders::_2;

JointOperator::JointOperator() : Node("joint_operator"),is_loop_(false)
{
  this->declare_parameter<std::string>("trajectory_info","");
  this->declare_parameter<bool>("is_loop",false);

  std::string yaml_file = this->get_parameter("trajectory_info").as_string();
  is_loop_ = this->get_parameter("is_loop").as_bool();

  jnt_tra_msg_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);

  if (result == false)
  {
    RCLCPP_ERROR(this->get_logger(),"Please check YAML file.");
    rclcpp::shutdown();
    return;
  }

  joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory",10);
  move_command_server_ = this->create_service<std_srvs::srv::Trigger>("execution",std::bind(&JointOperator::moveCommandMsgCallback,this,_1,_2));

}

JointOperator::~JointOperator()
{
  // destructor
}

bool JointOperator::getTrajectoryInfo(const std::string & yaml_file, std::shared_ptr<trajectory_msgs::msg::JointTrajectory> jnt_tra_msg)
{
  YAML::Node file = YAML::LoadFile(yaml_file.c_str());
  if (!file) return false;
  YAML::Node joint = file["joint"];
  uint8_t joint_size = joint["names"].size();

  for (uint8_t i=0;i<joint_size;i++)
  {
    jnt_tra_msg->joint_names.push_back(joint["names"][i].as<std::string>());
  }

  YAML::Node motion = file["motion"];
  uint8_t motion_size = motion["names"].size();

  for (uint8_t i=0;i<motion_size;i++)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;

    std::string name = motion["names"][i].as<std::string>();
    YAML::Node motion_name = motion[name];

    if (joint_size!=motion_name["step"].size())
    {
      RCLCPP_ERROR(this->get_logger(),"Motion step size must equal joint size");
      return false;
    }

    for (uint8_t j=0;j< joint_size;j++)
    {
      double val = motion_name["step"][j].as<double>();
      point.positions.push_back(val);
      RCLCPP_INFO(this->get_logger(),"motion_name : %s, step : %f", name.c_str(),val);
    }
    if (!motion_name["time_from_start"])
    {
      RCLCPP_ERROR(this->get_logger(),"time_from_start missing");
      return false;
    }
    point.time_from_start = rclcpp::Duration::from_seconds(motion_name["time_from_start"].as<double>());
    jnt_tra_msg->points.push_back(point);
  }
  return true;

}

void JointOperator::moveCommandMsgCallback( const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  joint_trajectory_pub_->publish(*jnt_tra_msg_);
  res->success =true;
  res->message = "Success to publish joint trajectory";
}

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<JointOperator>();

  RCLCPP_INFO(node->get_logger(), "Trigger /execution service to publish trajectory");

  if (node->isLoop())
  {
    rclcpp::Rate rate(1);
    while(rclcpp::ok())
    {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
      node->moveCommandMsgCallback(req,res);
      rate.sleep();
    }
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}