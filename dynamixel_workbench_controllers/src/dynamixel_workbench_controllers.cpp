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

#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"
using namespace std::chrono_literals;

DynamixelController::DynamixelController()
: rclcpp_lifecycle::LifecycleNode("dynamixel_workbench_controllers"),
  is_joint_state_topic_(false),
  is_cmd_vel_topic_(false),
  use_moveit_(false),
  wheel_separation_(0.0f),
  wheel_radius_(0.0f),
  is_moving_(false)
{
  RCLCPP_WARN(this->get_logger(), "Unconfigured Controller");

  this->declare_parameter<bool>("autostart", false);
  this->declare_parameter("use_joint_states_topic", true);
  this->declare_parameter("use_cmd_vel_topic", false);
  this->declare_parameter("use_moveit", false);
  this->declare_parameter("dxl_read_period", 0.010);
  this->declare_parameter("dxl_write_period", 0.010);
  this->declare_parameter("publish_period", 0.010);
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 57600);
  this->declare_parameter<std::string>("dynamixel_info", "");

  // in constructor or on_configure
read_cb_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
write_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
pub_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

}

DynamixelController::~DynamixelController()
{
  // RCLCPP_DEBUG(this->get_logger(), "*** Destructor ***");
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

LCL_RET DynamixelController::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "Configuring...");

  is_joint_state_topic_ = this->get_parameter("use_joint_states_topic").as_bool();
  is_cmd_vel_topic_     = this->get_parameter("use_cmd_vel_topic").as_bool();
  use_moveit_           = this->get_parameter("use_moveit").as_bool();
  read_period_          = this->get_parameter("dxl_read_period").as_double();
  write_period_         = this->get_parameter("dxl_write_period").as_double();
  pub_period_           = this->get_parameter("publish_period").as_double();

  if (is_cmd_vel_topic_)
  {
    this->declare_parameter("mobile_robot_config.seperation_between_wheels", 0.0);
    this->declare_parameter("mobile_robot_config.radius_of_wheel", 0.0);
    wheel_separation_ = this->get_parameter("mobile_robot_config.seperation_between_wheels").as_double();
    wheel_radius_     = this->get_parameter("mobile_robot_config.radius_of_wheel").as_double();
  }

  dxl_wb_  = std::make_unique<DynamixelWorkbench>();
  jnt_tra_ = std::make_unique<JointTrajectory>();


  std::string port_name = this->get_parameter("port_name").as_string();
  int         baud_rate = this->get_parameter("baud_rate").as_int();
  std::string yaml_file = this->get_parameter("dynamixel_info").as_string();

  if (!initWorkbench(port_name, (uint32_t)baud_rate))
  {
    RCLCPP_ERROR(this->get_logger(),"Please check USB port name");
    return LCL_RET::FAILURE;
  }
  if (!getDynamixelsInfo(yaml_file))
  {
    RCLCPP_ERROR(this->get_logger(),"Please check YAML file");
    return LCL_RET::FAILURE;
}
  if (!loadDynamixels())
  {
    RCLCPP_ERROR(this->get_logger(),"Please check Dynamixel ID or BaudRate");
    return LCL_RET::FAILURE;
}
  
  if (!initDynamixels())
  {
    RCLCPP_ERROR(this->get_logger(),"Please check control table (http://emanual.robotis.com/#control-table)");
    return LCL_RET::FAILURE;
}
  
  if (!initControlItems())
  {
    RCLCPP_ERROR(this->get_logger(),"Please check control items");
    return LCL_RET::FAILURE;
  }
    
  if (!initSDKHandlers())
  {
    RCLCPP_ERROR(this->get_logger(),"Failed to set Dynamixel SDK Handler");
    return LCL_RET::FAILURE;
  }
    

  initPublisher();
  initSubscriber();
  initServer();

  RCLCPP_INFO(this->get_logger(), "Configured Successfully");
  return LCL_RET::SUCCESS;
}

LCL_RET DynamixelController::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "Activating...");
  // Enable torque on all motors
  for (auto const& dxl : dynamixel_)
    dxl_wb_->torqueOn((uint8_t)dxl.second);

  dynamixel_state_list_pub_->on_activate();
  if (is_joint_state_topic_) joint_states_pub_->on_activate();

  read_timer_    = this->create_timer(std::chrono::duration<double>(read_period_),
                    [this]() { this->readCallback(); },read_cb_group_);
  write_timer_   = this->create_timer(std::chrono::duration<double>(write_period_),
                    [this]() { this->writeCallback(); }, write_cb_group_);
  publish_timer_ = this->create_timer(std::chrono::duration<double>(pub_period_),
                    [this]() { this->publishCallback(); }, pub_cb_group_);

  LifecycleNode::on_activate(state);
  RCLCPP_INFO(this->get_logger(), "Activated Successfully");
  return LCL_RET::SUCCESS;
}

LCL_RET DynamixelController::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "Deactivating...");
  is_moving_ = false;  // stop trajectory execution
  
  for (auto const& dxl : dynamixel_)
    dxl_wb_->torqueOff((uint8_t)dxl.second);

  dynamixel_state_list_pub_->on_deactivate();
  if (is_joint_state_topic_) joint_states_pub_->on_deactivate();

  read_timer_->cancel();
  write_timer_->cancel();
  publish_timer_->cancel();

  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(this->get_logger(), "Deactivated Successfully");
  return LCL_RET::SUCCESS;
}

LCL_RET DynamixelController::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "Cleaning up...");
  
  dynamixel_.clear();
  dynamixel_info_.clear();
  control_items_.clear();
  pre_goal_.clear();

  read_timer_.reset();
  write_timer_.reset();
  publish_timer_.reset();

  dxl_wb_.reset();
  jnt_tra_.reset();
  
  RCLCPP_INFO(this->get_logger(), "Cleaned up Successfully");
  return LCL_RET::SUCCESS;
}

LCL_RET DynamixelController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "Shutting Down...");

  // for (auto const& dxl : dynamixel_)
  //   dxl_wb_->torqueOff((uint8_t)dxl.second);

    if (read_timer_)    read_timer_->cancel();
  if (write_timer_)   write_timer_->cancel();
  if (publish_timer_) publish_timer_->cancel();

  std::lock_guard<std::mutex> lock(dxl_mutex_);
  if (dxl_wb_)
    {
      for (auto const& dxl : dynamixel_)
        dxl_wb_->torqueOff((uint8_t)dxl.second);
      dxl_wb_.reset();  // safe — no callback can be inside mutex simultaneously
    }
  
  RCLCPP_INFO(this->get_logger(), "Node shutdown Successful. CTRL+C ");
  
  return LCL_RET::SUCCESS;
}

// ---------------------------------------------------------------------------
// Init helpers
// ---------------------------------------------------------------------------

bool DynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  RCLCPP_DEBUG(this->get_logger(), "*** initWorkbench ***");

  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
    RCLCPP_ERROR(this->get_logger(), "%s", log);

  return result;
}

bool DynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  RCLCPP_DEBUG(this->get_logger(), "*** getDynamixelsInfo ***");

  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (!dynamixel)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0) continue;

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t     value     = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      dynamixel_info_.push_back({name, item_value});
    }
  }

  return true;
}

bool DynamixelController::loadDynamixels(void)
{
  RCLCPP_DEBUG(this->get_logger(), "*** loadDynamixels ***");

  bool result = false;
  const char* log;

  for (auto const& dxl : dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", log);
      RCLCPP_ERROR(this->get_logger(), "Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Name : %s, ID : %d, Model Number : %d",
                  dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool DynamixelController::initDynamixels(void)
{
  RCLCPP_DEBUG(this->get_logger(), "*** initDynamixels ***");

  const char* log;

  for (auto const& dxl : dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info : dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second,
                                            info.second.item_name.c_str(),
                                            info.second.value, &log);
          RCLCPP_DEBUG(this->get_logger(), "itemWrite -> ID: %d | Item: %s | Value: %d",
                       (uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value);

          if (result == false)
          {
            RCLCPP_ERROR(this->get_logger(), "%s", log);
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]",
                         info.second.value, info.second.item_name.c_str(),
                         dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    // dxl_wb_->torqueOn((uint8_t)dxl.second);

    // int32_t torque_val = 0;
    // const char* tlog;
    // dxl_wb_->readRegister((uint8_t)dxl.second, "Torque_Enable", &torque_val, &tlog);
    // RCLCPP_DEBUG(this->get_logger(), "ID %d Torque_Enable = %d", dxl.second, torque_val);
  }

  return true;
}

bool DynamixelController::initControlItems(void)
{
  RCLCPP_DEBUG(this->get_logger(), "*** initControlItems ***");

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL) goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL) return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL) present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL) present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"]     = goal_position;
  control_items_["Goal_Velocity"]     = goal_velocity;
  control_items_["Present_Position"]  = present_position;
  control_items_["Present_Velocity"]  = present_velocity;
  control_items_["Present_Current"]   = present_current;

  return true;
}

bool DynamixelController::initSDKHandlers(void)
{
  RCLCPP_DEBUG(this->get_logger(), "*** initSDKHandlers ***");

  bool result = false;
  const char* log = NULL;

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address,
                                         control_items_["Goal_Position"]->data_length, &log);
  if (result == false) { RCLCPP_ERROR(this->get_logger(), "%s", log); return result; }
  else                   RCLCPP_INFO(this->get_logger(), "%s", log);

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address,
                                         control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false) { RCLCPP_ERROR(this->get_logger(), "%s", log); return result; }
  else                   RCLCPP_INFO(this->get_logger(), "%s", log);

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    uint16_t start_address = std::min(control_items_["Present_Position"]->address,
                                       control_items_["Present_Current"]->address);
    uint16_t read_length   = control_items_["Present_Position"]->data_length
                           + control_items_["Present_Velocity"]->data_length
                           + control_items_["Present_Current"]->data_length + 2;

    result = dxl_wb_->addSyncReadHandler(start_address, read_length, &log);
    if (result == false) { RCLCPP_ERROR(this->get_logger(), "%s", log); return result; }
  }

  return result;
}

bool DynamixelController::getPresentPosition(std::vector<std::string> dxl_name)
{
  RCLCPP_DEBUG(this->get_logger(), "*** getPresentPosition called for %ld joints ***", dxl_name.size());

  bool result = false;
  const char* log = NULL;

  std::vector<int32_t> get_position(dxl_name.size());
  std::vector<uint8_t> id_array(dxl_name.size());
  uint8_t id_cnt = 0;

  for (auto const& name : dxl_name)
    id_array[id_cnt++] = dynamixel_[name];

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                id_array.data(), dxl_name.size(), &log);
    if (result == false)
      RCLCPP_ERROR(this->get_logger(), "syncRead failed: %s", log);

    WayPoint wp;
    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                       id_array.data(), id_cnt,
                                       control_items_["Present_Position"]->address,
                                       control_items_["Present_Position"]->data_length,
                                       get_position.data(), &log);
    if (result == false)
    {
      RCLCPP_ERROR(this->get_logger(), "getSyncReadData failed: %s", log);
    }
    else
    {
      for (uint8_t index = 0; index < id_cnt; index++)
      {
        wp.position     = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
        wp.velocity     = 0.0f;
        wp.acceleration = 0.0f;
        pre_goal_.push_back(wp);
        RCLCPP_INFO(this->get_logger(), "  ID %d raw=%d -> pre_goal=%.4f rad",
                    id_array[index], get_position[index], wp.position);
      }
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    WayPoint wp;
    uint32_t read_position;
    for (auto const& dxl : dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                      control_items_["Present_Position"]->address,
                                      control_items_["Present_Position"]->data_length,
                                      &read_position, &log);
      if (result == false)
        RCLCPP_ERROR(this->get_logger(), "readRegister failed: %s", log);

      wp.position     = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
      wp.velocity     = 0.0f;
      wp.acceleration = 0.0f;
      pre_goal_.push_back(wp);
      RCLCPP_INFO(this->get_logger(), "  ID %d raw=%d -> pre_goal=%.4f rad",
                  dxl.second, read_position, wp.position);
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Publisher / Subscriber / Server init
// ---------------------------------------------------------------------------

void DynamixelController::initPublisher()
{
  RCLCPP_DEBUG(this->get_logger(), "*** initPublisher ***");

  dynamixel_state_list_pub_ =
    this->create_publisher<dynamixel_workbench_msgs::msg::DynamixelStateList>("dynamixel_state", 10);

  if (is_joint_state_topic_)
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

void DynamixelController::initSubscriber()
{
  RCLCPP_DEBUG(this->get_logger(), "*** initSubscriber ***");

  trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory", 10,
    std::bind(&DynamixelController::trajectoryMsgCallback, this, std::placeholders::_1));

  if (is_cmd_vel_topic_)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&DynamixelController::commandVelocityCallback, this, std::placeholders::_1));
}

void DynamixelController::initServer()
{
  RCLCPP_DEBUG(this->get_logger(), "*** initServer ***");

  dynamixel_command_server_ = this->create_service<dynamixel_workbench_msgs::srv::DynamixelCommand>(
    "dynamixel_command",
    std::bind(&DynamixelController::dynamixelCommandMsgCallback, this,
              std::placeholders::_1, std::placeholders::_2));
}

// ---------------------------------------------------------------------------
// Timer callbacks
// ---------------------------------------------------------------------------

void DynamixelController::readCallback()
{
  // auto t0 = this->get_clock()->now();
  std::lock_guard<std::mutex> lock(dxl_mutex_);
  if (!dxl_wb_) return; 
  
  bool result = false;
  const char* log = NULL;

  std::vector<dynamixel_workbench_msgs::msg::DynamixelState> dynamixel_state(dynamixel_.size());
  dynamixel_state_list_.dynamixel_state.clear();

  std::vector<int32_t> get_current(dynamixel_.size());
  std::vector<int32_t> get_velocity(dynamixel_.size());
  std::vector<int32_t> get_position(dynamixel_.size());
  std::vector<uint8_t> id_array(dynamixel_.size());
  uint8_t id_cnt = 0;

  for (auto const& dxl : dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id  = (uint8_t)dxl.second;
    id_array[id_cnt++]           = (uint8_t)dxl.second;
  }

  if (true)
  {
    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
      result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                  id_array.data(), dynamixel_.size(), &log);
      if (result == false) RCLCPP_ERROR(this->get_logger(), "%s", log);

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                         id_array.data(), id_cnt,
                                         control_items_["Present_Current"]->address,
                                         control_items_["Present_Current"]->data_length,
                                         get_current.data(), &log);
      if (result == false) RCLCPP_ERROR(this->get_logger(), "%s", log);

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                         id_array.data(), id_cnt,
                                         control_items_["Present_Velocity"]->address,
                                         control_items_["Present_Velocity"]->data_length,
                                         get_velocity.data(), &log);
      if (result == false) RCLCPP_ERROR(this->get_logger(), "%s", log);

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                         id_array.data(), id_cnt,
                                         control_items_["Present_Position"]->address,
                                         control_items_["Present_Position"]->data_length,
                                         get_position.data(), &log);
      if (result == false) RCLCPP_ERROR(this->get_logger(), "%s", log);

      for (uint8_t index = 0; index < id_cnt; index++)
      {
        dynamixel_state[index].present_current  = get_current[index];
        dynamixel_state[index].present_velocity = get_velocity[index];
        dynamixel_state[index].present_position = get_position[index];
        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
      }
    }
    else if (dxl_wb_->getProtocolVersion() == 1.0f)
    {
      uint16_t length_of_data = control_items_["Present_Position"]->data_length
                              + control_items_["Present_Velocity"]->data_length
                              + control_items_["Present_Current"]->data_length;
      std::vector<uint32_t> get_all_data(length_of_data);
      uint8_t dxl_cnt = 0;

      for (auto const& dxl : dynamixel_)
      {
        result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                        control_items_["Present_Position"]->address,
                                        length_of_data, get_all_data.data(), &log);
        if (result == false) RCLCPP_ERROR(this->get_logger(), "%s", log);

        dynamixel_state[dxl_cnt].present_current  = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
        dynamixel_state[dxl_cnt].present_velocity  = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
        dynamixel_state[dxl_cnt].present_position  = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);
        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
        dxl_cnt++;
      }
    }
  }
  // auto dt = this->get_clock()->now() - t0;
  // if (dt.seconds() > read_period_)
  //   RCLCPP_WARN(this->get_logger(), "readCallback took %.4f s", dt.seconds());
}

void DynamixelController::publishCallback()
{
  std::lock_guard<std::mutex> lock(dxl_mutex_);
  if (!dxl_wb_) return;

  if (dynamixel_state_list_.dynamixel_state.size() != dynamixel_.size())
    return;

  dynamixel_state_list_pub_->publish(dynamixel_state_list_);

  if (is_joint_state_topic_)
  {
    joint_state_msg_.header.stamp = this->get_clock()->now();
    joint_state_msg_.name.clear();
    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();

    uint8_t id_cnt = 0;
    for (auto const& dxl : dynamixel_)
    {
      double effort = 0.0;

      joint_state_msg_.name.push_back(dxl.first);

      if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        if (strcmp(dxl_wb_->getModelName((uint8_t)dxl.second), "XL-320") == 0)
          effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
        else
          effort = dxl_wb_->convertValue2Current((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
      }
      else if (dxl_wb_->getProtocolVersion() == 1.0f)
      {
        effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
      }

      double velocity = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second,
                          (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity);
      double position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second,
                          (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_position);

      joint_state_msg_.effort.push_back(effort);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);

      id_cnt++;
    }

    joint_states_pub_->publish(joint_state_msg_);
  }
}

void DynamixelController::writeCallback()
{
  std::lock_guard<std::mutex> lock(dxl_mutex_);
  if (!dxl_wb_) return; 

  bool result = false;
  const char* log = NULL;

  std::vector<uint8_t>  id_array(dynamixel_.size());
  std::vector<int32_t>  dynamixel_position(dynamixel_.size());
  uint8_t id_cnt = 0;

  static uint32_t point_cnt    = 0;
  static uint32_t position_cnt = 0;

  for (auto const& joint : jnt_tra_msg_.joint_names)
    id_array[id_cnt++] = (uint8_t)dynamixel_[joint];

  if (is_moving_ == true)
  {
    for (uint8_t index = 0; index < id_cnt; index++)
    {
      dynamixel_position[index] = dxl_wb_->convertRadian2Value(
        id_array[index], jnt_tra_msg_.points[point_cnt].positions.at(index));

      RCLCPP_DEBUG(this->get_logger(), "writeCallback: ID=%d raw_value=%d from_radian=%.4f",
                   id_array[index], dynamixel_position[index],
                   jnt_tra_msg_.points[point_cnt].positions.at(index));
    }

    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
                                 id_array.data(), id_cnt,
                                 dynamixel_position.data(), 1, &log);

    RCLCPP_DEBUG(this->get_logger(), "syncWrite result=%d point=%d/%ld",
                 result, point_cnt, jnt_tra_msg_.points.size());

    if (result == false)
      RCLCPP_ERROR(this->get_logger(), "%s", log);

    position_cnt++;
    if (position_cnt >= jnt_tra_msg_.points[point_cnt].positions.size())
    {
      point_cnt++;
      position_cnt = 0;
      if (point_cnt >= jnt_tra_msg_.points.size())
      {
        is_moving_    = false;
        point_cnt     = 0;
        position_cnt  = 0;
        RCLCPP_INFO(this->get_logger(), "Complete Execution");
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Topic / service callbacks
// ---------------------------------------------------------------------------

void DynamixelController::commandVelocityCallback(
  std::shared_ptr<const geometry_msgs::msg::Twist> msg)
{
  bool result = false;
  const char* log = NULL;

  const uint8_t LEFT  = 0;
  const uint8_t RIGHT = 1;

  std::vector<double>   wheel_velocity(dynamixel_.size());
  std::vector<int32_t>  dynamixel_velocity(dynamixel_.size());
  std::vector<uint8_t>  id_array(dynamixel_.size());
  uint8_t id_cnt = 0;

  double robot_lin_vel = msg->linear.x;
  double robot_ang_vel = msg->angular.z;

  float rpm = 0.0f;
  for (auto const& dxl : dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  double velocity_constant_value = 1.0 / (wheel_radius_ * rpm * 0.10472);

  wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * wheel_separation_ / 2);
  wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * wheel_separation_ / 2);

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    if (strcmp(dxl_wb_->getModelName(id_array[0]), "XL-320") == 0)
    {
      dynamixel_velocity[LEFT]  = (wheel_velocity[LEFT]  == 0.0f) ? 0 :
        (wheel_velocity[LEFT]  < 0.0f) ? (int32_t)((-1.0f * wheel_velocity[LEFT])  * velocity_constant_value + 1023) :
                                          (int32_t)(wheel_velocity[LEFT]  * velocity_constant_value);
      dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] == 0.0f) ? 0 :
        (wheel_velocity[RIGHT] < 0.0f) ? (int32_t)((-1.0f * wheel_velocity[RIGHT]) * velocity_constant_value + 1023) :
                                          (int32_t)(wheel_velocity[RIGHT] * velocity_constant_value);
    }
    else
    {
      dynamixel_velocity[LEFT]  = (int32_t)(wheel_velocity[LEFT]  * velocity_constant_value);
      dynamixel_velocity[RIGHT] = (int32_t)(wheel_velocity[RIGHT] * velocity_constant_value);
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    dynamixel_velocity[LEFT]  = (wheel_velocity[LEFT]  == 0.0f) ? 0 :
      (wheel_velocity[LEFT]  < 0.0f) ? (int32_t)((-1.0f * wheel_velocity[LEFT])  * velocity_constant_value + 1023) :
                                        (int32_t)(wheel_velocity[LEFT]  * velocity_constant_value);
    dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] == 0.0f) ? 0 :
      (wheel_velocity[RIGHT] < 0.0f) ? (int32_t)((-1.0f * wheel_velocity[RIGHT]) * velocity_constant_value + 1023) :
                                        (int32_t)(wheel_velocity[RIGHT] * velocity_constant_value);
  }

  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
                               id_array.data(), dynamixel_.size(),
                               dynamixel_velocity.data(), 1, &log);
  if (result == false)
    RCLCPP_ERROR(this->get_logger(), "%s", log);
}

void DynamixelController::trajectoryMsgCallback(
  std::shared_ptr<const trajectory_msgs::msg::JointTrajectory> msg)
{
  std::lock_guard<std::mutex> lock(dxl_mutex_);
  uint8_t id_cnt = 0;
  bool result = false;
  WayPoint wp;

  if (is_moving_ == false)
  {
    jnt_tra_msg_.joint_names.clear();
    jnt_tra_msg_.points.clear();
    pre_goal_.clear();

    result = getPresentPosition(msg->joint_names);

    RCLCPP_INFO(this->get_logger(), "target position = %.4f rad", msg->points[0].positions[0]);
    RCLCPP_INFO(this->get_logger(), "move_time = %.4f sec",
                rclcpp::Duration(msg->points[0].time_from_start).seconds());

    if (result == false)
      RCLCPP_ERROR(this->get_logger(), "Failed to get Present Position");

    for (auto const& joint : msg->joint_names)
    {
      RCLCPP_INFO(this->get_logger(), "'%s' is ready to move", joint.c_str());
      jnt_tra_msg_.joint_names.push_back(joint);
      id_cnt++;
    }

    if (id_cnt != 0)
    {
      uint8_t cnt = 0;
      while (cnt < msg->points.size())
      {
        std::vector<WayPoint> goal;
        for (size_t id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
        {
          wp.position     = msg->points[cnt].positions.at(id_num);
          wp.velocity     = (msg->points[cnt].velocities.size()     != 0) ? msg->points[cnt].velocities.at(id_num)     : 0.0f;
          wp.acceleration = (msg->points[cnt].accelerations.size()  != 0) ? msg->points[cnt].accelerations.at(id_num)  : 0.0f;
          goal.push_back(wp);
        }

        if (use_moveit_ == true)
        {
          trajectory_msgs::msg::JointTrajectoryPoint jnt_tra_point_msg;
          for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
          {
            jnt_tra_point_msg.positions.push_back(goal[id_num].position);
            jnt_tra_point_msg.velocities.push_back(goal[id_num].velocity);
            jnt_tra_point_msg.accelerations.push_back(goal[id_num].acceleration);
          }
          jnt_tra_msg_.points.push_back(jnt_tra_point_msg);
          cnt++;
        }
        else
        {
          jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());

          double move_time = (cnt == 0)
            ? rclcpp::Duration(msg->points[cnt].time_from_start).seconds()
            : rclcpp::Duration(msg->points[cnt].time_from_start).seconds()
              - rclcpp::Duration(msg->points[cnt - 1].time_from_start).seconds();

          jnt_tra_->init(move_time, write_period_, pre_goal_, goal);

          trajectory_msgs::msg::JointTrajectoryPoint jnt_tra_point_msg;

          for (double index = 0.0; index < move_time; index += write_period_)
          {
            std::vector<WayPoint> way_point = jnt_tra_->getJointWayPoint(index);

            for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
            {
              jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
              jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
              jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
            }

            jnt_tra_msg_.points.push_back(jnt_tra_point_msg);
            jnt_tra_point_msg.positions.clear();
            jnt_tra_point_msg.velocities.clear();
            jnt_tra_point_msg.accelerations.clear();
          }

          pre_goal_ = goal;
          cnt++;
        }
      }

      RCLCPP_INFO(this->get_logger(), "Succeeded to get joint trajectory!");
      is_moving_ = true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Please check joint_name");
    }
  }
  else
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dynamixel is Moving");
  }
}

bool DynamixelController::dynamixelCommandMsgCallback(
  const std::shared_ptr<dynamixel_workbench_msgs::srv::DynamixelCommand::Request> req,
  std::shared_ptr<dynamixel_workbench_msgs::srv::DynamixelCommand::Response> res)
{
  RCLCPP_DEBUG(this->get_logger(), "*** dynamixelCommandMsgCallback ***");

  bool result = false;
  const char* log;

  result = dxl_wb_->itemWrite((uint8_t)req->id, req->addr_name.c_str(), req->value, &log);
  if (result == false)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", log);
    RCLCPP_ERROR(this->get_logger(), "Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]",
                 req->value, req->addr_name.c_str(), req->id);
  }

  res->comm_result = result;
  return true;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DynamixelController>();

  // rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  
  if (node->get_parameter("autostart").as_bool())
  {
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }
  RCLCPP_DEBUG(node->get_logger(), "Lifecycle node ready. Use 'ros2 lifecycle' to manage state.");
  
  executor.spin(); 

  rclcpp::shutdown();
  return 0;
}