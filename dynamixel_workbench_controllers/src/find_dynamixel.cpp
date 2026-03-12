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

#include "rclcpp/rclcpp.hpp"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define BAUDRATE_NUM 7

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamixel_scanner");
  std::string port_name = "/dev/ttyUSB0";

  if (argc < 2)
  {
    RCLCPP_WARN(node->get_logger(),"Please set '-port_name'arguments for connected Dynamixels");
    rclcpp::shutdown();
    return 0;
  }
  else
  {
    port_name = argv[1];
  }

  DynamixelWorkbench dxl_wb;

  const char *log;
  bool result = false;

  uint8_t scanned_id[100];
  uint8_t dxl_cnt = 0;

  uint32_t baudrate[BAUDRATE_NUM] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000};
  uint8_t range = 253;

  uint8_t index = 0;

   while (rclcpp::ok() && index < BAUDRATE_NUM)
  {
    result = dxl_wb.init(port_name.c_str(), baudrate[index], &log);
    if (result == false)
    {
      RCLCPP_WARN(node->get_logger(),"%s",log);
      RCLCPP_WARN(node->get_logger(),"Failed to init(%d)", baudrate[index]);
    }
    else
    {
      RCLCPP_INFO(node->get_logger(),"Succeed to init(%d)",baudrate[index]);
    

      dxl_cnt = 0;
      for (uint8_t num = 0; num < 100; num++) scanned_id[num] = 0;

      RCLCPP_INFO(node->get_logger(),"Wait for scanning...");
      result = dxl_wb.scan(scanned_id, &dxl_cnt, range, &log);
      if (result == false)
      {
        RCLCPP_WARN(node->get_logger(),"%s", log);
        RCLCPP_WARN(node->get_logger(),"Failed to scan");
      }
      else
      {
        RCLCPP_INFO(node->get_logger(),"Find %d Dynamixels", dxl_cnt);

        for (int cnt = 0; cnt < dxl_cnt; cnt++)
          RCLCPP_INFO(node->get_logger(),"id : %d, model name : %s", scanned_id[cnt], dxl_wb.getModelName(scanned_id[cnt]));
      }
    }
    
    dxl_wb.~DynamixelWorkbench();
    index++;
  }
  RCLCPP_INFO(node->get_logger(), "Scanner shutting down.");
  rclcpp::shutdown();
  return 0;
}
