// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("velocity_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/velocity_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;

  commands.data.push_back(0);
  commands.data.push_back(0);
  commands.data.push_back(0);
  commands.data.push_back(0);
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);
  
// First lift the AGV from low leverl to high level
  commands.data[0] = 0.2;
  commands.data[1] = -0.6;
  commands.data[2] = 0.0;
  commands.data[3] = 0.0;
  publisher->publish(commands);
  std::this_thread::sleep_for(3s);
  
// Perform tilt to one end
  commands.data[0] = 0.2;
  commands.data[1] = 0.0;
  commands.data[2] = 0.0;
  commands.data[3] = 0.0;
  publisher->publish(commands);
  std::this_thread::sleep_for(3s);
  
// back to the top level
  commands.data[0] = 0.2;
  commands.data[1] = -1.5;
  commands.data[2] = 0.0;
  commands.data[3] = 0.0;
  publisher->publish(commands);
  std::this_thread::sleep_for(3s);
  
// Perform unload-1
  commands.data[0] = 0.2;
  commands.data[1] = -0.6;
  commands.data[2] = 0.1;
  commands.data[3] = 0.0;
  publisher->publish(commands);
  std::this_thread::sleep_for(5s);
  
// Perform unload-2
  commands.data[0] = 0.2;
  commands.data[1] = -0.6;
  commands.data[2] = 0.1;
  commands.data[3] = -0.3;
  publisher->publish(commands);
  std::this_thread::sleep_for(5s);
  
// Perform load-1
  commands.data[0] = 0.2;
  commands.data[1] = -0.6;
  commands.data[2] = -0.1;
  commands.data[3] = -0.3;
  publisher->publish(commands);
  std::this_thread::sleep_for(5s);
  
// Perform load-2
  commands.data[0] = 0.2;
  commands.data[1] = -0.6;
  commands.data[2] = -0.1;
  commands.data[3] = 0.3;
  publisher->publish(commands);
  std::this_thread::sleep_for(5s);
  
// Back to low level
  commands.data[0] = -0.6;
  commands.data[1] = 0.0005;
  commands.data[2] = -0.1;
  commands.data[3] = 0.3;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);
  
// reset to stable position
  commands.data[0] = 0.0;
  commands.data[1] = 0.0;
  commands.data[2] = 0.0;
  commands.data[3] = 0.0;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);
  
  rclcpp::shutdown();

  return 0;
}
