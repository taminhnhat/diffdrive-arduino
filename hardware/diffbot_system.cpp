// Copyright 2021 ros2_control Development Team
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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <jsoncpp/json/json.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
    cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
    cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
    cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
    if (info_.hardware_parameters.count("pid_p") > 0)
    {
      cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
      cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
      cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
      cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
    }

    wheel_rear_l_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_front_l_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_rear_r_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);
    wheel_front_r_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rear_l_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rear_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rear_r_.name, hardware_interface::HW_IF_POSITION, &wheel_rear_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rear_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_r_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_front_l_.name, hardware_interface::HW_IF_POSITION, &wheel_front_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_front_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_front_r_.name, hardware_interface::HW_IF_POSITION, &wheel_front_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_front_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_r_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_rear_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_l_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_front_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_rear_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rear_r_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_front_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_front_r_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    std::string read_str = "";
    if (!comms_.read_hardware_states(read_str, false))
      return hardware_interface::return_type::OK;

    Json::Value root;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(read_str, root);
    if (!parsingSuccessful)
    {
      std::cout << "Error parsing the string" << std::endl;
    }
    // const int battery = root["battery"].asDouble();
    pipe_.writeLine(read_str);
    const auto velocity = root["velocity"];
    const auto position = root["position"];
    // std::cout << "battery: " << battery << std::endl;
    // std::cout << "velocity: " << velocity[0] << "\t" << velocity[1] << "\t" << velocity[2] << "\t" << velocity[3] << "\t" << std::endl;
    // std::cout << "position: " << position[0] << "\t" << position[1] << "\t" << position[2] << "\t" << position[3] << "\t" << std::endl;

    wheel_front_r_.vel = velocity[0].asDouble();
    wheel_rear_r_.vel = velocity[1].asDouble();
    wheel_rear_l_.vel = velocity[2].asDouble();
    wheel_front_l_.vel = velocity[3].asDouble();

    wheel_front_r_.pos = position[0].asDouble();
    wheel_rear_r_.pos = position[1].asDouble();
    wheel_rear_l_.pos = position[2].asDouble();
    wheel_front_l_.pos = position[3].asDouble();

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
    // int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
    // comms_.set_motor_values(motor_r_counts_per_loop, motor_l_counts_per_loop);
    double front_right_vel = wheel_front_r_.cmd;
    double rear_right_vel = wheel_rear_r_.cmd;
    double front_left_vel = wheel_front_l_.cmd;
    double rear_left_vel = wheel_rear_l_.cmd;
    char cmd[100];
    sprintf(cmd, "{\"topic\":\"ros2_control\",\"velocity\":[%.2f,%.2f,%.2f,%.2f]}", front_right_vel, rear_right_vel, rear_left_vel, front_left_vel);
    std::string msg = cmd;
    comms_.write_hardware_command(msg, false);
    return hardware_interface::return_type::OK;
  }

} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
