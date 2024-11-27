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

#include "ackermann_arduino/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ackermann_arduino
{
  hardware_interface::CallbackReturn AckermannArduinoHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
    cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];

    cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
    cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];

    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    rear_right_wheel_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);
    rear_left_wheel_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);

    steering_.name_r = cfg_.front_right_wheel_name;
    steering_.name_l = cfg_.front_left_wheel_name;

    RCLCPP_INFO(
        rclcpp::get_logger("AckermannArduinoHardware"), "Joint '%s' is a steering joint.",
        steering_.name_l.c_str());

    // Check if the number of joints is correct based on the mode of operation
    if (info_.joints.size() != 4)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("AckermannArduinoHardware"),
          "AckermannArduinoHardware::on_init() - Failed to initialize, "
          "because the number of joints %ld is not 4.",
          info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      bool joint_is_steering = joint.name.find("front") != std::string::npos;

      // Steering joints have a position command interface and a position state interface
      if (joint_is_steering)
      {
        RCLCPP_INFO(
            rclcpp::get_logger("AckermannArduinoHardware"), "Joint '%s' is a steering joint.",
            joint.name.c_str());

        if (joint.command_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      else
      {
        RCLCPP_INFO(
            rclcpp::get_logger("AckermannArduinoHardware"), "Joint '%s' is a drive joint.",
            joint.name.c_str());

        // Drive joints have a velocity command interface and a velocity state interface
        if (joint.command_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("AckermannArduinoHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> AckermannArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // rear right wheel state interfaces
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rear_right_wheel_.name, hardware_interface::HW_IF_POSITION, &rear_right_wheel_.pos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rear_right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_.vel));

    // rear left wheel state interfaces
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rear_left_wheel_.name, hardware_interface::HW_IF_POSITION, &rear_left_wheel_.pos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rear_left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_.vel));

    // steering joint state interface
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        steering_.name_r, hardware_interface::HW_IF_POSITION, &steering_.pos_r));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        steering_.name_l, hardware_interface::HW_IF_POSITION, &steering_.pos_l));

    RCLCPP_INFO(
        rclcpp::get_logger("AckermannArduinoHardware"), "Exported %zu state interfaces.",
        state_interfaces.size());

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  AckermannArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // steering
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        steering_.name_r, hardware_interface::HW_IF_POSITION,
        &steering_.cmd_r));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        steering_.name_l, hardware_interface::HW_IF_POSITION,
        &steering_.cmd_l));

    // traction
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        rear_right_wheel_.name, hardware_interface::HW_IF_VELOCITY,
        &rear_right_wheel_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        rear_left_wheel_.name, hardware_interface::HW_IF_VELOCITY,
        &rear_left_wheel_.cmd));

    RCLCPP_INFO(
        rclcpp::get_logger("AckermannArduinoHardware"), "Exported %zu command interfaces.",
        command_interfaces.size());

    return command_interfaces;
  }

  hardware_interface::CallbackReturn AckermannArduinoHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Activating ...please wait...");

    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn AckermannArduinoHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Deactivating ...please wait...");

    if (comms_.connected())
    {
      comms_.disconnect();
    }

    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn AckermannArduinoHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Activating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn AckermannArduinoHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("AckermannArduinoHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type AckermannArduinoHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comms_.connected())
    {
      RCLCPP_ERROR(rclcpp::get_logger("AckermannArduinoHardware"), "Arduino connection lost.");
      return hardware_interface::return_type::ERROR;
    }
    // read encoder values and servo angle from arduino
    comms_.read_encoder_values(rear_left_wheel_.enc, rear_right_wheel_.enc);
    comms_.read_steering_value(steering_.pos_raw);

    // map values
    steering_.map_pos_to_control();

    double delta_seconds = period.seconds();
    float pos_prev = rear_left_wheel_.pos;

    rear_left_wheel_.pos = rear_left_wheel_.calc_enc_angle();
    rear_left_wheel_.vel = (rear_left_wheel_.pos - pos_prev) / delta_seconds;

    pos_prev = rear_right_wheel_.pos;

    rear_right_wheel_.pos = rear_right_wheel_.calc_enc_angle();
    rear_right_wheel_.vel = (rear_right_wheel_.pos - pos_prev) / delta_seconds;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ackermann_arduino ::AckermannArduinoHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      RCLCPP_ERROR(rclcpp::get_logger("AckermannArduinoHardware"), "Arduino connection lost.");
      return hardware_interface::return_type::ERROR;
    }
    int rear_left_wheel_counts_per_loop = rear_left_wheel_.cmd / rear_left_wheel_.rads_per_count / cfg_.loop_rate;
    int rear_right_wheel_counts_per_loop = rear_right_wheel_.cmd / rear_right_wheel_.rads_per_count / cfg_.loop_rate;

    steering_.map_cmd_to_arduino();

    comms_.set_motor_values(rear_left_wheel_counts_per_loop, rear_right_wheel_counts_per_loop);
    comms_.set_steering_value(steering_.cmd_raw);

    return hardware_interface::return_type::OK;
  }

} // namespace ackermann_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ackermann_arduino::AckermannArduinoHardware, hardware_interface::SystemInterface)
