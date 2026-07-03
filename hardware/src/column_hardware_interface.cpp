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

#include "robodrill_hw_interface/column_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace column_hw_interface
{


  hardware_interface::CallbackReturn ColumnHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse hardware parameters
    cfg_.column_name = info_.hardware_parameters["column_name"];

    column_.setup(cfg_.column_name);
  
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ColumnHardwareInterface"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ColumnHardwareInterface"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ColumnHardwareInterface"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ColumnHardwareInterface"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ColumnHardwareInterface"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  

  std::vector<hardware_interface::StateInterface> ColumnHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        column_.name, hardware_interface::HW_IF_POSITION, &column_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        column_.name, hardware_interface::HW_IF_VELOCITY, &column_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ColumnHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        column_.name, hardware_interface::HW_IF_POSITION, &column_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ColumnHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Configuring ...please wait...");
        modbus_ctx_ = modbus_new_tcp(LC3_IP, LC3_PORT);
    if (!modbus_ctx_) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to create modbus context");
        return hardware_interface::CallbackReturn::ERROR;
    }
    modbus_set_slave(modbus_ctx_, SLAVE_ID);

    if (modbus_connect(modbus_ctx_) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to connect to column: %s", 
                     modbus_strerror(errno));
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Connected to column");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ColumnHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)

  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ColumnHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Activating Column hardware interface...");

    // Initialize state variables
    heartbeat_ = 0;
    last_position_ = 0;

    // Clear power-on block
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, 64256) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to write initial command: %s", 
                     modbus_strerror(errno));
    }
    
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, CMD_STOP) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to send stop command: %s", 
                     modbus_strerror(errno));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Initial Stop command sent to clear power-on block");
    }
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), GREEN "Clear power-on block" RESET);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ColumnHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Deactivating hardware interface...");
    
    if (!modbus_ctx_) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Modbus context is null");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Send position 0 command to retract the column
    if (send_target_position(0) != hardware_interface::return_type::OK) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to send retract command");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Retracting column to position 0...");
    
    // Wait for column to retract until it stops moving
    // Keep sending heartbeat while waiting
    uint16_t position_fb;
    uint16_t last_position = 65535;  // Initialize to impossible value
    uint16_t local_heartbeat = heartbeat_;
    const int max_wait_seconds = 15;
    int elapsed_seconds = 0;
    int stall_count = 0;  // Count how many times position hasn't changed
    
    while (true) {
        // Send heartbeat (column needs this to keep moving)
        if (modbus_write_register(modbus_ctx_, REG_STATUS, local_heartbeat) == -1) {
            RCLCPP_WARN(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to send heartbeat during retraction");
        }
        local_heartbeat++;
        if (local_heartbeat > 255) {
            local_heartbeat = 0;
        }
        
        modbus_read_registers(modbus_ctx_, REG_FB_POSITION, 1, &position_fb);
        
        // Check if position is no longer changing (stalled)
        if (position_fb == last_position) {
            stall_count++;
            if (stall_count >= 5) {  // Position hasn't changed for 5 seconds (increased from 3)
                RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), 
                           "Column fully retracted at position: %d (%.4f m)", position_fb, position_fb/10000.0);
                break;
            }
        } else {
            stall_count = 0;  // Reset stall counter if position changed
        }
        last_position = position_fb;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        elapsed_seconds++;
        
        if (elapsed_seconds >= max_wait_seconds) {
            RCLCPP_WARN(rclcpp::get_logger("ColumnHardwareInterface"), 
                        "Timeout waiting for column retraction. Final position: %d (%.4f m)", 
                        position_fb, position_fb/10000.0);
            break;
        }
    }
    
    // Don't send stop command - let the column finish its motion naturally
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), 
                "Column retraction complete at position: %d (%.4f m)", position_fb, position_fb/10000.0);
    
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), GREEN "Column hardware interface deactivated." RESET);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ColumnHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Shutting down hardware interface...");
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, CMD_STOP) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to send stop command: %s", 
                     modbus_strerror(errno));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Stop command sent to column");
    }
    // Cleanup Modbus connection
    if (modbus_ctx_) {
        RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Closing Modbus connection...");
        modbus_close(modbus_ctx_);
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Modbus connection closed.");
    }

    RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), GREEN "Hardware interface shutdown complete." RESET);

    return hardware_interface::CallbackReturn::SUCCESS;
  }



  hardware_interface::return_type ColumnHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!modbus_ctx_) return hardware_interface::return_type::ERROR;

    // Reading feedback registers
    uint16_t position_fb;
    uint16_t speed_fb;// current_fb, error_code, status_flag;
  
    modbus_read_registers(modbus_ctx_, REG_FB_POSITION, 1, &position_fb);
    column_.pos = static_cast<float>(position_fb) / 10000.0f; // Convert back to meters 
    
    modbus_read_registers(modbus_ctx_, REG_FB_SPEED, 1, &speed_fb);
    column_.vel = static_cast<float>(speed_fb) / 0.005 ; //! Convert back to m/s not sure about the unit

    // Removed periodic logging - use 'ros2 topic echo' to monitor position if needed

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ColumnHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Send heartbeat
    if (modbus_write_register(modbus_ctx_, REG_STATUS, heartbeat_) == -1) {
        RCLCPP_WARN(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to send heartbeat: %s", 
                    modbus_strerror(errno));
        return hardware_interface::return_type::ERROR;
    }
    
    heartbeat_++;
    if (heartbeat_ > 255) {
        heartbeat_ = 0;
    }

    // Convert position to register value
    int32_t target_position = static_cast<int32_t>(column_.cmd * 10000);

    // Only send position command if it has changed
    if ( target_position != last_position_) {
        RCLCPP_INFO(rclcpp::get_logger("ColumnHardwareInterface"), "Position command: %.4f m", column_.cmd);

        // Send the position command
        if (send_target_position(target_position) != hardware_interface::return_type::OK) {
            return hardware_interface::return_type::ERROR;
        }

        // Update cached position
        last_position_ = target_position;
    }

    return hardware_interface::return_type::OK;
  }


  bool ColumnHardwareInterface::general_run_prerequisites() {
    if (!modbus_ctx_) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Modbus context is null");
        return false;
    }

    // Send stop commands
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, 64256) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to write command: %s", 
                     modbus_strerror(errno));
        return false;
    }
    
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, CMD_STOP) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to send stop: %s", 
                     modbus_strerror(errno));
        return false;
    }

    // Read status flag and error code
    uint16_t status_flag, error_code;
    
    if (modbus_read_registers(modbus_ctx_, REG_STATUS_FLAG, 1, &status_flag) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to read status flag: %s", 
                     modbus_strerror(errno));
        return false;
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_ERROR_CODE, 1, &error_code) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to read error code: %s", 
                     modbus_strerror(errno));
        return false;
    }

    bool ok = true;
    
    if (error_code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), 
                     "Prerequisite failed - error code: %d", error_code);
        ok = false;
    }
    
    if ((status_flag & 0x04) == 0x04) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), 
                     "Prerequisite failed - overcurrent (bit 2 of status flag)");
        ok = false;
    }
    
    if ((status_flag & 0x20) == 0x20) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), 
                     "Prerequisite failed - heartbeat needed (bit 5 of status flag)");
        ok = false;
    }
    
    return ok;
  }

  hardware_interface::return_type ColumnHardwareInterface::send_target_position(int32_t target_position) {
    // step 1: Check prerequisites
    if (!general_run_prerequisites()) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "General run prerequisites failed. Aborting write.");
        return hardware_interface::return_type::ERROR;
    }
    
    // step 2-5: Set motor parameters
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_CURRENT, 251) == -1 ||
        modbus_write_register(modbus_ctx_, REG_COMMAND_SPEED, 251) == -1 ||
        modbus_write_register(modbus_ctx_, REG_COMMAND_SOFT_START, 251) == -1 ||
        modbus_write_register(modbus_ctx_, REG_COMMAND_SOFT_STOP, 251) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to set parameters: %s", modbus_strerror(errno));
        return hardware_interface::return_type::ERROR;
    }

    // step 6: Write target position
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, target_position) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("ColumnHardwareInterface"), "Failed to write position: %s", modbus_strerror(errno));
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

} // namespace column_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    column_hw_interface::ColumnHardwareInterface, hardware_interface::SystemInterface)
