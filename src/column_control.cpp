#include "robo_drill/column_control.hpp"
#include <chrono>

using namespace std::chrono_literals;

ColumnNode::ColumnNode() 
    : Node("column"),
      modbus_ctx_(nullptr),
      heartbeat_(10),
      last_position_(0),
      has_last_position_(false)
{
    // Create publishers
    pos_pub_ = this->create_publisher<std_msgs::msg::Int32>("column_position", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::Int32>("column_status", 10);
    moving_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_moving", 10);

    // Create services
    set_position_service_ = this->create_service<robo_drill::srv::SetPosition>(
        "set_column_position",
        std::bind(&ColumnNode::set_position_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_column",
        std::bind(&ColumnNode::stop_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Initialize Modbus connection
    modbus_ctx_ = modbus_new_tcp(LC3_IP, LC3_PORT);
    if (!modbus_ctx_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create modbus context");
        return;
    }

    modbus_set_slave(modbus_ctx_, SLAVE_ID);

    if (modbus_connect(modbus_ctx_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to column: %s", 
                     modbus_strerror(errno));
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to column");

    // Clear power-on block
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, 64256) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write initial command: %s", 
                     modbus_strerror(errno));
    }
    
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, CMD_STOP) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send stop command: %s", 
                     modbus_strerror(errno));
    } else {
        RCLCPP_INFO(this->get_logger(), "Initial Stop command sent to clear power-on block");
    }

    // Create timers
    poll_timer_ = this->create_wall_timer(
        1000ms, std::bind(&ColumnNode::poll, this));
    
    aux_feedback_timer_ = this->create_wall_timer(
        500ms, std::bind(&ColumnNode::aux_feedback, this));
}

ColumnNode::~ColumnNode() {
    if (modbus_ctx_) {
        modbus_close(modbus_ctx_);
        modbus_free(modbus_ctx_);
    }
}

bool ColumnNode::general_run_prerequisites() {
    if (!modbus_ctx_) {
        RCLCPP_ERROR(this->get_logger(), "Modbus context is null");
        return false;
    }

    // Send stop commands
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, 64256) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write command: %s", 
                     modbus_strerror(errno));
        return false;
    }
    
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, CMD_STOP) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send stop: %s", 
                     modbus_strerror(errno));
        return false;
    }

    // Read status flag and error code
    uint16_t status_flag, error_code;
    
    if (modbus_read_registers(modbus_ctx_, REG_STATUS_FLAG, 1, &status_flag) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read status flag: %s", 
                     modbus_strerror(errno));
        return false;
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_ERROR_CODE, 1, &error_code) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read error code: %s", 
                     modbus_strerror(errno));
        return false;
    }

    bool ok = true;
    
    if (error_code != 0) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Prerequisite failed - error code: %d", error_code);
        ok = false;
    }
    
    if ((status_flag & 0x04) == 0x04) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Prerequisite failed - overcurrent (bit 2 of status flag)");
        ok = false;
    }
    
    if ((status_flag & 0x20) == 0x20) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Prerequisite failed - heartbeat needed (bit 5 of status flag)");
        ok = false;
    }
    
    return ok;
}

void ColumnNode::poll() {
    if (!modbus_ctx_) return;

    // Send heartbeat
    if (modbus_write_register(modbus_ctx_, REG_STATUS, heartbeat_) == -1) {
        RCLCPP_WARN(this->get_logger(), "Failed to send heartbeat: %s", 
                    modbus_strerror(errno));
        return;
    }
    
    heartbeat_++;
    if (heartbeat_ > 255) {
        heartbeat_ = 0;
    }

    // Read position and status
    uint16_t position, status;
    
    if (modbus_read_registers(modbus_ctx_, REG_FB_POSITION, 1, &position) == -1) {
        RCLCPP_WARN(this->get_logger(), "Failed to read position: %s", 
                    modbus_strerror(errno));
        return;
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_STATUS_FLAG, 1, &status) == -1) {
        RCLCPP_WARN(this->get_logger(), "Failed to read status: %s", 
                    modbus_strerror(errno));
        return;
    }

    // Publish position and status
    auto pos_msg = std_msgs::msg::Int32();
    pos_msg.data = static_cast<int32_t>(position);
    pos_pub_->publish(pos_msg);

    auto status_msg = std_msgs::msg::Int32();
    status_msg.data = static_cast<int32_t>(status);
    status_pub_->publish(status_msg);

    // Detect motion
    bool moving = false;
    if (has_last_position_) {
        moving = std::abs(static_cast<int32_t>(position) - last_position_) > 2;
    }
    
    auto moving_msg = std_msgs::msg::Bool();
    moving_msg.data = moving;
    moving_pub_->publish(moving_msg);

    last_position_ = static_cast<int32_t>(position);
    has_last_position_ = true;
}

void ColumnNode::aux_feedback() {
    if (!modbus_ctx_) return;

    uint16_t position, current, speed, error_code, status_flag;
    
    if (modbus_read_registers(modbus_ctx_, REG_FB_POSITION, 1, &position) != -1) {
        RCLCPP_INFO(this->get_logger(), "Position feedback: %d", position);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_FB_CURRENT, 1, &current) != -1) {
        RCLCPP_INFO(this->get_logger(), "Current feedback: %d", current);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_FB_SPEED, 1, &speed) != -1) {
        RCLCPP_INFO(this->get_logger(), "Speed feedback: %d", speed);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_ERROR_CODE, 1, &error_code) != -1) {
        RCLCPP_INFO(this->get_logger(), "Error code feedback: %d", error_code);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_STATUS_FLAG, 1, &status_flag) != -1) {
        RCLCPP_INFO(this->get_logger(), "Status flag feedback: %d", status_flag);
    }
}

void ColumnNode::aux_command() {
    if (!modbus_ctx_) return;

    uint16_t position, current, speed, soft_start, soft_stop;
    
    if (modbus_read_registers(modbus_ctx_, REG_COMMAND_POSITION, 1, &position) != -1) {
        RCLCPP_INFO(this->get_logger(), "Position command: %d", position);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_COMMAND_CURRENT, 1, &current) != -1) {
        RCLCPP_INFO(this->get_logger(), "Current command: %d", current);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_COMMAND_SPEED, 1, &speed) != -1) {
        RCLCPP_INFO(this->get_logger(), "Speed command: %d", speed);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_COMMAND_SOFT_START, 1, &soft_start) != -1) {
        RCLCPP_INFO(this->get_logger(), "Soft start command: %d", soft_start);
    }
    
    if (modbus_read_registers(modbus_ctx_, REG_COMMAND_SOFT_STOP, 1, &soft_stop) != -1) {
        RCLCPP_INFO(this->get_logger(), "Soft stop command: %d", soft_stop);
    }
}

void ColumnNode::set_position_callback(
    const std::shared_ptr<robo_drill::srv::SetPosition::Request> request,
    std::shared_ptr<robo_drill::srv::SetPosition::Response> response)
{
    if (!general_run_prerequisites()) {
        response->message = "Prerequisites not met";
        return;
    }

    uint16_t target = static_cast<uint16_t>(request->target_pos);
    RCLCPP_WARN(this->get_logger(), "Target position: %d", target);

    // Set default parameters
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_CURRENT, 251) == -1 ||
        modbus_write_register(modbus_ctx_, REG_COMMAND_SPEED, 251) == -1 ||
        modbus_write_register(modbus_ctx_, REG_COMMAND_SOFT_START, 251) == -1 ||
        modbus_write_register(modbus_ctx_, REG_COMMAND_SOFT_STOP, 251) == -1) {
        response->message = std::string("Failed to set parameters: ") + modbus_strerror(errno);
        return;
    }

    // Write target position
    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, target) == -1) {
        response->message = std::string("Failed to write position: ") + modbus_strerror(errno);
        return;
    }

    response->message = "Target " + std::to_string(target) + " written";
}

void ColumnNode::stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused parameter

    if (modbus_write_register(modbus_ctx_, REG_COMMAND_POSITION, CMD_STOP) == -1) {
        response->success = false;
        response->message = std::string("Failed to send stop: ") + modbus_strerror(errno);
        return;
    }

    response->success = true;
    response->message = "Stop command sent";
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColumnNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}