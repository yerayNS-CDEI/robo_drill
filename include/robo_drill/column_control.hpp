#ifndef COLUMN_NODE_HPP
#define COLUMN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "robo_drill/srv/set_position.hpp"
#include <modbus/modbus.h>
#include <memory>

class ColumnNode : public rclcpp::Node {
public:
    ColumnNode();
    ~ColumnNode();

private:
    // Modbus registers
    static constexpr uint16_t REG_STATUS = 0x2001;
    static constexpr uint16_t REG_COMMAND_POSITION = 0x2002;
    static constexpr uint16_t REG_COMMAND_CURRENT = 0x2003;
    static constexpr uint16_t REG_COMMAND_SPEED = 0x2004;
    static constexpr uint16_t REG_COMMAND_SOFT_START = 0x2005;
    static constexpr uint16_t REG_COMMAND_SOFT_STOP = 0x2006;
    static constexpr uint16_t REG_FB_POSITION = 0x2101;
    static constexpr uint16_t REG_FB_CURRENT = 0x2102;
    static constexpr uint16_t REG_STATUS_FLAG = 0x2103;
    static constexpr uint16_t REG_ERROR_CODE = 0x2104;
    static constexpr uint16_t REG_FB_SPEED = 0x2105;

    // Commands
    static constexpr uint16_t CMD_STOP = 0xFB03;
    static constexpr uint16_t CMD_RUN_OUT = 0xFB01;

    // Connection parameters
    static constexpr const char* LC3_IP = "192.168.1.10";
    static constexpr int LC3_PORT = 502;
    static constexpr int SLAVE_ID = 1;

    // Modbus client
    modbus_t* modbus_ctx_;

    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr moving_pub_;

    // ROS2 services
    rclcpp::Service<robo_drill::srv::SetPosition>::SharedPtr set_position_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

    // Timers
    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::TimerBase::SharedPtr aux_feedback_timer_;

    // State variables
    uint16_t heartbeat_;
    int32_t last_position_;
    bool has_last_position_;

    // Methods
    void poll();
    void aux_feedback();
    void aux_command();
    bool general_run_prerequisites();
    
    void set_position_callback(
        const std::shared_ptr<robo_drill::srv::SetPosition::Request> request,
        std::shared_ptr<robo_drill::srv::SetPosition::Response> response);
    
    void stop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif // COLUMN_NODE_HPP