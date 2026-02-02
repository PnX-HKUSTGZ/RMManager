#include "rm_custom_controller_state/rm_custom_controller_state.hpp"

namespace rm_custom_controller_state
{


int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_uint, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (float)((float)x_uint * span / ((float)((1 << bits) - 1)) + offset);
}


RmCustomControllerState::RmCustomControllerState(const rclcpp::NodeOptions & options)
: Node("rm_custom_controller_state", options)
{

    // 获取参数
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // 填充参数
    left_arm_state_topic = params_.left_arm_state_topic;
    right_arm_state_topic = params_.right_arm_state_topic;
    left_joint_names = params_.left_joint_names;
    right_joint_names = params_.right_joint_names;
    ref_topic = params_.ref_topic;
    watchdog_timeout_ = params_.watchdog_timeout;

    // 检查 left_joint_names 与 right_joint_names 长度是否为6
    if (left_joint_names.size() != JOINT_NUM) {
        RCLCPP_ERROR(this->get_logger(), "Left joint names size is not 6!");
        throw std::runtime_error("Left joint names size is not 6!");
    }
    if (right_joint_names.size() != JOINT_NUM) {
        RCLCPP_ERROR(this->get_logger(), "Right joint names size is not 6!");
        throw std::runtime_error("Right joint names size is not 6!");
    }

    // 输出所有参数
    RCLCPP_INFO(this->get_logger(), "Left arm state topic: %s", left_arm_state_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Right arm state topic: %s", right_arm_state_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Left joint names: ");
    for (const auto& name : left_joint_names) {
        RCLCPP_INFO(this->get_logger(), "  %s", name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Right joint names: ");
    for (const auto& name : right_joint_names) {
        RCLCPP_INFO(this->get_logger(), "  %s", name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Ref topic: %s", ref_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Watchdog timeout: %.2f seconds", watchdog_timeout_);
    
    // 读取底盘命令相关参数
    enable_chassis_cmd_ = params_.enable_chassis_cmd;
    chassis_cmd_topic_ = params_.chassis_cmd_topic;
    max_linear_x_ = params_.max_linear_x;
    max_linear_y_ = params_.max_linear_y;
    max_angular_z_ = params_.max_angular_z;
    
    RCLCPP_INFO(this->get_logger(), "Chassis command enabled: %s", 
                enable_chassis_cmd_ ? "true" : "false");
    if (enable_chassis_cmd_) {
        RCLCPP_INFO(this->get_logger(), "Chassis cmd topic: %s", chassis_cmd_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Max velocities - linear_x: %.2f, linear_y: %.2f, angular_z: %.2f",
                    max_linear_x_, max_linear_y_, max_angular_z_);
    }
    
    // 初始化看门狗
    last_command_time_ = this->now();
    watchdog_triggered_ = false;
    
    // 创建发布者
    left_arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(left_arm_state_topic, 10);
    right_arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(right_arm_state_topic, 10);
    if (enable_chassis_cmd_) {
        chassis_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(chassis_cmd_topic_, 10);
    }
    // 创建订阅者
    ref_topic_sub_ = this->create_subscription<rm_message::msg::CustomController>(
        ref_topic,
        10,
        std::bind(&RmCustomControllerState::ref_topic_callback, this, std::placeholders::_1)
    );
    
    // 创建看门狗定时器（每100ms检查一次）
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RmCustomControllerState::watchdog_callback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "RmCustomControllerState node initialized with watchdog enabled");

}

void RmCustomControllerState::ref_topic_callback(const rm_message::msg::CustomController::SharedPtr msg)
{
    if(msg == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Received null message on ref_topic");
        return;
    }
    
    // 更新看门狗时间戳
    last_command_time_ = this->now();
    if (watchdog_triggered_) {
        RCLCPP_INFO(this->get_logger(), "Command received, watchdog reset");
        watchdog_triggered_ = false;
    }

    // 输出 msg
    // RCLCPP_INFO(this->get_logger(), "Received RobotCustomData message with data size: %zu", msg->data.size());
    // for(size_t i = 0; i < msg->data.size(); ++i) {
    //     RCLCPP_INFO(this->get_logger(), "  data[%zu]: %u", i, msg->data[i]);
    // }

    // 解析裁判系统自定义数据
    ControlData control_data;
    rclcpp::Time now = this->now();
    memcpy(&control_data, msg->data.data(), sizeof(ControlData));

    // 输出 control_data
    RCLCPP_DEBUG(this->get_logger(), "Received custom controller data:");
    RCLCPP_DEBUG(this->get_logger(), "  Rotor Angles: ");
    for (size_t i = 0; i < 12; ++i) {
        RCLCPP_DEBUG(this->get_logger(), "    %d", control_data.rotor_angles[i]);
    }
    RCLCPP_DEBUG(this->get_logger(), "  Channels: %d, %d, %d, %d",
                 control_data.channel_0,
                 control_data.channel_1,
                 control_data.channel_2,
                 control_data.channel_3);
    RCLCPP_DEBUG(this->get_logger(), "  GPIO State: %d", control_data.gpio_state);

    // 发布左机械臂关节状态
    auto left_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    left_joint_state_msg->header.stamp = now;
    left_joint_state_msg->name = left_joint_names;
    left_joint_state_msg->position.resize(JOINT_NUM, 0.0);
    left_joint_state_msg->velocity.resize(JOINT_NUM, 0.0);
    left_joint_state_msg->effort.resize(JOINT_NUM, 0.0);
    for (size_t i = 0; i < JOINT_NUM; ++i) {
        left_joint_state_msg->position[i] = uint_to_float(control_data.rotor_angles[i], POS_MIN, POS_MAX, BITS);
    }
    // 输出 left_joint_state_msg
    RCLCPP_DEBUG(this->get_logger(), "Publishing left arm joint states:");
    for (size_t i = 0; i < JOINT_NUM; ++i) {
        RCLCPP_DEBUG(this->get_logger(), "  %s: %.4f", left_joint_names[i].c_str(), left_joint_state_msg->position[i]);
    }
    left_arm_state_pub_->publish(*left_joint_state_msg);

    // 发布右机械臂关节状态
    auto right_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    right_joint_state_msg->header.stamp = now;
    right_joint_state_msg->name = right_joint_names;
    right_joint_state_msg->position.resize(JOINT_NUM, 0.0);
    right_joint_state_msg->velocity.resize(JOINT_NUM, 0.0);
    right_joint_state_msg->effort.resize(JOINT_NUM, 0.0);
    for (size_t i = 0; i < JOINT_NUM; ++i) {
        right_joint_state_msg->position[i] = uint_to_float(control_data.rotor_angles[i + 6], POS_MIN, POS_MAX, BITS);
    }
    // 输出 right_joint_state_msg
    RCLCPP_DEBUG(this->get_logger(), "Publishing right arm joint states:");
    for (size_t i = 0; i < JOINT_NUM; ++i) {
        RCLCPP_DEBUG(this->get_logger(), "  %s: %.4f", right_joint_names[i].c_str(), right_joint_state_msg->position[i]);
    }
    right_arm_state_pub_->publish(*right_joint_state_msg);

    // 发布底盘命令
    publishChassisCommand(
        control_data.channel_0,
        control_data.channel_1,
        control_data.channel_2,
        control_data.channel_3
    );

    // RCLCPP_DEBUG(this->get_logger(), "Published joint states from custom controller data");

}

void RmCustomControllerState::publishChassisCommand(
    int8_t ch0, int8_t ch1, int8_t ch2, int8_t ch3)
{
    if (!enable_chassis_cmd_) {
        return;
    }
    
    // 归一化通道值到 [-1.0, 1.0]
    // int8_t [-128, 127] -> float [-1.0, 1.0]
    float norm_ch0 = std::clamp(static_cast<float>(ch0) / 127.0f, -1.0f, 1.0f);
    float norm_ch1 = std::clamp(static_cast<float>(ch1) / 127.0f, -1.0f, 1.0f);
    float norm_ch2 = std::clamp(static_cast<float>(ch2) / 127.0f, -1.0f, 1.0f);
    // ch3 预留未使用
    (void)ch3;  // 抑制未使用参数警告
    
    // 创建 TwistStamped 消息
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";
    
    // 映射到实际速度
    twist_msg.twist.linear.x = norm_ch0 * max_linear_x_;
    twist_msg.twist.linear.y = norm_ch1 * max_linear_y_;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = norm_ch2 * max_angular_z_;
    
    // 发布
    chassis_cmd_pub_->publish(twist_msg);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Published chassis command: linear=[%.3f, %.3f], angular=%.3f",
        twist_msg.twist.linear.x, twist_msg.twist.linear.y, 
        twist_msg.twist.angular.z);
}

void RmCustomControllerState::watchdog_callback()
{
    rclcpp::Time current_time = this->now();
    double time_since_last_command = (current_time - last_command_time_).seconds();
    
    if (time_since_last_command > watchdog_timeout_) {
        if (!watchdog_triggered_) {
            RCLCPP_WARN(this->get_logger(), 
                "WATCHDOG ALERT: No command received for %.2f seconds (timeout: %.2f seconds)",
                time_since_last_command, watchdog_timeout_);
            watchdog_triggered_ = true;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "WATCHDOG: Still no command (%.2f seconds elapsed)", time_since_last_command);
        }
    }
}


} // namespace rm_custom_controller_state

// 注册为 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_custom_controller_state::RmCustomControllerState)