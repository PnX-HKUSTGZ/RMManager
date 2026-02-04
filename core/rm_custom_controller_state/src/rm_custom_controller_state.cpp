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
    channel_mapping_ = params_.channel_mapping;
    channel_max_ = params_.channel_max;
    
    // 验证数组长度
    if (channel_mapping_.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "channel_mapping size must be 4, got %zu", channel_mapping_.size());
        throw std::runtime_error("channel_mapping size must be 4");
    }
    if (channel_max_.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "channel_max size must be 4, got %zu", channel_max_.size());
        throw std::runtime_error("channel_max size must be 4");
    }
    
    // 初始化通道映射和最大值数组
    channel_mappings_ = {
        &channel_mapping_[0], &channel_mapping_[1],
        &channel_mapping_[2], &channel_mapping_[3]
    };
    channel_maxs_ = {
        channel_max_[0], channel_max_[1],
        channel_max_[2], channel_max_[3]
    };
    
    RCLCPP_INFO(this->get_logger(), "Chassis command enabled: %s", 
                enable_chassis_cmd_ ? "true" : "false");
    if (enable_chassis_cmd_) {
        RCLCPP_INFO(this->get_logger(), "Chassis cmd topic: %s", chassis_cmd_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Channel mappings - ch0:%s, ch1:%s, ch2:%s, ch3:%s",
                    channel_mapping_[0].c_str(), channel_mapping_[1].c_str(),
                    channel_mapping_[2].c_str(), channel_mapping_[3].c_str());
        RCLCPP_INFO(this->get_logger(), "Channel max values - ch0:%.2f, ch1:%.2f, ch2:%.2f, ch3:%.2f",
                    channel_max_[0], channel_max_[1], channel_max_[2], channel_max_[3]);
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
    
    // 创建GPIO发布者
    for (int i = 0; i < GPIO_NUM; ++i) {
        std::string gpio_topic = "~/gpio" + std::to_string(i) + "_state";
        gpio_publishers_[i] = this->create_publisher<std_msgs::msg::Bool>(gpio_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Created GPIO publisher for topic: %s", gpio_topic.c_str());
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
    
    for (int i = 0; i < GPIO_NUM; ++i) {
        bool gpio_state = (control_data.gpio_state >> i) & 0x01;
        auto gpio_msg = std_msgs::msg::Bool();
        gpio_msg.data = gpio_state;
        gpio_publishers_[i]->publish(gpio_msg);
        RCLCPP_DEBUG(this->get_logger(), "  GPIO%d: %s", i, gpio_state ? "HIGH" : "LOW");
    }

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
    publish_chassis_command(
        control_data.channel_0,
        control_data.channel_1,
        control_data.channel_2,
        control_data.channel_3
    );

    // RCLCPP_DEBUG(this->get_logger(), "Published joint states from custom controller data");

}

float RmCustomControllerState::apply_channel_mapping(
    const std::string& mapping, const std::array<uint8_t, 4>& channels)
{
    if (mapping == "none") return 0.0;
    
    // 查找匹配的通道
    for (size_t i = 0; i < 4; ++i) {
        if (*channel_mappings_[i] == mapping) {
            // uint8_t [0, 255] 归一化到 [-1.0, 1.0]，128为中位
            float normalized = (static_cast<float>(channels[i]) - 127.0f) / 127.0f;
            normalized = std::clamp(normalized, -1.0f, 1.0f);
            return normalized * channel_maxs_[i];
        }
    }
    
    return 0.0;  // 未找到匹配的映射
}

void RmCustomControllerState::publish_chassis_command(
    uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3)
{
    if (!enable_chassis_cmd_) {
        return;
    }
    
    // 组装通道数组
    const std::array<uint8_t, 4> channels = {ch0, ch1, ch2, ch3};
    
    // 创建 TwistStamped 消息
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";
    
    // 根据配置映射通道到速度
    twist_msg.twist.linear.x = apply_channel_mapping("linear_x", channels);
    twist_msg.twist.linear.y = apply_channel_mapping("linear_y", channels);
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = apply_channel_mapping("angular_z", channels);
    
    // 发布
    chassis_cmd_pub_->publish(twist_msg);
    
    RCLCPP_INFO(this->get_logger(), 
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
