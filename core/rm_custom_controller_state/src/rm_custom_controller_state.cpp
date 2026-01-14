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

    // 检查 left_joint_names 与 right_joint_names 长度是否为6
    if (left_joint_names.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "Left joint names size is not 6!");
        throw std::runtime_error("Left joint names size is not 6!");
    }
    if (right_joint_names.size() != 6) {
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
    // 创建发布者
    left_arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(left_arm_state_topic, 10);
    right_arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(right_arm_state_topic, 10);
    // 创建订阅者
    ref_topic_sub_ = this->create_subscription<rm_message::msg::RobotCustomData>(
        ref_topic,
        10,
        std::bind(&RmCustomControllerState::ref_topic_callback, this, std::placeholders::_1)
    );

}

void RmCustomControllerState::ref_topic_callback(const rm_message::msg::RobotCustomData::SharedPtr msg)
{
    if(msg == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Received null message on ref_topic");
        return;
    }

    // 解析裁判系统自定义数据
    ControlData control_data;
    rclcpp::Time now = this->now();
    memccpy(&control_data, msg->data.data(), 0, sizeof(ControlData));

    // 发布左机械臂关节状态
    auto left_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    left_joint_state_msg->header.stamp = now;
    left_joint_state_msg->name = left_joint_names;
    left_joint_state_msg->position.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        left_joint_state_msg->position[i] = uint_to_float(control_data.rotor_angles[i], POS_MIN, POS_MAX, BITS);
    }
    left_arm_state_pub_->publish(*left_joint_state_msg);

    // 发布右机械臂关节状态
    auto right_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    right_joint_state_msg->header.stamp = now;
    right_joint_state_msg->name = right_joint_names;
    right_joint_state_msg->position.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        right_joint_state_msg->position[i] = uint_to_float(control_data.rotor_angles[i + 6], POS_MIN, POS_MAX, BITS);
    }
    right_arm_state_pub_->publish(*right_joint_state_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published joint states from custom controller data");

}


} // namespace rm_custom_controller_state

// 注册为 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_custom_controller_state::RmCustomControllerState)