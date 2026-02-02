#ifndef RM_CUSTOM_CONTROLLER_STATE__RM_CUSTOM_CONTROLLER_STATE_HPP_
#define RM_CUSTOM_CONTROLLER_STATE__RM_CUSTOM_CONTROLLER_STATE_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>


#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

#include "rm_message/msg/custom_controller.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rm_custom_controller_state/rm_custom_controller_state_parameters.hpp"

namespace rm_custom_controller_state
{

const float POS_MIN = -3.1415926535f;  // -π
const float POS_MAX = 3.1415926535f;   // +π
const int BITS = 16;                   // 压缩到16位
const int JOINT_NUM = 6;                // 每个机械臂关节数

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_uint, float x_min, float x_max, int bits);

typedef struct {
    /**
     * \brief 12个电机的机械角度 (24字节)
     * \brief 顺序为 left_j0 - left_j5 right_j0 - right_j5
     */
    uint16_t rotor_angles[12];
    uint8_t channel_0;
    uint8_t channel_1;
    uint8_t channel_2;
    uint8_t channel_3;
    /**
     * \brief 8个gpio状态,每个bit标识一个，未使用的为后续功能预留 (8字节)
     */
    uint8_t gpio_state;
    uint8_t reserved;
} ControlData;

/**
 * \class RmCustomControllerState
 * \brief 用于从裁判系统接受自定义控制器状态信息的类
 */
class RmCustomControllerState : public rclcpp::Node{
public:
    RmCustomControllerState(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RmCustomControllerState() = default;

private:

    // 左机械臂关节状态topic
    std::string left_arm_state_topic;
    // 右机械臂关节状态topic
    std::string right_arm_state_topic;

    // 左机械臂关节名称，长度为6
    std::vector<std::string> left_joint_names;
    // 右机械臂关节名称，长度为6
    std::vector<std::string> right_joint_names;

    // 裁判系统的自定义数据topic
    std::string ref_topic;

    // 参数lisener
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // 裁判系统自定义数据的订阅者
    rclcpp::Subscription<rm_message::msg::CustomController>::SharedPtr ref_topic_sub_;

    // 左机械臂关节状态的发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_state_pub_;
    // 右机械臂关节状态的发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_state_pub_;
    // 底盘命令发布者
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_pub_;

    void ref_topic_callback(const rm_message::msg::CustomController::SharedPtr msg);

    // 底盘命令相关成员
    bool enable_chassis_cmd_;
    std::string chassis_cmd_topic_;
    std::vector<std::string> channel_mapping_;
    std::vector<double> channel_max_;
    
    std::array<const std::string*, 4> channel_mappings_;
    std::array<double, 4> channel_maxs_;
    
    void publish_chassis_command(uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3);
    float apply_channel_mapping(const std::string& mapping, const std::array<uint8_t, 4>& channels);

    bool previous_gpio_state_ = false;
    bool current_gpio_state_ = false;
    bool gpio_state_changed_ = false;
    int counter_ = 0;
    double gpio_pub_period_ = 1.0;
    rclcpp::Time last_gpio_pub_time_;
    double counter_1_pub_value_ = 1.0;
    double counter_0_pub_value_ = -1.0;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> gpio_pub_;
    std::string gpio_pub_topic_ = "/right_gripper_controller/command";
    void gpio_pub(bool state);

    // 看门狗相关成员
    rclcpp::Time last_command_time_;              // 最后一次接收到命令的时间
    rclcpp::TimerBase::SharedPtr watchdog_timer_; // 看门狗定时器
    double watchdog_timeout_;                     // 看门狗超时时间（秒）
    bool watchdog_triggered_;                     // 看门狗是否已触发
    
    void watchdog_callback();                     // 看门狗检查回调函数

};

} // namespace rm_custom_controller_state

#endif  // RM_CUSTOM_CONTROLLER_STATE__RM_CUSTOM_CONTROLLER_STATE_HPP_