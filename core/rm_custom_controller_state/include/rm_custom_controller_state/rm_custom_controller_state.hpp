#ifndef RM_CUSTOM_CONTROLLER_STATE__RM_CUSTOM_CONTROLLER_STATE_HPP_
#define RM_CUSTOM_CONTROLLER_STATE__RM_CUSTOM_CONTROLLER_STATE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rm_message/msg/custom_controller.hpp"
#include "rm_custom_controller_state/rm_custom_controller_state_parameters.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rm_custom_controller_state
{

constexpr float POS_MIN = -M_PI * 2;  // -π
constexpr float POS_MAX = M_PI * 2;   // +π
constexpr int BITS = 16;                   // 压缩到16位
constexpr int JOINT_NUM = 7;                // 单机械臂关节数
constexpr int GPIO_NUM = 8;                 // GPIO 数量

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_uint, float x_min, float x_max, int bits);

typedef struct
{
    /**
     * \brief 7个电机的机械角度 (14字节)
     * \brief 顺序为 j0 - j6
     */
    uint16_t rotor_angles[JOINT_NUM];
    uint8_t channel_0;
    uint8_t channel_1;
    uint8_t channel_2;
    uint8_t channel_3;
    /**
     * \brief 8个gpio状态,每个bit标识一个
     */
    uint8_t gpio_state;
    uint8_t reserved[11];
} ControlData;

static_assert(sizeof(ControlData) == 30, "ControlData must be 30 bytes");

/**
 * \class RmCustomControllerState
 * \brief 用于从裁判系统接受自定义控制器状态信息的类
 */
class RmCustomControllerState : public rclcpp::Node{
public:
    RmCustomControllerState(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RmCustomControllerState() = default;

private:
    // 单机械臂关节状态topic
    std::string arm_state_topic_;
    // 单机械臂关节名称，长度为7
    std::vector<std::string> joint_names_;
    // 单机械臂关节反转标志，长度为7
    std::vector<bool> joint_reverse_;

    // 裁判系统的自定义数据topic
    std::string ref_topic_;

    // 参数lisener
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // 裁判系统自定义数据的订阅者
    rclcpp::Subscription<rm_message::msg::CustomController>::SharedPtr ref_topic_sub_;

    // 单机械臂关节状态的发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
    // 底盘命令发布者
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_pub_;

    // GPIO状态发布者 (GPIO0-GPIO7)
    std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr, GPIO_NUM> gpio_publishers_;

    void ref_topic_callback(const rm_message::msg::CustomController::SharedPtr msg);

    // 底盘命令相关成员
    bool enable_chassis_cmd_;
    std::string chassis_cmd_topic_;
    std::vector<std::string> channel_mapping_;
    std::vector<double> channel_max_;

    std::array<const std::string *, 4> channel_mappings_;
    std::array<double, 4> channel_maxs_;

    void publish_chassis_command(uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3);
    float apply_channel_mapping(
        const std::string & mapping,
        const std::array<uint8_t, 4> & channels);

    // 看门狗相关成员
    rclcpp::Time last_command_time_;              // 最后一次接收到命令的时间
    rclcpp::TimerBase::SharedPtr watchdog_timer_; // 看门狗定时器
    double watchdog_timeout_;                     // 看门狗超时时间（秒）
    bool watchdog_triggered_;                     // 看门狗是否已触发

    void watchdog_callback();                     // 看门狗检查回调函数

};

} // namespace rm_custom_controller_state

#endif  // RM_CUSTOM_CONTROLLER_STATE__RM_CUSTOM_CONTROLLER_STATE_HPP_
