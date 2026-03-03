#ifndef RM_CUSTOM_CONTROLLER_IMU__RM_CUSTOM_CONTROLLER_IMU_HPP_
#define RM_CUSTOM_CONTROLLER_IMU__RM_CUSTOM_CONTROLLER_IMU_HPP_

#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <memory>
#include <cstring>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "rm_message/msg/custom_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "rm_custom_controller_imu/rm_custom_controller_imu_parameters.hpp"

namespace rm_custom_controller_imu
{

constexpr int GPIO_NUM = 8;  // GPIO 数量

/**
 * \brief ControlData1 - 四元数 + 位置数据 (30字节)
 * \brief 数据头: 0xA1
 */
typedef struct {
    uint8_t header;     // 数据头，固定为0xA1 (1字节)
    float qw;           // 四元数实部 (4字节)
    float qx;           // 四元数 i 分量 (4字节)
    float qy;           // 四元数 j 分量 (4字节)
    float qz;           // 四元数 k 分量 (4字节)
    float pos_x;        // X坐标 (4字节)
    float pos_y;        // Y坐标 (4字节)
    float pos_z;        // Z坐标 (4字节)
    uint8_t reserved;   // 保留字节 (1字节)
} __attribute__((packed)) ControlData1;  // 总计30字节

/**
 * \brief ControlData2 - 通道 + GPIO 数据 (30字节)
 * \brief 数据头: 0xA2
 */
typedef struct {
    uint8_t header;      // 数据头，固定为0xA2 (1字节)
    int8_t channel_0;    // 通道0 (1字节) - 左摇杆水平
    int8_t channel_1;    // 通道1 (1字节) - 左摇杆垂直
    int8_t channel_2;    // 通道2 (1字节) - 右摇杆水平
    int8_t channel_3;    // 通道3 (1字节) - 右摇杆垂直
    uint8_t gpio_state;  // GPIO状态 (1字节) - bit0和bit1对应左右按钮
    uint8_t reserved[24]; // 填充到30字节 (24字节)
} __attribute__((packed)) ControlData2;  // 总计30字节

/**
 * \class RmCustomControllerImu
 * \brief 用于从裁判系统接收自定义控制器 IMU 数据的类
 * \brief 处理两种数据包: ControlData1 (0xA1) 和 ControlData2 (0xA2)
 */
class RmCustomControllerImu : public rclcpp::Node {
public:
    RmCustomControllerImu(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RmCustomControllerImu() = default;

private:
    // 参数监听器
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // 裁判系统的自定义数据topic
    std::string ref_topic_;

    // TF2相关参数 (ControlData1)
    std::string parent_frame_;
    std::string child_frame_;
    std::string pose_topic_;
    double position_scale_x_;
    double position_scale_y_;
    double position_scale_z_;

    // Twist命令相关参数 (ControlData2)
    bool enable_twist_cmd_;
    std::string twist_cmd_topic_;
    std::vector<std::string> channel_mapping_;
    std::vector<double> channel_max_;

    // 看门狗参数
    double watchdog_timeout_;
    rclcpp::Time last_command_time_;
    bool watchdog_triggered_;

    // 裁判系统自定义数据的订阅者
    rclcpp::Subscription<rm_message::msg::CustomController>::SharedPtr ref_topic_sub_;

    // TF2广播器 (ControlData1)
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Pose发布者 (ControlData1)
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // Twist命令发布者 (ControlData2)
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;

    // GPIO状态发布者 (GPIO0-GPIO7)
    std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr, GPIO_NUM> gpio_publishers_;

    // 看门狗定时器
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    /**
     * \brief 裁判系统数据回调函数
     * \param msg 接收到的自定义控制器消息
     */
    void ref_topic_callback(const rm_message::msg::CustomController::SharedPtr msg);

    /**
     * \brief 看门狗定时器回调函数
     */
    void watchdog_callback();

    /**
     * \brief 处理 ControlData1 (0xA1) - 发布TF2变换
     * \param data ControlData1 数据结构
     */
    void process_control_data1(const ControlData1& data);

    /**
     * \brief 处理 ControlData2 (0xA2) - 发布Twist命令和GPIO状态
     * \param data ControlData2 数据结构
     */
    void process_control_data2(const ControlData2& data);

};

} // namespace rm_custom_controller_imu

#endif // RM_CUSTOM_CONTROLLER_IMU__RM_CUSTOM_CONTROLLER_IMU_HPP_
