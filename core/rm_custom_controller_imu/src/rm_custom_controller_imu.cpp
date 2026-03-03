#include "rm_custom_controller_imu/rm_custom_controller_imu.hpp"

namespace rm_custom_controller_imu
{

RmCustomControllerImu::RmCustomControllerImu(const rclcpp::NodeOptions & options)
: Node("rm_custom_controller_imu", options)
{
    // 获取参数
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // 填充参数
    ref_topic_ = params_.ref_topic;
    watchdog_timeout_ = params_.watchdog_timeout;
    
    // ControlData1 (TF2) 参数
    parent_frame_ = params_.parent_frame;
    child_frame_ = params_.child_frame;
    pose_topic_ = params_.pose_topic;
    position_scale_x_ = params_.position_scale_x;
    position_scale_y_ = params_.position_scale_y;
    position_scale_z_ = params_.position_scale_z;
    
    // ControlData2 (Twist) 参数
    enable_twist_cmd_ = params_.enable_twist_cmd;
    twist_cmd_topic_ = params_.twist_cmd_topic;
    channel_mapping_ = params_.channel_mapping;
    channel_max_ = params_.channel_max;

    // 验证channel_mapping和channel_max的长度
    if (channel_mapping_.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "channel_mapping size must be 4, got %zu", channel_mapping_.size());
        throw std::runtime_error("channel_mapping size must be 4");
    }
    if (channel_max_.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "channel_max size must be 4, got %zu", channel_max_.size());
        throw std::runtime_error("channel_max size must be 4");
    }

    // 输出参数信息
    RCLCPP_INFO(this->get_logger(), "=== RmCustomControllerImu Parameters ===");
    RCLCPP_INFO(this->get_logger(), "Ref topic: %s", ref_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Watchdog timeout: %.2f seconds", watchdog_timeout_);
    RCLCPP_INFO(this->get_logger(), "--- ControlData1 (0xA1) TF2 Parameters ---");
    RCLCPP_INFO(this->get_logger(), "  Parent frame: %s", parent_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Child frame: %s", child_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Pose topic: %s", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Position scale: X=%.3f, Y=%.3f, Z=%.3f", 
                position_scale_x_, position_scale_y_, position_scale_z_);
    RCLCPP_INFO(this->get_logger(), "--- ControlData2 (0xA2) Twist Parameters ---");
    RCLCPP_INFO(this->get_logger(), "  Twist command enabled: %s", enable_twist_cmd_ ? "true" : "false");
    if (enable_twist_cmd_) {
        RCLCPP_INFO(this->get_logger(), "  Twist cmd topic: %s", twist_cmd_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Channel mappings: ch0=%s, ch1=%s, ch2=%s, ch3=%s",
                    channel_mapping_[0].c_str(), channel_mapping_[1].c_str(),
                    channel_mapping_[2].c_str(), channel_mapping_[3].c_str());
        RCLCPP_INFO(this->get_logger(), "  Channel max values: ch0=%.2f, ch1=%.2f, ch2=%.2f, ch3=%.2f",
                    channel_max_[0], channel_max_[1], channel_max_[2], channel_max_[3]);
    }

    // 初始化看门狗
    last_command_time_ = this->now();
    watchdog_triggered_ = false;

    // 创建TF2广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(this->get_logger(), "TF2 broadcaster created");

    // 创建Pose发布者
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    RCLCPP_INFO(this->get_logger(), "Pose publisher created");

    // 创建Twist命令发布者
    if (enable_twist_cmd_) {
        twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_cmd_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "Twist command publisher created");
    }

    // 创建GPIO发布者
    for (int i = 0; i < GPIO_NUM; ++i) {
        std::string gpio_topic = "~/gpio" + std::to_string(i) + "_state";
        gpio_publishers_[i] = this->create_publisher<std_msgs::msg::Bool>(gpio_topic, 10);
        RCLCPP_DEBUG(this->get_logger(), "Created GPIO publisher: %s", gpio_topic.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Created %d GPIO publishers (gpio0_state ~ gpio7_state)", GPIO_NUM);

    // 创建订阅者
    ref_topic_sub_ = this->create_subscription<rm_message::msg::CustomController>(
        ref_topic_,
        10,
        std::bind(&RmCustomControllerImu::ref_topic_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", ref_topic_.c_str());

    // 创建看门狗定时器（每100ms检查一次）
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RmCustomControllerImu::watchdog_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "=== RmCustomControllerImu initialized successfully ===");
}

void RmCustomControllerImu::ref_topic_callback(const rm_message::msg::CustomController::SharedPtr msg)
{
    if (msg == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Received null message on ref_topic");
        return;
    }

    // 检查数据长度
    if (msg->data.size() != 30) {
        RCLCPP_WARN(this->get_logger(), "Invalid message size: %zu (expected 30)", msg->data.size());
        return;
    }

    // 更新看门狗时间戳
    last_command_time_ = this->now();
    if (watchdog_triggered_) {
        RCLCPP_INFO(this->get_logger(), "Command received, watchdog reset");
        watchdog_triggered_ = false;
    }

    // 读取数据包头
    uint8_t header = msg->data[0];

    // 根据包头分发处理
    if (header == 0xA1) {
        // ControlData1: 四元数 + 位置
        ControlData1 data1;
        std::memcpy(&data1, msg->data.data(), sizeof(ControlData1));
        process_control_data1(data1);
        RCLCPP_DEBUG(this->get_logger(), "Processed ControlData1 (0xA1)");
    }
    else if (header == 0xA2) {
        // ControlData2: 通道 + GPIO
        ControlData2 data2;
        std::memcpy(&data2, msg->data.data(), sizeof(ControlData2));
        process_control_data2(data2);
        RCLCPP_DEBUG(this->get_logger(), "Processed ControlData2 (0xA2)");
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Unknown packet header: 0x%02X", header);
    }
}

void RmCustomControllerImu::process_control_data1(const ControlData1& data)
{
    // 构造TF2变换消息
    geometry_msgs::msg::TransformStamped transform_stamped;

    // 设置时间戳和坐标系
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = parent_frame_;
    transform_stamped.child_frame_id = child_frame_;

    // 设置位置（应用缩放）
    transform_stamped.transform.translation.x = data.pos_x * position_scale_x_;
    transform_stamped.transform.translation.y = data.pos_y * position_scale_y_;
    transform_stamped.transform.translation.z = data.pos_z * position_scale_z_;

    // 设置旋转（四元数）
    transform_stamped.transform.rotation.w = data.qw;
    transform_stamped.transform.rotation.x = data.qx;
    transform_stamped.transform.rotation.y = data.qy;
    transform_stamped.transform.rotation.z = data.qz;

    // 发布TF2变换
    tf_broadcaster_->sendTransform(transform_stamped);

    // 构造并发布Pose消息
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = transform_stamped.header.stamp;
    pose_msg.header.frame_id = parent_frame_;
    
    // 从transform复制位置和姿态
    pose_msg.pose.position.x = transform_stamped.transform.translation.x;
    pose_msg.pose.position.y = transform_stamped.transform.translation.y;
    pose_msg.pose.position.z = transform_stamped.transform.translation.z;
    pose_msg.pose.orientation.w = transform_stamped.transform.rotation.w;
    pose_msg.pose.orientation.x = transform_stamped.transform.rotation.x;
    pose_msg.pose.orientation.y = transform_stamped.transform.rotation.y;
    pose_msg.pose.orientation.z = transform_stamped.transform.rotation.z;
    
    pose_pub_->publish(pose_msg);

    RCLCPP_DEBUG(this->get_logger(), 
                 "Published TF2: %s -> %s | Pos(%.3f, %.3f, %.3f) Quat(%.3f, %.3f, %.3f, %.3f)",
                 parent_frame_.c_str(), child_frame_.c_str(),
                 transform_stamped.transform.translation.x,
                 transform_stamped.transform.translation.y,
                 transform_stamped.transform.translation.z,
                 data.qw, data.qx, data.qy, data.qz);
}

void RmCustomControllerImu::process_control_data2(const ControlData2& data)
{
    // 处理Twist命令
    if (enable_twist_cmd_) {
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link";

        // 初始化为0
        twist_msg.twist.linear.x = 0.0;
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        // 处理4个通道，归一化到[-1, 1]，然后乘以最大值
        int8_t channels[4] = {data.channel_0, data.channel_1, data.channel_2, data.channel_3};
        
        for (int i = 0; i < 4; ++i) {
            // 将int8_t [-128, 127] 归一化到 [-1.0, 1.0]
            double normalized = static_cast<double>(channels[i]) / 127.0;
            
            // 乘以最大值
            double value = normalized * channel_max_[i];
            
            // 根据映射赋值
            const std::string& mapping = channel_mapping_[i];
            if (mapping == "linear_x") {
                twist_msg.twist.linear.x = value;
            } else if (mapping == "linear_y") {
                twist_msg.twist.linear.y = value;
            } else if (mapping == "angular_z") {
                twist_msg.twist.angular.z = value;
            }
            // "none" 或其他值不做处理
        }

        // 发布Twist消息
        twist_cmd_pub_->publish(twist_msg);

        RCLCPP_DEBUG(this->get_logger(), 
                     "Published Twist: linear(%.3f, %.3f, %.3f) angular(%.3f, %.3f, %.3f)",
                     twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z,
                     twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z);
    }

    // 处理GPIO状态（8个bit）
    for (int i = 0; i < GPIO_NUM; ++i) {
        std_msgs::msg::Bool gpio_msg;
        gpio_msg.data = (data.gpio_state >> i) & 0x01;  // 提取第i位
        gpio_publishers_[i]->publish(gpio_msg);
    }

    RCLCPP_DEBUG(this->get_logger(), "Published GPIO states: 0x%02X", data.gpio_state);
}

void RmCustomControllerImu::watchdog_callback()
{
    // 检查是否超时
    rclcpp::Duration time_since_last_command = this->now() - last_command_time_;
    
    if (time_since_last_command.seconds() > watchdog_timeout_) {
        if (!watchdog_triggered_) {
            RCLCPP_WARN(this->get_logger(), 
                        "Watchdog timeout! No command received for %.2f seconds (timeout: %.2f)",
                        time_since_last_command.seconds(), watchdog_timeout_);
            watchdog_triggered_ = true;
        }
    }
}

} // namespace rm_custom_controller_imu

// 注册为ROS2组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_custom_controller_imu::RmCustomControllerImu)
