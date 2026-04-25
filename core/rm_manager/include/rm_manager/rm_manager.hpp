#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rm_message/msg/general_message.hpp"
#include "rm_message/msg/send_message.hpp"
#include "rm_message/msg/remote_control.hpp"

#include "rm_manager/frame_parser.hpp"
#include "rm_manager/uart_driver.hpp"
#include "rm_manager/msg_publisher.hpp"

#include <atomic>

# ifndef RM_MANAGER_HPP
# define RM_MANAGER_HPP

namespace RMManager
{

class RMManagerNode : public rclcpp::Node {

public:
    RMManagerNode(std::string name = "rm_manager");
    ~RMManagerNode();

private:
    // 图传链路
    std::shared_ptr<SerialCommunicator> image_uart_;

    // 裁判系统链路
    std::shared_ptr<SerialCommunicator> referee_uart_;

    /**
     * @brief 处理接受到的数据的回调函数，分割数据
     *
     * @param data  接收到的数据
     * @param link_type 当前串口所属链路
     * @param link_status 链路状态的标志位
     */
    void _read_callback(
        const std::vector<uint8_t> & data,
        LinkType link_type,
        std::atomic<bool> & link_status);

    // 图传链路是否发送了消息
    std::atomic<bool> image_send_{true};

    // 裁判系统链路是否发送了消息
    std::atomic<bool> referee_send_{true};

    // 每条链路的待解析缓存，解决串口 read 的半包/粘包问题
    std::vector<uint8_t> image_pending_buffer_;
    std::vector<uint8_t> referee_pending_buffer_;

    // 监测图传链路状态的线程
    std::unique_ptr<std::thread> image_check_thread_;
    // 监测裁判系统链路状态的线程
    std::unique_ptr<std::thread> referee_check_thread_;

    // 图传链路状态的pub
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> image_status_pub_;
    // 裁判系统链路状态的pub
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> referee_status_pub_;

    // legacy 图传私有遥控帧兼容发布
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RemoteControl>> remote_control_pub_;

    // 消息发布管理器 - 处理所有协议消息的发布
    std::unique_ptr<MsgPublisher> msg_publisher_;

    // 接受数据的sub
    std::shared_ptr<rclcpp::Subscription<rm_message::msg::SendMessage>> send_sub_;

    // debugger publisher - 用于调试目的，发布所有串口收到的原始数据
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>> debugger_pub_;


    /**
     * @brief 处理接受到的数据
     *
     */
    void _send_sub_callback(const rm_message::msg::SendMessage::SharedPtr msg);

    /**
     * @brief 处理已经成功解析出的协议包
     *
     * @param packet 解析结果
     * @param link_type 当前链路
     * @param link_status 当前链路活跃状态标志
     */
    void _handle_parsed_packet(
        const ParsedPacket & packet,
        LinkType link_type,
        std::atomic<bool> & link_status);

    /**
     * @brief 记录流式解析过程中被丢弃的数据原因
     *
     * @param event 丢弃原因
     * @param link_type 当前链路
     */
    void _log_stream_parser_event(StreamParserEvent event, LinkType link_type) const;

}; // class RMManagerNode

} // namespace RMManager

# endif // RM_MANAGER_HPP
