#include "rm_manager/rm_manager.hpp"
#include "rm_manager/util.hpp"

namespace RMManager
{

namespace
{

const char * link_type_name(LinkType link_type)
{
    switch (link_type) {
      case LinkType::Image:
          return "image";
      case LinkType::Referee:
          return "referee";
    }
    return "unknown";
}

}  // namespace

RMManagerNode::RMManagerNode(std::string name)
  : Node(name)
{
    // 初始化串口对象
    this->declare_parameter<std::string>("image_port", "/dev/ttyImage");
    this->declare_parameter<std::string>("referee_port", "/dev/ttyRef");

    std::string image_port = this->get_parameter("image_port").as_string();
    std::string referee_port = this->get_parameter("referee_port").as_string();

    try {
        // 创建状态发布者
        image_status_pub_ =
          this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) +
        "/image_port_status", 10);
        referee_status_pub_ =
          this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) +
        "/referee_port_status", 10);
        remote_control_pub_ =
          this->create_publisher<rm_message::msg::RemoteControl>(std::string(this->get_name()) +
        "/remote_control", 10);

        debugger_pub_ =
          this->create_publisher<rm_message::msg::GeneralMessage>(std::string(this->get_name()) +
        "/all_receive_data", 10);

        // 初始化消息发布管理器
        msg_publisher_ = std::make_unique<MsgPublisher>(this);

    } catch(const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception when create publishers: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "Publishers initialized.");

    try {

        if(image_port != "" && image_port != "None") {
            image_uart_ = std::make_shared<SerialCommunicator>(image_port, 921600);
            image_uart_->register_read_callback(std::bind(
                &RMManagerNode::_read_callback,
                this,
                std::placeholders::_1,
                LinkType::Image,
                std::ref(image_send_)));
            if (!image_uart_->openPort()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open image port: %s",
                      image_port.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Image port opened: %s", image_port.c_str());
            }
        }
        if(referee_port != "" && referee_port != "None") {
            referee_uart_ = std::make_shared<SerialCommunicator>(referee_port, 115200);
            referee_uart_->register_read_callback(std::bind(
                &RMManagerNode::_read_callback,
                this,
                std::placeholders::_1,
                LinkType::Referee,
                std::ref(referee_send_)));
            if (!referee_uart_->openPort()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open referee port: %s",
                      referee_port.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Referee port opened: %s", referee_port.c_str());
            }
        }
    } catch(const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception when open port: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "Serial ports initialized.");

    try {
        // 创建监测图传链路状态的线程
        if(image_port != "" && image_port != "None") {
            image_check_thread_ = std::make_unique<std::thread>([this]() {
                    int nomessage_times = 0;
                    rclcpp::Rate rate(1); // 1 Hz
                    while (rclcpp::ok()) {
                        rate.sleep();
                        if (image_send_) {
                            nomessage_times = 0;
                            image_send_ = false;
                        } else {
                            nomessage_times++;
                        }


                    // 检查串口状态
                        if (!image_uart_->isPortOK()) {
                            if(image_uart_->reopenPort() && image_uart_->startRead()) {
                                RCLCPP_INFO(this->get_logger(),
                  "Error Occurred: Image port re-opened and reading started successfully.");
                            } else {
                                nomessage_times = 3;
                                RCLCPP_ERROR(this->get_logger(),
                                  "Error Occurred: Image port re-open failed.");
                            }
                        }

                        std_msgs::msg::Bool status_msg;
                        if (nomessage_times >= 3) {
                            status_msg.data = false;
                            RCLCPP_WARN(this->get_logger(), "Image link seems offline!");
                        // 尝试重启串口
                            if(image_uart_->reopenPort() && image_uart_->startRead()) {
                                RCLCPP_INFO(this->get_logger(),
                  "Image port re-opened and reading started successfully.");
                            } else {
                                nomessage_times = 3;
                                RCLCPP_ERROR(this->get_logger(), "Image port re-open failed.");
                            }
                        } else {
                            status_msg.data = true;
                        }
                        image_status_pub_->publish(status_msg);
                    }
            });
        }
    } catch(const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception when create image check thread: %s", e.what());
    }

    try {
        // 创建监测裁判系统链路状态的线程
        if(referee_port != "" && referee_port != "None") {
            referee_check_thread_ = std::make_unique<std::thread>([this]() {
                    int nomessage_times = 0;
                    rclcpp::Rate rate(1); // 1 Hz
                    while (rclcpp::ok()) {
                        rate.sleep();
                        if (referee_send_) {
                            nomessage_times = 0;
                            referee_send_ = false;
                        } else {
                            nomessage_times++;
                        }

                    // 检查串口状态
                        if (!referee_uart_->isPortOK()) {
                            if(referee_uart_->reopenPort() && referee_uart_->startRead()) {
                                RCLCPP_INFO(this->get_logger(),
                  "Error Occurred: Referee port re-opened and reading started successfully.");
                            } else {
                                nomessage_times = 3;
                                RCLCPP_ERROR(this->get_logger(),
                                  "Error Occurred: Referee port re-open failed.");
                            }
                        }

                        std_msgs::msg::Bool status_msg;
                        if (nomessage_times >= 3) {
                            status_msg.data = false;
                            RCLCPP_WARN(this->get_logger(), "Referee link seems offline!");
                        // 尝试重启串口
                            if(referee_uart_->reopenPort() && referee_uart_->startRead()) {
                                RCLCPP_INFO(this->get_logger(),
                  "Referee port re-opened and reading started successfully.");
                            } else {
                                nomessage_times = 3;
                                RCLCPP_ERROR(this->get_logger(), "Referee port re-open failed.");
                            }
                        } else {
                            status_msg.data = true;
                        }
                        referee_status_pub_->publish(status_msg);
                    }
            });
        }
    } catch(const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception when create referee check thread: %s",
              e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Link check threads initialized.");

    // 启动串口读取
    if(image_uart_) {
        if(image_uart_->startRead()) {
            RCLCPP_INFO(this->get_logger(), "Image port reading started successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start reading from image port.");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Image port is disabled (not initialized).");
    }

    if(referee_uart_) {
        if(referee_uart_->startRead()) {
            RCLCPP_INFO(this->get_logger(), "Referee port reading started successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start reading from referee port.");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Referee port is disabled (not initialized).");
    }

    RCLCPP_INFO(this->get_logger(), "Serial ports initialized and start read.");

    // 创建接受数据的sub
    send_sub_ = this->create_subscription<rm_message::msg::SendMessage>(
            "send_message", 10,
            std::bind(&RMManagerNode::_send_sub_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "RMManagerNode initialized.");

}

RMManagerNode::~RMManagerNode()
{
    if(image_check_thread_ && image_check_thread_->joinable()) {
        image_check_thread_->join();
    }
    if(referee_check_thread_ && referee_check_thread_->joinable()) {
        referee_check_thread_->join();
    }
}

void RMManagerNode::_read_callback(
    const std::vector<uint8_t> & data,
    LinkType link_type,
    std::atomic<bool> & link_status)
{
    // debugger 发布原始数据
    rm_message::msg::GeneralMessage debug_msg;
    debug_msg.cmd_id = 0xFFFF; // 特殊cmd_id表示原始数据
    debug_msg.data_length = data.size();
    debug_msg.data_payload = data;

    debugger_pub_->publish(debug_msg);

    auto & pending_buffer =
      link_type == LinkType::Image ? image_pending_buffer_ : referee_pending_buffer_;
    const auto parser_mode =
      link_type == LinkType::Image ? StreamParserMode::kImageLink :
      StreamParserMode::kStandardOnly;

    pending_buffer.insert(pending_buffer.end(), data.begin(), data.end());

    while (true) {
        ParsedPacket packet;
        const auto outcome = extract_next_packet(pending_buffer, parser_mode, packet);

        switch (outcome.result) {
          case StreamParseResult::kOk:
              _handle_parsed_packet(packet, link_type, link_status);
              break;
          case StreamParseResult::kNeedMoreData:
              return;
          case StreamParseResult::kSkippedBytes:
              _log_stream_parser_event(outcome.event, link_type);
              break;
        }
    }
}

void RMManagerNode::_handle_parsed_packet(
    const ParsedPacket & packet,
    LinkType link_type,
    std::atomic<bool> & link_status)
{
    switch (packet.type) {
      case ParsedPacketType::kStandardFrame:
          msg_publisher_->publish(link_type, packet.frame.command_id, packet.frame.payload);
          break;
      case ParsedPacketType::kLegacyRemoteControl:
          remote_control_pub_->publish(packet.remote_control);
          break;
    }

    link_status = true;
}

void RMManagerNode::_log_stream_parser_event(
    StreamParserEvent event,
    LinkType link_type) const
{
    switch (event) {
      case StreamParserEvent::kNone:
      case StreamParserEvent::kSkippedNoise:
      case StreamParserEvent::kInvalidLegacyHeader:
          return;
      case StreamParserEvent::kInvalidStandardCrc8:
          RCLCPP_WARN(this->get_logger(), "CRC8 check failed on %s link.",
                link_type_name(link_type));
          return;
      case StreamParserEvent::kInvalidStandardLength:
          RCLCPP_WARN(this->get_logger(), "Frame length is invalid on %s link.",
                link_type_name(link_type));
          return;
      case StreamParserEvent::kInvalidStandardCrc16:
          RCLCPP_WARN(this->get_logger(), "CRC16 check failed on %s link.",
                link_type_name(link_type));
          return;
      case StreamParserEvent::kInvalidLegacyCrc16:
          RCLCPP_WARN(this->get_logger(),
                "Legacy remote-control CRC16 check failed on %s link.",
                link_type_name(link_type));
          return;
    }
}

void RMManagerNode::_send_sub_callback(const rm_message::msg::SendMessage::SharedPtr msg)
{

    // 处理帧头
    std::vector<uint8_t> frame;
    FrameHeader header = {};
    header.sof = 0xA5;
    // data_length 应与实际 payload 长度保持一致，防止接收端按错误长度校验 CRC
    header.data_length = msg->data_payload.size();
    header.seq = 0;
    header.crc8 = Get_CRC8_Check_Sum((uint8_t *)&header, sizeof(FrameHeader) - 1);

    frame.insert(frame.end(), (uint8_t *)&header, (uint8_t *)&header + sizeof(FrameHeader));

    // 处理命令字
    frame.push_back(msg->cmd_id & 0xFF);
    frame.push_back((msg->cmd_id >> 8) & 0xFF);

    // 处理数据部分
    frame.insert(frame.end(), msg->data_payload.begin(), msg->data_payload.end());

    // 处理crc16
    // CRC16 应覆盖帧头 + cmd_id + payload，保持与接收端一致
    uint16_t crc16 = Get_CRC16_Check_Sum(frame.data(), frame.size());
    frame.push_back(crc16 & 0xFF);
    frame.push_back((crc16 >> 8) & 0xFF);

    // 发送数据
    if(msg->target == 1 && image_uart_ && image_uart_->isPortOK()) {
        image_uart_->writeData(frame);
    } else if(msg->target == 2 && referee_uart_ && referee_uart_->isPortOK()) {
        referee_uart_->writeData(frame);
    } else {
        RCLCPP_ERROR(this->get_logger(), "SendMessage target error or port not open! target: %d",
        msg->target);
    }

}

} // namespace RMManager
