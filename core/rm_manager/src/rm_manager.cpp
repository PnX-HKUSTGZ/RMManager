#include "rm_manager/rm_manager.hpp"
#include "rm_manager/util.hpp"

namespace RMManager {

// 定义数据包的最小长度
const uint32_t PACKAGE_MIN_LENGTH = 5;
const uint32_t GENERAL_MESSAGE_HEADER = 0xA5;
const uint32_t IMAGE_OWN_MESSAGE_LENGTH = 21;
const uint32_t IMAGE_OWN_MESSAGE_HEADER[2] = {0xA9, 0x53};

RMManagerNode::RMManagerNode(std::string name) : Node(name) {
    // 初始化串口对象
    this->declare_parameter<std::string>("image_port", "/dev/ttyImage");
    this->declare_parameter<std::string>("referee_port", "/dev/ttyRef");

    std::string image_port = this->get_parameter("image_port").as_string();
    std::string referee_port = this->get_parameter("referee_port").as_string();

    try{
        // 创建状态发布者
        image_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) + "/image_port_status", 10);
        referee_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) + "/referee_port_status", 10);
        // 创建遥控器数据发布者
        remoto_control_pub_ = this->create_publisher<rm_message::msg::RemoteControl>(std::string(this->get_name()) + "/remote_control", 10);

        debugger_pub_ = this->create_publisher<rm_message::msg::GeneralMessage>(std::string(this->get_name()) + "/all_receive_data", 10);

        // 初始化消息发布管理器
        msg_publisher_ = std::make_unique<MsgPublisher>(this);

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when create publishers: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "Publishers initialized.");

    try{

        if(image_port != "" && image_port != "None"){
            image_uart_ = std::make_shared<SerialCommunicator>(image_port, 921600);
            image_uart_->register_read_callback( std::bind(&RMManagerNode::_read_callback, this, std::placeholders::_1, std::ref(image_send_)) );
        }
        if(referee_port != "" && referee_port != "None" ){
            referee_uart_ = std::make_shared<SerialCommunicator>(referee_port, 115200);
            referee_uart_->register_read_callback( std::bind(&RMManagerNode::_read_callback, this, std::placeholders::_1, std::ref(referee_send_)) );
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when open port: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "Serial ports initialized.");

    try{
        // 创建监测图传链路状态的线程
        if(image_port != "" && image_port != "None"){
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
                    if (!image_uart_->isPortOK()){
                        if(image_uart_->reopenPort() && image_uart_->startRead()){
                            RCLCPP_INFO(this->get_logger(), "Error Occurred: Image port re-opened and reading started successfully.");
                        }
                        else{
                            nomessage_times=3;
                            RCLCPP_ERROR(this->get_logger(), "Error Occurred: Image port re-open failed.");
                        }
                    }

                    std_msgs::msg::Bool status_msg;
                    if (nomessage_times >= 3) {
                        status_msg.data = false;
                        RCLCPP_WARN(this->get_logger(), "Image link seems offline!");
                        // 尝试重启串口
                        if(image_uart_->reopenPort() && image_uart_->startRead()){
                            RCLCPP_INFO(this->get_logger(), "Image port re-opened and reading started successfully.");
                        }
                        else{
                            nomessage_times=3;
                            RCLCPP_ERROR(this->get_logger(), "Image port re-open failed.");
                        }
                    } else {
                        status_msg.data = true;
                    }
                    image_status_pub_->publish(status_msg);
                }
            });
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when create image check thread: %s", e.what());
    }

    try{
        // 创建监测裁判系统链路状态的线程
        if(referee_port != "" && referee_port != "None"){
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
                    if (!referee_uart_->isPortOK()){
                        if(referee_uart_->reopenPort() && referee_uart_->startRead()){
                            RCLCPP_INFO(this->get_logger(), "Error Occurred: Referee port re-opened and reading started successfully.");
                        }
                        else{
                            nomessage_times=3;
                            RCLCPP_ERROR(this->get_logger(), "Error Occurred: Referee port re-open failed.");
                        }
                    }

                    std_msgs::msg::Bool status_msg;
                    if (nomessage_times >= 3) {
                        status_msg.data = false;
                        RCLCPP_WARN(this->get_logger(), "Referee link seems offline!");
                        // 尝试重启串口
                        if(referee_uart_->reopenPort() && referee_uart_->startRead()){
                            RCLCPP_INFO(this->get_logger(), "Referee port re-opened and reading started successfully.");
                        }
                        else{
                            nomessage_times=3;
                            RCLCPP_ERROR(this->get_logger(), "Referee port re-open failed.");
                        }
                    } else {
                        status_msg.data = true;
                    }
                    referee_status_pub_->publish(status_msg);
                }
            });
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when create referee check thread: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Link check threads initialized.");

    // 启动串口读取
    if(image_uart_){
        if(image_uart_->startRead()){
            RCLCPP_INFO(this->get_logger(), "Image port reading started successfully.");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to start reading from image port.");
        }
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Image port is disabled (not initialized).");
    }

    if(referee_uart_){
        if(referee_uart_->startRead()){
            RCLCPP_INFO(this->get_logger(), "Referee port reading started successfully.");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to start reading from referee port.");
        }
    }
    else{
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

RMManagerNode::~RMManagerNode() {
    if(image_check_thread_ && image_check_thread_->joinable()){
        image_check_thread_->join();
    }
    if(referee_check_thread_ && referee_check_thread_->joinable()){
        referee_check_thread_->join();
    }
}

void RMManagerNode::_read_callback(const std::vector<uint8_t>& data, std::atomic<bool>& link_status){

    // 当前处理的待处理的数据起始位置
    std::size_t start_ptr = 0;

    // debugger 发布原始数据
    rm_message::msg::GeneralMessage debug_msg;
    debug_msg.cmd_id = 0xFFFF; // 特殊cmd_id表示原始数据
    debug_msg.data_length = data.size();
    debug_msg.data_payload = data;

    debugger_pub_->publish(debug_msg);

    while(data.size() > start_ptr){

        // 检查前两位是不是 0xA9 0x53
        // 检查有没有帧头
        if(data.size() - start_ptr < PACKAGE_MIN_LENGTH){
            return;
        }

        // 处理图传的特殊消息
        if(data[start_ptr] == IMAGE_OWN_MESSAGE_HEADER[0] && data[start_ptr + 1] == IMAGE_OWN_MESSAGE_HEADER[1]){

            // 检查帧长度
            if(data.size() - start_ptr < IMAGE_OWN_MESSAGE_LENGTH){
                RCLCPP_WARN(this->get_logger(), "Image own message length mismatch! Expected: %d, Actual: %zu", IMAGE_OWN_MESSAGE_LENGTH, data.size() - start_ptr);
                return;
            }
            
            if(_process_image_own_message(std::vector<uint8_t>(data.begin() + start_ptr, data.begin() + start_ptr + IMAGE_OWN_MESSAGE_LENGTH))){
                link_status = true;
            }
            start_ptr += IMAGE_OWN_MESSAGE_LENGTH;
            continue;
        }

        if(data[start_ptr] != GENERAL_MESSAGE_HEADER){
            return;
        }

        FrameHeader header;
        memcpy(&header, data.data() + start_ptr, sizeof(FrameHeader));

        // 检查 帧头 crc8
        if(Get_CRC8_Check_Sum((uint8_t*)&header, sizeof(FrameHeader)-1) != header.crc8){
            RCLCPP_WARN(this->get_logger(), "CRC8 check failed!");
            return;
        }

        // 取出数据长度并检查
        uint16_t data_length = header.data_length;
        // 计算整帧长度
        uint16_t work_load = data_length + sizeof(FrameHeader) + 4;
        if(work_load > data.size() - start_ptr){
            RCLCPP_WARN(this->get_logger(), "Data length mismatch! Expected Bigger than: %d, Actual: %zu", work_load, data.size() - start_ptr);
            return;
        }

        uint16_t command_id = *(uint16_t*)(data.data() + start_ptr + sizeof(FrameHeader));

        // 取出数据部分
        std::vector<uint8_t> payload(data.begin() + start_ptr + sizeof(FrameHeader) + 2, data.begin() + start_ptr + sizeof(FrameHeader) + 2 + data_length);

        // 取出crc16
        uint16_t received_crc = *(uint16_t*)(data.data() + start_ptr + sizeof(FrameHeader) + 2 + data_length);
        if(Get_CRC16_Check_Sum(data.data() + start_ptr, work_load-2 ) != received_crc){
            RCLCPP_WARN(this->get_logger(), "CRC16 check failed!"); 
            start_ptr += work_load;
            continue;
        }

        // 使用 MsgPublisher 发布对应的消息
        msg_publisher_->publish(command_id, payload);
        link_status = true;

        start_ptr += work_load;
    }
}

bool RMManagerNode::_process_image_own_message(const std::vector<uint8_t>& data){

    RemoteControlData msg = {};
    memcpy(&msg, data.data(), sizeof(RemoteControlData));

    uint16_t crc = msg.crc;
    msg.crc = 0;

    // 校验CRC
    if(Get_CRC16_Check_Sum((uint8_t*)&msg, sizeof(RemoteControlData)-2) != crc){
        RCLCPP_WARN(this->get_logger(), "Image own message CRC16 check failed!");
        return false;
    }

    rm_message::msg::RemoteControl rc_msg=_remote_control_data_to_msg(msg);
    remoto_control_pub_->publish(rc_msg);
    return true;

}

rm_message::msg::RemoteControl _remote_control_data_to_msg(const RemoteControlData& data){
    rm_message::msg::RemoteControl msg;
    msg.chanal0 = data.chanal0;
    msg.chanal1 = data.chanal1;
    msg.chanal2 = data.chanal2;
    msg.chanal3 = data.chanal3;
    msg.cut = data.cut;
    msg.stop = data.stop;
    msg.keyl = data.keyl;
    msg.keyr = data.keyr;
    msg.wheel = data.wheel;
    msg.keyb = data.keyb;
    msg.mousex = data.mousex;
    msg.mousey = data.mousey;
    msg.mousez = data.mousez;
    msg.pressl = data.pressl;
    msg.pressr = data.pressr;
    msg.pressmid = data.pressmid;
    msg.w = data.keyboards.w;
    msg.s = data.keyboards.s;
    msg.a = data.keyboards.a;
    msg.d = data.keyboards.d;
    msg.shift = data.keyboards.shift;
    msg.ctrl = data.keyboards.ctrl;
    msg.q = data.keyboards.q;
    msg.e = data.keyboards.e;
    msg.r = data.keyboards.r;
    msg.f = data.keyboards.f;
    msg.g = data.keyboards.g;
    msg.z = data.keyboards.z;
    msg.x = data.keyboards.x;
    msg.c = data.keyboards.c;
    msg.v = data.keyboards.v;
    msg.b = data.keyboards.b;

    return msg;
}

void RMManagerNode::_send_sub_callback(const rm_message::msg::SendMessage::SharedPtr msg){

    // 处理帧头
    std::vector<uint8_t> frame;
    FrameHeader header = {};
    header.sof = 0xA5;
    header.data_length = msg->data_length;
    header.seq = 0;
    header.crc8 = Get_CRC8_Check_Sum((uint8_t*)&header, sizeof(FrameHeader)-1);

    frame.insert(frame.end(), (uint8_t*)&header, (uint8_t*)&header + sizeof(FrameHeader));

    // 处理命令字
    frame.push_back(msg->cmd_id & 0xFF);
    frame.push_back((msg->cmd_id >> 8) & 0xFF);

    // 处理数据部分
    frame.insert(frame.end(), msg->data_payload.begin(), msg->data_payload.end());

    // 处理crc16
    uint16_t crc16 = Get_CRC16_Check_Sum(msg->data_payload.data(), msg->data_payload.size());
    frame.push_back(crc16 & 0xFF);
    frame.push_back((crc16 >> 8) & 0xFF);

    // 发送数据
    if(msg->target == 1 && image_uart_ && image_uart_->isPortOK()){
        image_uart_->writeData(frame);
    }
    else if(msg->target == 2 && referee_uart_ && referee_uart_->isPortOK()){
        referee_uart_->writeData(frame);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "SendMessage target error or port not open! target: %d", msg->target);
    }

}

} // namespace RMManager