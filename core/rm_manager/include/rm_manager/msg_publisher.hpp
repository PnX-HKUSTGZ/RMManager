#ifndef MSG_PUBLISHER_HPP
#define MSG_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/general_message.hpp"
#include "rm_message/msg/game_state.hpp"
#include "rm_message/msg/game_result.hpp"
#include "rm_message/msg/robot_hp.hpp"
#include "rm_message/msg/field_events.hpp"
#include "rm_message/msg/referee_warning.hpp"
#include "rm_message/msg/dart_info.hpp"
#include "rm_message/msg/robot_status.hpp"
#include "rm_message/msg/buffer_and_heat.hpp"
#include "rm_message/msg/robot_position.hpp"
#include "rm_message/msg/robot_buffs.hpp"
#include "rm_message/msg/hurt_event.hpp"
#include "rm_message/msg/shoot_data.hpp"
#include "rm_message/msg/ammo_allowance.hpp"
#include "rm_message/msg/rfid_status.hpp"
#include "rm_message/msg/dart_cmd.hpp"
#include "rm_message/msg/ground_positions.hpp"
#include "rm_message/msg/radar_mark.hpp"
#include "rm_message/msg/sentry_decision.hpp"
#include "rm_message/msg/radar_decision.hpp"
#include "rm_message/msg/ref_remote_control.hpp"
#include "rm_message/msg/robot_interaction.hpp"
#include "rm_message/msg/custom_controller.hpp"
#include "rm_message/msg/map_downlink.hpp"
#include "rm_message/msg/radar_map_uplink.hpp"
#include "rm_message/msg/key_mouse_simulation.hpp"
#include "rm_message/msg/path_uplink.hpp"
#include "rm_message/msg/custom_string.hpp"
#include "rm_message/msg/robot_custom_data.hpp"
#include "rm_message/msg/robot_custom_data_large.hpp"
#include "rm_message/msg/set_vtm_channel.hpp"
#include "rm_message/msg/query_vtm_channel.hpp"

#include <cstring>
#include <map>

namespace RMManager {

// ============================================================
// 紧凑消息结构体定义 (使用 __attribute__((packed)))
// ============================================================

// 0x0001 - Game Status (11 bytes)
struct GameStateData {
    uint8_t game_type : 4;           // bit 0-3
    uint8_t game_stage : 4;          // bit 4-7
    uint16_t rest_time;
    uint64_t unix_time;
} __attribute__((packed));

// 0x0002 - Game Result (1 byte)
struct GameResultData {
    uint8_t result;
} __attribute__((packed));

// 0x0003 - Robot HP (16 bytes)
struct RobotHPData {
    uint16_t hp_hero;
    uint16_t hp_engineer;
    uint16_t hp_infantry3;
    uint16_t hp_infantry4;
    uint16_t reserved;
    uint16_t hp_sentry;
    uint16_t hp_outpost;
    uint16_t hp_base;
} __attribute__((packed));

// 0x0101 - Field Events (4 bytes)
struct FieldEventsData {
    uint32_t supply_zone1 : 1;           // bit 0
    uint32_t supply_zone2 : 1;           // bit 1
    uint32_t supply_zone3 : 1;           // bit 2
    uint32_t small_energy_status : 2;    // bit 3-4
    uint32_t large_energy_status : 2;    // bit 5-6
    uint32_t central_highland : 2;       // bit 7-8
    uint32_t trapezoid_highland : 2;     // bit 9-10
    uint32_t dart_last_hit_time : 9;     // bit 11-19
    uint32_t dart_target : 3;            // bit 20-22
    uint32_t central_buff : 2;           // bit 23-24
    uint32_t fortress_buff : 2;          // bit 25-26
    uint32_t outpost_buff : 2;           // bit 27-28
    uint32_t base_buff : 1;              // bit 29
    uint32_t reserved : 2;               // bit 30-31
} __attribute__((packed));

// 0x0104 - Referee Warning (3 bytes)
struct RefereeWarningData {
    uint8_t penalty_level;
    uint8_t robot_id;
    uint8_t violation_count;
} __attribute__((packed));

// 0x0105 - Dart Info (3 bytes)
struct DartInfoData {
    uint8_t dart_remaining_time;
    uint8_t target_type : 3;         // bit 0-2
    uint8_t hit_count : 3;           // bit 3-5
    uint8_t selected_target : 2;     // bit 6-7
    uint8_t reserved;
} __attribute__((packed));

// 0x0201 - Robot Status (13 bytes)
struct RobotStatusData {
    uint8_t robot_id;
    uint8_t level;
    uint16_t current_hp;
    uint16_t max_hp;
    uint16_t heat_cooling_rate;
    uint16_t heat_max;
    uint16_t chassis_power_limit;
    uint8_t gimbal_power : 1;        // bit 0
    uint8_t chassis_power : 1;       // bit 1
    uint8_t shooter_power : 1;       // bit 2
    uint8_t reserved : 5;            // bit 3-7
} __attribute__((packed));

// 0x0202 - Buffer And Heat (12 bytes)
struct BufferAndHeatData {
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;
    uint16_t heat_17mm;
    uint16_t heat_42mm;
} __attribute__((packed));

// 0x0203 - Robot Position (16 bytes)
struct RobotPositionData {
    float x;
    float y;
    float yaw;
} __attribute__((packed));

// 0x0204 - Robot Buffs (8 bytes)
struct RobotBuffsData {
    uint8_t hp_buff;
    uint16_t cooling_buff;
    uint8_t defense_buff;
    uint8_t negative_defense_buff;
    uint16_t attack_buff;
    uint8_t energy_flags; // bit0-5 energy thresholds, bit6 reserved, bit7 reserved
} __attribute__((packed));

// 0x0206 - Hurt Event (1 byte)
struct HurtEventData {
    uint8_t armor_id : 4;            // bit 0-3
    uint8_t hurt_type : 4;           // bit 4-7
} __attribute__((packed));

// 0x0207 - Shoot Data (7 bytes)
struct ShootDataData {
    uint8_t ammo_type_17mm : 1;      // bit 1
    uint8_t ammo_type_42mm : 1;      // bit 2
    uint8_t reserved : 6;            // bit 0, 3-7
    uint8_t shoot_mechanism;
    uint8_t shoot_speed;
    float initial_velocity;
} __attribute__((packed));

// 0x0208 - Ammo Allowance (6 bytes)
struct AmmoAllowanceData {
    uint16_t ammo_17mm;
    uint16_t ammo_42mm;
    uint16_t coin_count;
    uint16_t projectile_allowance_fortress;
} __attribute__((packed));

// 0x0209 - RFID Status (5 bytes)
struct RFIDStatusData {
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} __attribute__((packed));

// 0x020A - Dart Cmd (6 bytes)
struct DartCmdData {
    uint8_t dart_station_status;
    uint8_t reserved;
    uint16_t target_switch_countdown;
    uint16_t last_shoot_countdown;
} __attribute__((packed));

// 0x020B - Ground Positions (40 bytes)
struct GroundPositionsData {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float infantry3_x;
    float infantry3_y;
    float infantry4_x;
    float infantry4_y;
    float reserved1;
    float reserved2;
} __attribute__((packed));

// 0x020C - Radar Mark (2 bytes)
struct RadarMarkData {
    uint16_t radar_mark_status;
} __attribute__((packed));

// 0x020D - Sentry Decision (6 bytes)
struct SentryDecisionData {
    uint32_t sentry_info;      // 4字节
    uint16_t sentry_info_2;    // 2字节
} __attribute__((packed));

// 0x020E - Radar Decision (1 byte)
struct RadarDecisionData {
    uint8_t radar_info;
} __attribute__((packed));

// 0x0301 - Robot Interaction (可变长度)
struct RobotInteractionData {
    uint16_t content_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    // 后续是可变长度数据
} __attribute__((packed));

// 0x0302 - Custom Controller (30 bytes)
struct CustomControllerData {
    uint8_t data[30];
} __attribute__((packed));

// 0x0303 - Map Downlink (15 bytes)
struct MapDownlinkData {
    float target_x;
    float target_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t source_id;
} __attribute__((packed));

// 0x0304 - Ref Remote Control (12 bytes)
struct RefRemoteControlData {
    int16_t mouse_vx;
    int16_t mouse_vy;
    int16_t wheel_speed;
    uint8_t left_button;
    uint8_t right_button;
    uint8_t w : 1;              // bit 0 // wsad shift ctrl q e r f g z x c v b
    uint8_t s : 1;              // bit 1
    uint8_t a : 1;              // bit 2
    uint8_t d : 1;              // bit 3
    uint8_t shift : 1;          // bit 4
    uint8_t ctrl : 1;           // bit 5
    uint8_t q : 1;              // bit 6
    uint8_t e : 1;              // bit 7
    uint8_t r : 1;              // bit 0 of second byte
    uint8_t f : 1;              // bit 1
    uint8_t g : 1;              // bit 2
    uint8_t z : 1;              // bit 3
    uint8_t x : 1;              // bit 4
    uint8_t c : 1;              // bit 5
    uint8_t v : 1;              // bit 6
    uint8_t b : 1;              // bit 7 of second byte
    uint16_t reserved;
} __attribute__((packed));

// 0x0305 - Radar Map Uplink (24 bytes)
struct RadarMapUplinkData {
    uint16_t hero_x;
    uint16_t hero_y;
    uint16_t engineer_x;
    uint16_t engineer_y;
    uint16_t infantry3_x;
    uint16_t infantry3_y;
    uint16_t infantry4_x;
    uint16_t infantry4_y;
    uint16_t infantry5_x;
    uint16_t infantry5_y;
    uint16_t sentry_x;
    uint16_t sentry_y;
} __attribute__((packed));

// 0x0306 - Key Mouse Simulation (8 bytes)
struct KeyMouseSimulationData {
    uint8_t key1;
    uint8_t key2;
    uint16_t mouse_x;
    uint8_t left_button;
    uint16_t mouse_y;
    uint8_t right_button;
} __attribute__((packed));

// 0x0307 - Path Uplink (103 bytes)
struct PathUplinkData {
    uint8_t intention;
    uint16_t start_x;
    uint16_t start_y;
    uint8_t delta_x[49];
    uint8_t delta_y[49];
    uint16_t sender_id;
} __attribute__((packed));

// 0x0308 - Custom String (34 bytes)
struct CustomStringData {
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t content[30];
} __attribute__((packed));

// 0x0309 - Robot Custom Data (30 bytes)
struct RobotCustomDataData {
    uint8_t data[30];
} __attribute__((packed));

// 0x0310 - Robot Custom Data Large (150 bytes)
struct RobotCustomDataLargeData {
    uint8_t data[150];
} __attribute__((packed));

// 0x0F01 - Set VTM Channel (1 byte)
struct SetVTMChannelData {
    uint8_t channel;
} __attribute__((packed));

// 0x0F02 - Query VTM Channel (1 byte)
struct QueryVTMChannelData {
    uint8_t channel;
} __attribute__((packed));

// ============================================================
// Topic 名称映射表
// ============================================================
static const std::map<uint16_t, std::string> TOPIC_NAME_MAP = {
    {0x0001, "game_status"},
    {0x0002, "game_result"},
    {0x0003, "robot_hp"},
    {0x0101, "field_events"},
    {0x0104, "referee_warning"},
    {0x0105, "dart_info"},
    {0x0201, "robot_status"},
    {0x0202, "buffer_heat"},
    {0x0203, "robot_position"},
    {0x0204, "robot_buffs"},
    {0x0206, "hurt_event"},
    {0x0207, "shoot_data"},
    {0x0208, "ammo_allowance"},
    {0x0209, "rfid_status"},
    {0x020A, "dart_cmd"},
    {0x020B, "ground_positions"},
    {0x020C, "radar_mark"},
    {0x020D, "sentry_decision"},
    {0x020E, "radar_decision"},
    {0x0301, "robot_interaction"},
    {0x0302, "custom_controller"},
    {0x0303, "map_downlink"},
    {0x0304, "ref_remote_control"},
    {0x0305, "radar_map_uplink"},
    {0x0306, "key_mouse_simulation"},
    {0x0307, "path_uplink"},
    {0x0308, "custom_string"},
    {0x0309, "robot_custom_data"},
    {0x0310, "robot_custom_data_large"},
    {0x0F01, "set_vtm_channel"},
    {0x0F02, "query_vtm_channel"},
};

// ============================================================
// MsgPublisher 类 - 消息发布管理器
// ============================================================
class MsgPublisher {
public:
    /**
     * @brief 构造函数 - 初始化所有Publisher
     * @param node ROS2节点的原始指针
     */
    MsgPublisher(rclcpp::Node* node);

    /**
     * @brief 核心发布函数 - 根据cmd_id自动路由到对应publisher
     * @param cmd_id 命令ID
     * @param payload 有效载荷数据
     */
    void publish(uint16_t cmd_id, const std::vector<uint8_t>& payload);

private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    // ============================================================
    // 特定消息类型的Publishers
    // ============================================================
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GameState>> pub_0x0001_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GameResult>> pub_0x0002_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RobotHP>> pub_0x0003_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::FieldEvents>> pub_0x0101_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RefereeWarning>> pub_0x0104_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::DartInfo>> pub_0x0105_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RobotStatus>> pub_0x0201_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::BufferAndHeat>> pub_0x0202_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RobotPosition>> pub_0x0203_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RobotBuffs>> pub_0x0204_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::HurtEvent>> pub_0x0206_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::ShootData>> pub_0x0207_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::AmmoAllowance>> pub_0x0208_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RFIDStatus>> pub_0x0209_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::DartCmd>> pub_0x020A_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GroundPositions>> pub_0x020B_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RadarMark>> pub_0x020C_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::SentryDecision>> pub_0x020D_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RadarDecision>> pub_0x020E_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RobotInteraction>> pub_0x0301_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::CustomController>> pub_0x0302_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::MapDownlink>> pub_0x0303_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RefRemoteControl>> pub_0x0304_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RadarMapUplink>> pub_0x0305_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::KeyMouseSimulation>> pub_0x0306_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::PathUplink>> pub_0x0307_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::CustomString>> pub_0x0308_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RobotCustomData>> pub_0x0309_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RobotCustomDataLarge>> pub_0x0310_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::SetVTMChannel>> pub_0x0F01_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::QueryVTMChannel>> pub_0x0F02_;

    // 通用消息publisher - 用于可变长消息和未知消息类型
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>> pub_general_;
    
    // 所有消息的通用publisher - 无论如何都发出GeneralMessage形式的消息
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>> pub_all_messages_;

    // ============================================================
    // Parser 函数 - 解析二进制数据并发布到对应topic
    // ============================================================
    void _parse_and_publish_0x0001(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0002(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0003(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0101(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0104(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0105(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0201(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0202(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0203(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0204(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0206(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0207(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0208(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0209(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x020A(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x020B(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x020C(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x020D(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x020E(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0301(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0302(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0303(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0304(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0305(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0306(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0307(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0308(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0309(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0310(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0F01(const std::vector<uint8_t>& payload);
    void _parse_and_publish_0x0F02(const std::vector<uint8_t>& payload);

    /**
     * @brief 发布通用消息 - 用于可变长或未知消息类型
     * @param cmd_id 命令ID
     * @param payload 有效载荷
     */
    void _publish_general_message(uint16_t cmd_id, const std::vector<uint8_t>& payload);

    /**
     * @brief 发送所有消息为GeneralMessage格式
     * @param cmd_id 命令ID
     * @param payload 有效载荷
     */
    void _publish_to_all_messages(uint16_t cmd_id, const std::vector<uint8_t>& payload);

    /**
     * @brief 验证payload大小是否符合预期
     * @param expected_size 预期大小
     * @param actual_size 实际大小
     * @param cmd_id 命令ID（用于日志）
     * @return true 如果大小符合
     */
    bool _validate_payload_size(size_t expected_size, size_t actual_size, uint16_t cmd_id);

    template<typename T>
    bool load_struct(uint16_t cmd_id, const std::vector<uint8_t>& payload, T& out);
};

} // namespace RMManager

template<typename T>
bool RMManager::MsgPublisher::load_struct(uint16_t cmd_id, const std::vector<uint8_t>& payload, T& out) {
    if (!_validate_payload_size(sizeof(T), payload.size(), cmd_id)) return false;
    std::memcpy(&out, payload.data(), sizeof(T));
    return true;
}

#endif // MSG_PUBLISHER_HPP
