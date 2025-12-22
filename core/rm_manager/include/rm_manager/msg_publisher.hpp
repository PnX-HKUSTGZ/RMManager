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
    uint8_t attack_buff;
    uint8_t energy_125 : 1;          // bit 0
    uint8_t energy_100 : 1;          // bit 1
    uint8_t energy_50 : 1;           // bit 2
    uint8_t energy_30 : 1;           // bit 3
    uint8_t energy_15 : 1;           // bit 4
    uint8_t energy_5 : 1;            // bit 5
    uint8_t energy_1 : 1;            // bit 6
    uint8_t reserved : 1;            // bit 7
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
} __attribute__((packed));

// 0x0209 - RFID Status (4 bytes)
struct RFIDStatusData {
    uint32_t rfid_status;
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
    float infantry5_x;
    float infantry5_y;
} __attribute__((packed));

// 0x020C - Radar Mark (2 bytes)
struct RadarMarkData {
    uint16_t radar_mark_status;
} __attribute__((packed));

// 0x020D - Sentry Decision (5 bytes)
struct SentryDecisionData {
    uint16_t allow_bullet_count : 11;  // bit 0-10
    uint8_t exchange_count : 4;        // bit 11-14
    uint8_t free_revive_available : 1; // bit 19
    uint8_t immediate_revive_available : 1; // bit 20
    uint8_t sentry_attitude : 2;       // bit 12-13
    uint8_t energy_gear_available : 1; // bit 14
    uint8_t reserved : 3;              // bit 15-17
} __attribute__((packed));

// 0x020E - Radar Decision (1 byte)
struct RadarDecisionData {
    uint8_t double_vulnerable_count : 2; // bit 0-1
    uint8_t enemy_vulnerable : 1;        // bit 2
    uint8_t encryption_level : 2;        // bit 3-4
    uint8_t reserved : 3;                // bit 5-7
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
    {0x0304, "ref_remote_control"},
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
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RefRemoteControl>> pub_0x0304_;

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
    void _parse_and_publish_0x0304(const std::vector<uint8_t>& payload);

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
};

} // namespace RMManager

#endif // MSG_PUBLISHER_HPP
