#ifndef MSG_PUBLISHER_HPP
#define MSG_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/ammo_allowance.hpp"
#include "rm_message/msg/buffer_and_heat.hpp"
#include "rm_message/msg/client_custom_command.hpp"
#include "rm_message/msg/custom_controller.hpp"
#include "rm_message/msg/dart_cmd.hpp"
#include "rm_message/msg/dart_info.hpp"
#include "rm_message/msg/field_events.hpp"
#include "rm_message/msg/game_result.hpp"
#include "rm_message/msg/game_state.hpp"
#include "rm_message/msg/general_message.hpp"
#include "rm_message/msg/ground_positions.hpp"
#include "rm_message/msg/hurt_event.hpp"
#include "rm_message/msg/map_downlink.hpp"
#include "rm_message/msg/query_vtm_channel.hpp"
#include "rm_message/msg/radar_decision.hpp"
#include "rm_message/msg/radar_mark.hpp"
#include "rm_message/msg/referee_warning.hpp"
#include "rm_message/msg/rfid_status.hpp"
#include "rm_message/msg/robot_buffs.hpp"
#include "rm_message/msg/robot_hp.hpp"
#include "rm_message/msg/robot_interaction.hpp"
#include "rm_message/msg/robot_position.hpp"
#include "rm_message/msg/robot_status.hpp"
#include "rm_message/msg/sentry_decision.hpp"
#include "rm_message/msg/set_vtm_channel.hpp"
#include "rm_message/msg/shoot_data.hpp"

#include <cstring>
#include <vector>

namespace RMManager
{

enum class LinkType
{
    Image,
    Referee,
};

// 0x0001 - Game Status (11 bytes)
struct GameStateData
{
    uint8_t game_type : 4;
    uint8_t game_stage : 4;
    uint16_t rest_time;
    uint64_t unix_time;
} __attribute__((packed));

// 0x0002 - Game Result (1 byte)
struct GameResultData
{
    uint8_t result;
} __attribute__((packed));

// 0x0003 - Robot HP (16 bytes)
struct RobotHPData
{
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
struct FieldEventsData
{
    uint32_t supply_zone1 : 1;
    uint32_t supply_zone2 : 1;
    uint32_t supply_zone3 : 1;
    uint32_t small_energy_status : 2;
    uint32_t large_energy_status : 2;
    uint32_t central_highland : 2;
    uint32_t trapezoid_highland : 2;
    uint32_t dart_last_hit_time : 9;
    uint32_t dart_target : 3;
    uint32_t central_buff : 2;
    uint32_t fortress_buff : 2;
    uint32_t outpost_buff : 2;
    uint32_t base_buff : 1;
    uint32_t reserved : 2;
} __attribute__((packed));

// 0x0104 - Referee Warning (3 bytes)
struct RefereeWarningData
{
    uint8_t penalty_level;
    uint8_t robot_id;
    uint8_t violation_count;
} __attribute__((packed));

// 0x0105 - Dart Info (3 bytes)
struct DartInfoData
{
    uint8_t dart_remaining_time;
    uint8_t target_type : 3;
    uint8_t hit_count : 3;
    uint8_t selected_target : 2;
    uint8_t reserved;
} __attribute__((packed));

// 0x0201 - Robot Status (13 bytes)
struct RobotStatusData
{
    uint8_t robot_id;
    uint8_t level;
    uint16_t current_hp;
    uint16_t max_hp;
    uint16_t heat_cooling_rate;
    uint16_t heat_max;
    uint16_t chassis_power_limit;
    uint8_t gimbal_power : 1;
    uint8_t chassis_power : 1;
    uint8_t shooter_power : 1;
    uint8_t reserved : 5;
} __attribute__((packed));

// 0x0202 - Buffer And Heat (14 bytes)
struct BufferAndHeatData
{
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;
    uint16_t heat_17mm;
    uint16_t heat_42mm;
} __attribute__((packed));

// 0x0203 - Robot Position (12 bytes)
struct RobotPositionData
{
    float x;
    float y;
    float yaw;
} __attribute__((packed));

// 0x0204 - Robot Buffs (8 bytes)
struct RobotBuffsData
{
    uint8_t hp_buff;
    uint16_t cooling_buff;
    uint8_t defense_buff;
    uint8_t negative_defense_buff;
    uint16_t attack_buff;
    uint8_t energy_flags;
} __attribute__((packed));

// 0x0206 - Hurt Event (1 byte)
struct HurtEventData
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} __attribute__((packed));

// 0x0207 - Shoot Data (7 bytes)
struct ShootDataData
{
    uint8_t ammo_type_17mm : 1;
    uint8_t ammo_type_42mm : 1;
    uint8_t reserved : 6;
    uint8_t shoot_mechanism;
    uint8_t shoot_speed;
    float initial_velocity;
} __attribute__((packed));

// 0x0208 - Ammo Allowance (8 bytes)
struct AmmoAllowanceData
{
    uint16_t ammo_17mm;
    uint16_t ammo_42mm;
    uint16_t coin_count;
    uint16_t projectile_allowance_fortress;
} __attribute__((packed));

// 0x0209 - RFID Status (5 bytes)
struct RFIDStatusData
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} __attribute__((packed));

// 0x020A - Dart Cmd (6 bytes)
struct DartCmdData
{
    uint8_t dart_station_status;
    uint8_t reserved;
    uint16_t target_switch_countdown;
    uint16_t last_shoot_countdown;
} __attribute__((packed));

// 0x020B - Ground Positions (40 bytes)
struct GroundPositionsData
{
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
struct RadarMarkData
{
    uint16_t radar_mark_status;
} __attribute__((packed));

// 0x020D - Sentry Decision (6 bytes)
struct SentryDecisionData
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} __attribute__((packed));

// 0x020E - Radar Decision (1 byte)
struct RadarDecisionData
{
    uint8_t radar_info;
} __attribute__((packed));

// 0x0301 - Robot Interaction (variable)
struct RobotInteractionData
{
    uint16_t content_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} __attribute__((packed));

// 0x0302 - Custom Controller (30 bytes)
struct CustomControllerData
{
    uint8_t data[30];
} __attribute__((packed));

// 0x0303 - Map Downlink (12 bytes)
struct MapDownlinkData
{
    float target_x;
    float target_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t source_id;
} __attribute__((packed));

// 0x0311 - Client Custom Command (30 bytes)
struct ClientCustomCommandData
{
    uint8_t data[30];
} __attribute__((packed));

// 0x0F01 - Set VTM Channel (1 byte)
struct SetVTMChannelData
{
    uint8_t channel;
} __attribute__((packed));

// 0x0F02 - Query VTM Channel (1 byte)
struct QueryVTMChannelData
{
    uint8_t channel;
} __attribute__((packed));

class MsgPublisher {
public:
    explicit MsgPublisher(rclcpp::Node * node);

    void publish(LinkType link_type, uint16_t cmd_id, const std::vector<uint8_t> & payload);

private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

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
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::ClientCustomCommand>> pub_0x0311_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::SetVTMChannel>> pub_0x0F01_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::QueryVTMChannel>> pub_0x0F02_;

    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>> pub_general_;
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>> pub_all_messages_;

    void _parse_and_publish_0x0001(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0002(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0003(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0101(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0104(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0105(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0201(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0202(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0203(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0204(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0206(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0207(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0208(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0209(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x020A(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x020B(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x020C(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x020D(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x020E(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0301(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0302(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0303(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0311(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0F01(const std::vector<uint8_t> & payload);
    void _parse_and_publish_0x0F02(const std::vector<uint8_t> & payload);

    void _publish_general_message(uint16_t cmd_id, const std::vector<uint8_t> & payload);
    void _publish_to_all_messages(uint16_t cmd_id, const std::vector<uint8_t> & payload);
    bool _validate_payload_size(size_t expected_size, size_t actual_size, uint16_t cmd_id);
    bool _is_known_command(uint16_t cmd_id) const;
    bool _is_typed_command_supported_on_link(LinkType link_type, uint16_t cmd_id) const;

    template<typename T>
    bool load_struct(uint16_t cmd_id, const std::vector<uint8_t> & payload, T & out);
};

}  // namespace RMManager

template<typename T>
bool RMManager::MsgPublisher::load_struct(
    uint16_t cmd_id, const std::vector<uint8_t> & payload,
    T & out)
{
    if (!_validate_payload_size(sizeof(T), payload.size(), cmd_id)) {
        return false;
    }
    std::memcpy(&out, payload.data(), sizeof(T));
    return true;
}

#endif  // MSG_PUBLISHER_HPP
