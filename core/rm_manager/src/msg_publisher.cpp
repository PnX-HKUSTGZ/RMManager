#include "rm_manager/msg_publisher.hpp"
#include <algorithm>

namespace RMManager {

MsgPublisher::MsgPublisher(rclcpp::Node* node) : logger_(node->get_logger()), clock_(node->get_clock()) {
    std::string node_name = std::string(node->get_name()) + "/";

    // 初始化所有特定消息类型的Publishers
    pub_0x0001_ = node->create_publisher<rm_message::msg::GameState>(node_name + "game_status", 10);
    pub_0x0002_ = node->create_publisher<rm_message::msg::GameResult>(node_name + "game_result", 10);
    pub_0x0003_ = node->create_publisher<rm_message::msg::RobotHP>(node_name + "robot_hp", 10);
    pub_0x0101_ = node->create_publisher<rm_message::msg::FieldEvents>(node_name + "field_events", 10);
    pub_0x0104_ = node->create_publisher<rm_message::msg::RefereeWarning>(node_name + "referee_warning", 10);
    pub_0x0105_ = node->create_publisher<rm_message::msg::DartInfo>(node_name + "dart_info", 10);
    pub_0x0201_ = node->create_publisher<rm_message::msg::RobotStatus>(node_name + "robot_status", 10);
    pub_0x0202_ = node->create_publisher<rm_message::msg::BufferAndHeat>(node_name + "buffer_heat", 10);
    pub_0x0203_ = node->create_publisher<rm_message::msg::RobotPosition>(node_name + "robot_position", 10);
    pub_0x0204_ = node->create_publisher<rm_message::msg::RobotBuffs>(node_name + "robot_buffs", 10);
    pub_0x0206_ = node->create_publisher<rm_message::msg::HurtEvent>(node_name + "hurt_event", 10);
    pub_0x0207_ = node->create_publisher<rm_message::msg::ShootData>(node_name + "shoot_data", 10);
    pub_0x0208_ = node->create_publisher<rm_message::msg::AmmoAllowance>(node_name + "ammo_allowance", 10);
    pub_0x0209_ = node->create_publisher<rm_message::msg::RFIDStatus>(node_name + "rfid_status", 10);
    pub_0x020A_ = node->create_publisher<rm_message::msg::DartCmd>(node_name + "dart_cmd", 10);
    pub_0x020B_ = node->create_publisher<rm_message::msg::GroundPositions>(node_name + "ground_positions", 10);
    pub_0x020C_ = node->create_publisher<rm_message::msg::RadarMark>(node_name + "radar_mark", 10);
    pub_0x020D_ = node->create_publisher<rm_message::msg::SentryDecision>(node_name + "sentry_decision", 10);
    pub_0x020E_ = node->create_publisher<rm_message::msg::RadarDecision>(node_name + "radar_decision", 10);
    pub_0x0301_ = node->create_publisher<rm_message::msg::RobotInteraction>(node_name + "robot_interaction", 10);
    pub_0x0302_ = node->create_publisher<rm_message::msg::CustomController>(node_name + "custom_controller", 10);
    pub_0x0303_ = node->create_publisher<rm_message::msg::MapDownlink>(node_name + "map_downlink", 10);
    pub_0x0304_ = node->create_publisher<rm_message::msg::RefRemoteControl>(node_name + "ref_remote_control", 10);
    pub_0x0305_ = node->create_publisher<rm_message::msg::RadarMapUplink>(node_name + "radar_map_uplink", 10);
    pub_0x0306_ = node->create_publisher<rm_message::msg::KeyMouseSimulation>(node_name + "key_mouse_simulation", 10);
    pub_0x0307_ = node->create_publisher<rm_message::msg::PathUplink>(node_name + "path_uplink", 10);
    pub_0x0308_ = node->create_publisher<rm_message::msg::CustomString>(node_name + "custom_string", 10);
    pub_0x0309_ = node->create_publisher<rm_message::msg::RobotCustomData>(node_name + "robot_custom_data", 10);
    pub_0x0310_ = node->create_publisher<rm_message::msg::RobotCustomDataLarge>(node_name + "robot_custom_data_large", 10);
    pub_0x0F01_ = node->create_publisher<rm_message::msg::SetVTMChannel>(node_name + "set_vtm_channel", 10);
    pub_0x0F02_ = node->create_publisher<rm_message::msg::QueryVTMChannel>(node_name + "query_vtm_channel", 10);

    // 初始化通用消息publisher
    pub_general_ = node->create_publisher<rm_message::msg::GeneralMessage>(node_name + "unknown_command", 10);
    
    // 初始化所有消息的通用publisher
    pub_all_messages_ = node->create_publisher<rm_message::msg::GeneralMessage>(node_name + "all_messages", 10);

    RCLCPP_INFO(logger_, "MsgPublisher initialized with 31 specific publishers");
}

void MsgPublisher::publish(uint16_t cmd_id, const std::vector<uint8_t>& payload) {
    // 先发送通用消息格式到all_messages
    _publish_to_all_messages(cmd_id, payload);
    
    // 然后根据cmd_id发送特定类型的消息
    switch (cmd_id) {
        case 0x0001: _parse_and_publish_0x0001(payload); break;
        case 0x0002: _parse_and_publish_0x0002(payload); break;
        case 0x0003: _parse_and_publish_0x0003(payload); break;
        case 0x0101: _parse_and_publish_0x0101(payload); break;
        case 0x0104: _parse_and_publish_0x0104(payload); break;
        case 0x0105: _parse_and_publish_0x0105(payload); break;
        case 0x0201: _parse_and_publish_0x0201(payload); break;
        case 0x0202: _parse_and_publish_0x0202(payload); break;
        case 0x0203: _parse_and_publish_0x0203(payload); break;
        case 0x0204: _parse_and_publish_0x0204(payload); break;
        case 0x0206: _parse_and_publish_0x0206(payload); break;
        case 0x0207: _parse_and_publish_0x0207(payload); break;
        case 0x0208: _parse_and_publish_0x0208(payload); break;
        case 0x0209: _parse_and_publish_0x0209(payload); break;
        case 0x020A: _parse_and_publish_0x020A(payload); break;
        case 0x020B: _parse_and_publish_0x020B(payload); break;
        case 0x020C: _parse_and_publish_0x020C(payload); break;
        case 0x020D: _parse_and_publish_0x020D(payload); break;
        case 0x020E: _parse_and_publish_0x020E(payload); break;
        case 0x0301: _parse_and_publish_0x0301(payload); break;
        case 0x0302: _parse_and_publish_0x0302(payload); break;
        case 0x0303: _parse_and_publish_0x0303(payload); break;
        case 0x0304: _parse_and_publish_0x0304(payload); break;
        case 0x0305: _parse_and_publish_0x0305(payload); break;
        case 0x0306: _parse_and_publish_0x0306(payload); break;
        case 0x0307: _parse_and_publish_0x0307(payload); break;
        case 0x0308: _parse_and_publish_0x0308(payload); break;
        case 0x0309: _parse_and_publish_0x0309(payload); break;
        case 0x0310: _parse_and_publish_0x0310(payload); break;
        case 0x0F01: _parse_and_publish_0x0F01(payload); break;
        case 0x0F02: _parse_and_publish_0x0F02(payload); break;
        default:
            // 未知CMD ID - 使用通用消息
            RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 5000,
                "Received unknown cmd_id 0x" << std::hex << cmd_id << ", publishing as GeneralMessage");
            _publish_general_message(cmd_id, payload);
            break;
    }
}

bool MsgPublisher::_validate_payload_size(size_t expected_size, size_t actual_size, uint16_t cmd_id) {
    if (actual_size != expected_size) {
        RCLCPP_WARN(logger_,
            "Payload size mismatch for cmd_id 0x%04X: expected %zu bytes, got %zu bytes",
            cmd_id, expected_size, actual_size);
        return false;
    }
    return true;
}

// ============================================================
// Parser 实现 - 0x000x (比赛基础信息类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0001(const std::vector<uint8_t>& payload) {
    GameStateData data{};
    if (!load_struct(0x0001, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::GameState>();
    msg->game_type = data.game_type;
    msg->game_stage = data.game_stage;
    msg->rest_time = data.rest_time;
    msg->unix_time = data.unix_time;

    pub_0x0001_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0002(const std::vector<uint8_t>& payload) {
    GameResultData data{};
    if (!load_struct(0x0002, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::GameResult>();
    msg->result = data.result;

    pub_0x0002_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0003(const std::vector<uint8_t>& payload) {
    RobotHPData data{};
    if (!load_struct(0x0003, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RobotHP>();
    msg->hp_hero = data.hp_hero;
    msg->hp_engineer = data.hp_engineer;
    msg->hp_infantry3 = data.hp_infantry3;
    msg->hp_infantry4 = data.hp_infantry4;
    msg->reserved = data.reserved;
    msg->hp_sentry = data.hp_sentry;
    msg->hp_outpost = data.hp_outpost;
    msg->hp_base = data.hp_base;

    pub_0x0003_->publish(std::move(msg));
}

// ============================================================
// Parser 实现 - 0x010x (场地与裁判类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0101(const std::vector<uint8_t>& payload) {
    FieldEventsData data{};
    if (!load_struct(0x0101, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::FieldEvents>();
    // 直接从位字段赋值
    msg->supply_area_1 = data.supply_zone1;
    msg->supply_area_2 = data.supply_zone2;
    msg->supply_area_own = data.supply_zone3;
    msg->small_energy_status = data.small_energy_status;
    msg->large_energy_status = data.large_energy_status;
    msg->center_highland_status = data.central_highland;
    msg->trapezoid_highland_status = data.trapezoid_highland;
    msg->dart_hit_time = data.dart_last_hit_time;
    msg->dart_target_type = data.dart_target;
    msg->center_gain_status = data.central_buff;
    msg->fort_gain_status = data.fortress_buff;
    msg->outpost_gain_status = data.outpost_buff;
    msg->base_gain_status = data.base_buff;

    pub_0x0101_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0104(const std::vector<uint8_t>& payload) {
    RefereeWarningData data{};
    if (!load_struct(0x0104, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RefereeWarning>();
    msg->penalty_level = data.penalty_level;
    msg->robot_id = data.robot_id;
    msg->violation_count = data.violation_count;

    pub_0x0104_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0105(const std::vector<uint8_t>& payload) {
    DartInfoData data{};
    if (!load_struct(0x0105, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::DartInfo>();
    msg->dart_remaining_time = data.dart_remaining_time;
    msg->target_type = data.target_type;
    msg->hit_count = data.hit_count;
    msg->selected_target = data.selected_target;

    pub_0x0105_->publish(std::move(msg));
}

// ============================================================
// Parser 实现 - 0x020x (机器人状态类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0201(const std::vector<uint8_t>& payload) {
    RobotStatusData data{};
    if (!load_struct(0x0201, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RobotStatus>();
    msg->robot_id = data.robot_id;
    msg->level = data.level;
    msg->current_hp = data.current_hp;
    msg->max_hp = data.max_hp;
    msg->heat_cooling_rate = data.heat_cooling_rate;
    msg->heat_max = data.heat_max;
    msg->chassis_power_limit = data.chassis_power_limit;
    msg->gimbal_power = data.gimbal_power;
    msg->chassis_power = data.chassis_power;
    msg->shooter_power = data.shooter_power;

    pub_0x0201_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0202(const std::vector<uint8_t>& payload) {
    // if (!_validate_payload_size(sizeof(BufferAndHeatData), payload.size(), 0x0202)) return;

    BufferAndHeatData data = {};
    std::memcpy(&data, payload.data(), sizeof(BufferAndHeatData));

    auto msg = std::make_unique<rm_message::msg::BufferAndHeat>();
    msg->buffer_energy = data.buffer_energy;
    msg->heat_17mm = data.heat_17mm;
    msg->heat_42mm = data.heat_42mm;

    pub_0x0202_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0203(const std::vector<uint8_t>& payload) {
    RobotPositionData data{};
    if (!load_struct(0x0203, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RobotPosition>();
    msg->x = data.x;
    msg->y = data.y;
    msg->yaw = data.yaw;

    pub_0x0203_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0204(const std::vector<uint8_t>& payload) {
    RobotBuffsData data{};
    if (!load_struct(0x0204, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RobotBuffs>();
    msg->hp_buff = data.hp_buff;
    msg->cooling_buff = data.cooling_buff;
    msg->defense_buff = data.defense_buff;
    msg->negative_defense_buff = data.negative_defense_buff;
    msg->attack_buff = data.attack_buff;
    msg->energy_125 = (data.energy_flags >> 0) & 0x01;
    msg->energy_100 = (data.energy_flags >> 1) & 0x01;
    msg->energy_50 = (data.energy_flags >> 2) & 0x01;
    msg->energy_30 = (data.energy_flags >> 3) & 0x01;
    msg->energy_15 = (data.energy_flags >> 4) & 0x01;
    msg->energy_5 = (data.energy_flags >> 5) & 0x01;
    msg->energy_1 = (data.energy_flags >> 6) & 0x01;

    pub_0x0204_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0206(const std::vector<uint8_t>& payload) {
    HurtEventData data{};
    if (!load_struct(0x0206, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::HurtEvent>();
    msg->armor_id = data.armor_id;
    msg->hurt_type = data.hurt_type;

    pub_0x0206_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0207(const std::vector<uint8_t>& payload) {
    ShootDataData data{};
    if (!load_struct(0x0207, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::ShootData>();
    msg->ammo_type = (data.ammo_type_17mm << 1) | (data.ammo_type_42mm << 2);
    msg->shoot_mechanism = data.shoot_mechanism;
    msg->shoot_speed = data.shoot_speed;
    msg->initial_velocity = data.initial_velocity;

    pub_0x0207_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0208(const std::vector<uint8_t>& payload) {
    AmmoAllowanceData data{};
    if (!load_struct(0x0208, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::AmmoAllowance>();
    msg->ammo_17mm = data.ammo_17mm;
    msg->ammo_42mm = data.ammo_42mm;
    msg->coin_count = data.coin_count;
    msg->projectile_allowance_fortress = data.projectile_allowance_fortress;

    pub_0x0208_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0209(const std::vector<uint8_t>& payload) {
    RFIDStatusData data{};
    if (!load_struct(0x0209, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RFIDStatus>();
    
    // 解析第一个 uint32 (offset 0, 4字节)
    msg->base_gain = (data.rfid_status >> 0) & 0x01;
    msg->own_center_highland = (data.rfid_status >> 1) & 0x01;
    msg->enemy_center_highland = (data.rfid_status >> 2) & 0x01;
    msg->own_trapezoid_highland = (data.rfid_status >> 3) & 0x01;
    msg->enemy_trapezoid_highland = (data.rfid_status >> 4) & 0x01;
    
    // bit 5-8: 地形跨越（飞坡）
    msg->terrain_crossing_ramp_own_near_front = (data.rfid_status >> 5) & 0x01;
    msg->terrain_crossing_ramp_own_near_back = (data.rfid_status >> 6) & 0x01;
    msg->terrain_crossing_ramp_enemy_near_front = (data.rfid_status >> 7) & 0x01;
    msg->terrain_crossing_ramp_enemy_near_back = (data.rfid_status >> 8) & 0x01;
    
    // bit 9-12: 地形跨越（中央高地）
    msg->terrain_crossing_highland_own_below = (data.rfid_status >> 9) & 0x01;
    msg->terrain_crossing_highland_own_above = (data.rfid_status >> 10) & 0x01;
    msg->terrain_crossing_highland_enemy_below = (data.rfid_status >> 11) & 0x01;
    msg->terrain_crossing_highland_enemy_above = (data.rfid_status >> 12) & 0x01;
    
    // bit 13-16: 地形跨越（公路区）
    msg->terrain_crossing_road_own_below = (data.rfid_status >> 13) & 0x01;
    msg->terrain_crossing_road_own_above = (data.rfid_status >> 14) & 0x01;
    msg->terrain_crossing_road_enemy_below = (data.rfid_status >> 15) & 0x01;
    msg->terrain_crossing_road_enemy_above = (data.rfid_status >> 16) & 0x01;
    
    // bit 17-25: 其他增益点
    msg->own_fort = (data.rfid_status >> 17) & 0x01;
    msg->own_outpost = (data.rfid_status >> 18) & 0x01;
    msg->own_supply_area = (data.rfid_status >> 19) & 0x01;
    msg->own_overlap_zone = (data.rfid_status >> 20) & 0x01;
    msg->own_continuous = (data.rfid_status >> 21) & 0x01;
    msg->enemy_continuous = (data.rfid_status >> 22) & 0x01;
    msg->center_rmul = (data.rfid_status >> 23) & 0x01;
    msg->enemy_fort = (data.rfid_status >> 24) & 0x01;
    msg->enemy_outpost = (data.rfid_status >> 25) & 0x01;
    
    // bit 26-31: 地形跨越（隧道）
    msg->terrain_crossing_tunnel_own_near_below = (data.rfid_status >> 26) & 0x01;
    msg->terrain_crossing_tunnel_own_near_above = (data.rfid_status >> 27) & 0x01;
    msg->terrain_crossing_tunnel_own_trapezoid_start = (data.rfid_status >> 28) & 0x01;
    msg->terrain_crossing_tunnel_own_trapezoid_end = (data.rfid_status >> 29) & 0x01;
    msg->terrain_crossing_tunnel_enemy_near_below = (data.rfid_status >> 30) & 0x01;
    msg->terrain_crossing_tunnel_enemy_near_above = (data.rfid_status >> 31) & 0x01;
    
    // 解析第二个 uint8 (offset 4, 1字节)
    msg->terrain_crossing_tunnel_enemy_trapezoid_low = (data.rfid_status_2 >> 0) & 0x01;
    msg->terrain_crossing_tunnel_enemy_trapezoid_high = (data.rfid_status_2 >> 1) & 0x01;

    pub_0x0209_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020A(const std::vector<uint8_t>& payload) {
    DartCmdData data{};
    if (!load_struct(0x020A, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::DartCmd>();
    msg->dart_station_status = data.dart_station_status;
    msg->reserved = data.reserved;
    msg->target_switch_countdown = data.target_switch_countdown;
    msg->last_shoot_countdown = data.last_shoot_countdown;

    pub_0x020A_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020B(const std::vector<uint8_t>& payload) {
    GroundPositionsData data{};
    if (!load_struct(0x020B, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::GroundPositions>();
    msg->hero_x = data.hero_x;
    msg->hero_y = data.hero_y;
    msg->engineer_x = data.engineer_x;
    msg->engineer_y = data.engineer_y;
    msg->infantry3_x = data.infantry3_x;
    msg->infantry3_y = data.infantry3_y;
    msg->infantry4_x = data.infantry4_x;
    msg->infantry4_y = data.infantry4_y;
    msg->reserved1 = data.reserved1;
    msg->reserved2 = data.reserved2;

    pub_0x020B_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020C(const std::vector<uint8_t>& payload) {
    RadarMarkData data{};
    if (!load_struct(0x020C, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RadarMark>();
    
    // 解析每个bit位
    msg->enemy_hero_vulnerable = (data.radar_mark_status >> 0) & 0x01;
    msg->enemy_engineer_vulnerable = (data.radar_mark_status >> 1) & 0x01;
    msg->enemy_infantry3_vulnerable = (data.radar_mark_status >> 2) & 0x01;
    msg->enemy_infantry4_vulnerable = (data.radar_mark_status >> 3) & 0x01;
    msg->enemy_sentry_vulnerable = (data.radar_mark_status >> 4) & 0x01;
    msg->own_hero_special_mark = (data.radar_mark_status >> 5) & 0x01;
    msg->own_engineer_special_mark = (data.radar_mark_status >> 6) & 0x01;
    msg->own_infantry3_special_mark = (data.radar_mark_status >> 7) & 0x01;
    msg->own_infantry4_special_mark = (data.radar_mark_status >> 8) & 0x01;
    msg->own_sentry_special_mark = (data.radar_mark_status >> 9) & 0x01;

    pub_0x020C_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020D(const std::vector<uint8_t>& payload) {
    SentryDecisionData data{};
    if (!load_struct(0x020D, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::SentryDecision>();
    
    // 解析 sentry_info (32位)
    msg->allow_bullet_count = (data.sentry_info >> 0) & 0x7FF;        // bit 0-10 (11位)
    msg->exchange_bullet_count = (data.sentry_info >> 11) & 0x0F;     // bit 11-14 (4位)
    msg->exchange_hp_count = (data.sentry_info >> 15) & 0x0F;         // bit 15-18 (4位)
    msg->free_revive_available = (data.sentry_info >> 19) & 0x01;     // bit 19
    msg->immediate_revive_available = (data.sentry_info >> 20) & 0x01; // bit 20
    msg->immediate_revive_coin_cost = (data.sentry_info >> 21) & 0x3FF; // bit 21-30 (10位)
    
    // 解析 sentry_info_2 (16位)
    msg->sentry_attitude = (data.sentry_info_2 >> 0) & 0x03;          // bit 0-1 (2位)
    msg->energy_gear_available = (data.sentry_info_2 >> 2) & 0x01;   // bit 2

    pub_0x020D_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020E(const std::vector<uint8_t>& payload) {
    RadarDecisionData data{};
    if (!load_struct(0x020E, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RadarDecision>();
    msg->double_vulnerable_count = (data.radar_info >> 0) & 0x03;  // bit 0-1 (2位)
    msg->enemy_vulnerable = (data.radar_info >> 2) & 0x01;         // bit 2
    msg->encryption_level = (data.radar_info >> 3) & 0x03;         // bit 3-4 (2位)
    msg->can_modify_key = (data.radar_info >> 5) & 0x01;          // bit 5

    pub_0x020E_->publish(std::move(msg));
}

// ============================================================
// Parser 实现 - 0x030x (交互数据与图传类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0304(const std::vector<uint8_t>& payload) {
    RefRemoteControlData data{};
    if (!load_struct(0x0304, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RefRemoteControl>();
    msg->mouse_vx = data.mouse_vx;
    msg->mouse_vy = data.mouse_vy;
    msg->wheel_speed = data.wheel_speed;
    msg->left_button = data.left_button;
    msg->right_button = data.right_button;
    msg->w = data.w;
    msg->s = data.s;
    msg->a = data.a;
    msg->d = data.d;

    pub_0x0304_->publish(std::move(msg));
}

// ============================================================
// Parser 实现 - 0x030x (新增的交互数据类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0301(const std::vector<uint8_t>& payload) {
    // 可变长度消息，至少需要6字节的头部
    if (payload.size() < 6) {
        RCLCPP_WARN(logger_, "Payload size too small for cmd_id 0x0301: expected at least 6 bytes, got %zu bytes", payload.size());
        return;
    }

    auto msg = std::make_unique<rm_message::msg::RobotInteraction>();
    msg->content_id = *reinterpret_cast<const uint16_t*>(&payload[0]);
    msg->sender_id = *reinterpret_cast<const uint16_t*>(&payload[2]);
    msg->receiver_id = *reinterpret_cast<const uint16_t*>(&payload[4]);
    
    // 复制可变长度数据
    if (payload.size() > 6) {
        msg->data.assign(payload.begin() + 6, payload.end());
    }

    pub_0x0301_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0302(const std::vector<uint8_t>& payload) {
    CustomControllerData data{};
    if (!load_struct(0x0302, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::CustomController>();
    std::copy(std::begin(data.data), std::end(data.data), msg->data.begin());

    pub_0x0302_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0303(const std::vector<uint8_t>& payload) {
    MapDownlinkData data{};
    if (!load_struct(0x0303, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::MapDownlink>();
    msg->target_x = data.target_x;
    msg->target_y = data.target_y;
    msg->cmd_keyboard = data.cmd_keyboard;
    msg->target_robot_id = data.target_robot_id;
    msg->source_id = data.source_id;

    pub_0x0303_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0305(const std::vector<uint8_t>& payload) {
    RadarMapUplinkData data{};
    if (!load_struct(0x0305, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RadarMapUplink>();
    msg->hero_x = data.hero_x;
    msg->hero_y = data.hero_y;
    msg->engineer_x = data.engineer_x;
    msg->engineer_y = data.engineer_y;
    msg->infantry3_x = data.infantry3_x;
    msg->infantry3_y = data.infantry3_y;
    msg->infantry4_x = data.infantry4_x;
    msg->infantry4_y = data.infantry4_y;
    msg->infantry5_x = data.infantry5_x;
    msg->infantry5_y = data.infantry5_y;
    msg->sentry_x = data.sentry_x;
    msg->sentry_y = data.sentry_y;

    pub_0x0305_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0306(const std::vector<uint8_t>& payload) {
    KeyMouseSimulationData data{};
    if (!load_struct(0x0306, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::KeyMouseSimulation>();
    msg->key1 = data.key1;
    msg->key2 = data.key2;
    msg->mouse_x = data.mouse_x;
    msg->left_button = data.left_button;
    msg->mouse_y = data.mouse_y;
    msg->right_button = data.right_button;

    pub_0x0306_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0307(const std::vector<uint8_t>& payload) {
    PathUplinkData data{};
    if (!load_struct(0x0307, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::PathUplink>();
    msg->intention = data.intention;
    msg->start_x = data.start_x;
    msg->start_y = data.start_y;
    std::copy(std::begin(data.delta_x), std::end(data.delta_x), msg->delta_x.begin());
    std::copy(std::begin(data.delta_y), std::end(data.delta_y), msg->delta_y.begin());
    msg->sender_id = data.sender_id;
    
    pub_0x0307_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0308(const std::vector<uint8_t>& payload) {
    CustomStringData data{};
    if (!load_struct(0x0308, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::CustomString>();
    msg->sender_id = data.sender_id;
    msg->receiver_id = data.receiver_id;
    std::copy(std::begin(data.content), std::end(data.content), msg->content.begin());

    pub_0x0308_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0309(const std::vector<uint8_t>& payload) {
    RobotCustomDataData data{};
    if (!load_struct(0x0309, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RobotCustomData>();
    std::copy(std::begin(data.data), std::end(data.data), msg->data.begin());

    pub_0x0309_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0310(const std::vector<uint8_t>& payload) {
    RobotCustomDataLargeData data{};
    if (!load_struct(0x0310, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::RobotCustomDataLarge>();
    std::copy(std::begin(data.data), std::end(data.data), msg->data.begin());

    pub_0x0310_->publish(std::move(msg));
}

// ============================================================
// Parser 实现 - 0x0F0x (VTM信道配置类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0F01(const std::vector<uint8_t>& payload) {
    SetVTMChannelData data{};
    if (!load_struct(0x0F01, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::SetVTMChannel>();
    msg->channel = data.channel;

    pub_0x0F01_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0F02(const std::vector<uint8_t>& payload) {
    QueryVTMChannelData data{};
    if (!load_struct(0x0F02, payload, data)) return;

    auto msg = std::make_unique<rm_message::msg::QueryVTMChannel>();
    msg->channel = data.channel;

    pub_0x0F02_->publish(std::move(msg));
}

// ============================================================
// 通用消息处理
// ============================================================

void MsgPublisher::_publish_general_message(uint16_t cmd_id, const std::vector<uint8_t>& payload) {
    auto msg = std::make_unique<rm_message::msg::GeneralMessage>();
    msg->cmd_id = cmd_id;
    msg->data_length = payload.size();
    msg->data_payload = payload;

    pub_general_->publish(std::move(msg));
}

void MsgPublisher::_publish_to_all_messages(uint16_t cmd_id, const std::vector<uint8_t>& payload) {
    auto msg = std::make_unique<rm_message::msg::GeneralMessage>();
    msg->cmd_id = cmd_id;
    msg->data_length = payload.size();
    msg->data_payload = payload;

    pub_all_messages_->publish(std::move(msg));
}

} // namespace RMManager
