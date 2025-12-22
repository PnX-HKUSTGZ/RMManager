#include "rm_manager/msg_publisher.hpp"

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
    pub_0x0304_ = node->create_publisher<rm_message::msg::RefRemoteControl>(node_name + "ref_remote_control", 10);

    // 初始化通用消息publisher
    pub_general_ = node->create_publisher<rm_message::msg::GeneralMessage>(node_name + "unknown_command", 10);
    
    // 初始化所有消息的通用publisher
    pub_all_messages_ = node->create_publisher<rm_message::msg::GeneralMessage>(node_name + "all_messages", 10);

    RCLCPP_INFO(logger_, "MsgPublisher initialized with 20 specific publishers");
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
        case 0x0304: _parse_and_publish_0x0304(payload); break;
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
    if (!_validate_payload_size(sizeof(GameStateData), payload.size(), 0x0001)) return;

    GameStateData data = {};
    std::memcpy(&data, payload.data(), sizeof(GameStateData));

    auto msg = std::make_unique<rm_message::msg::GameState>();
    msg->game_type = data.game_type;
    msg->game_stage = data.game_stage;
    msg->rest_time = data.rest_time;
    msg->unix_time = data.unix_time;

    pub_0x0001_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0002(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(GameResultData), payload.size(), 0x0002)) return;

    GameResultData data = {};
    std::memcpy(&data, payload.data(), sizeof(GameResultData));

    auto msg = std::make_unique<rm_message::msg::GameResult>();
    msg->result = data.result;

    pub_0x0002_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0003(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(RobotHPData), payload.size(), 0x0003)) return;

    RobotHPData data = {};
    std::memcpy(&data, payload.data(), sizeof(RobotHPData));

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
    if (!_validate_payload_size(sizeof(FieldEventsData), payload.size(), 0x0101)) return;

    FieldEventsData data = {};
    std::memcpy(&data, payload.data(), sizeof(FieldEventsData));

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
    if (!_validate_payload_size(sizeof(RefereeWarningData), payload.size(), 0x0104)) return;

    RefereeWarningData data = {};
    std::memcpy(&data, payload.data(), sizeof(RefereeWarningData));

    auto msg = std::make_unique<rm_message::msg::RefereeWarning>();
    msg->penalty_level = data.penalty_level;
    msg->robot_id = data.robot_id;
    msg->violation_count = data.violation_count;

    pub_0x0104_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0105(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(4, payload.size(), 0x0105)) return;

    auto msg = std::make_unique<rm_message::msg::DartInfo>();
    msg->dart_remaining_time = payload[0];
    msg->target_type = payload[1] & 0x07;
    msg->hit_count = (payload[1] >> 3) & 0x07;
    msg->selected_target = (payload[2] >> 6) & 0x03;

    pub_0x0105_->publish(std::move(msg));
}

// ============================================================
// Parser 实现 - 0x020x (机器人状态类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0201(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(RobotStatusData), payload.size(), 0x0201)) return;

    RobotStatusData data = {};
    std::memcpy(&data, payload.data(), sizeof(RobotStatusData));

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
    if (!_validate_payload_size(sizeof(RobotPositionData), payload.size(), 0x0203)) return;

    RobotPositionData data = {};
    std::memcpy(&data, payload.data(), sizeof(RobotPositionData));

    auto msg = std::make_unique<rm_message::msg::RobotPosition>();
    msg->x = data.x;
    msg->y = data.y;
    msg->yaw = data.yaw;

    pub_0x0203_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0204(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(12, payload.size(), 0x0204)) return;

    auto msg = std::make_unique<rm_message::msg::RobotBuffs>();
    msg->hp_buff = payload[0];
    msg->cooling_buff = *reinterpret_cast<const uint16_t*>(&payload[1]);
    msg->defense_buff = payload[3];
    msg->negative_defense_buff = payload[4];
    msg->attack_buff = payload[5];
    msg->energy_125 = (payload[6] >> 0) & 0x01;
    msg->energy_100 = (payload[6] >> 1) & 0x01;
    msg->energy_50 = (payload[6] >> 2) & 0x01;
    msg->energy_30 = (payload[6] >> 3) & 0x01;
    msg->energy_15 = (payload[6] >> 4) & 0x01;
    msg->energy_5 = (payload[6] >> 5) & 0x01;
    msg->energy_1 = (payload[6] >> 6) & 0x01;

    pub_0x0204_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0206(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(HurtEventData), payload.size(), 0x0206)) return;

    HurtEventData data = {};
    std::memcpy(&data, payload.data(), sizeof(HurtEventData));

    auto msg = std::make_unique<rm_message::msg::HurtEvent>();
    msg->armor_id = data.armor_id;
    msg->hurt_type = data.hurt_type;

    pub_0x0206_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0207(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(ShootDataData), payload.size(), 0x0207)) return;

    ShootDataData data = {};
    std::memcpy(&data, payload.data(), sizeof(ShootDataData));

    auto msg = std::make_unique<rm_message::msg::ShootData>();
    msg->ammo_type = (data.ammo_type_17mm << 1) | (data.ammo_type_42mm << 2);
    msg->shoot_mechanism = data.shoot_mechanism;
    msg->shoot_speed = data.shoot_speed;
    msg->initial_velocity = data.initial_velocity;

    pub_0x0207_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0208(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(AmmoAllowanceData), payload.size(), 0x0208)) return;

    AmmoAllowanceData data = {};
    std::memcpy(&data, payload.data(), sizeof(AmmoAllowanceData));

    auto msg = std::make_unique<rm_message::msg::AmmoAllowance>();
    msg->ammo_17mm = data.ammo_17mm;
    msg->ammo_42mm = data.ammo_42mm;
    msg->coin_count = data.coin_count;

    pub_0x0208_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x0209(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(RFIDStatusData), payload.size(), 0x0209)) return;

    RFIDStatusData data = {};
    std::memcpy(&data, payload.data(), sizeof(RFIDStatusData));

    auto msg = std::make_unique<rm_message::msg::RFIDStatus>();
    // 解析bit字段到各个字段
    msg->base_gain = (data.rfid_status >> 0) & 0x01;
    msg->own_center_highland = (data.rfid_status >> 1) & 0x01;
    msg->enemy_center_highland = (data.rfid_status >> 2) & 0x01;
    msg->own_trapezoid_highland = (data.rfid_status >> 3) & 0x01;
    msg->enemy_trapezoid_highland = (data.rfid_status >> 4) & 0x01;
    msg->terrain_crossing_ramp = (data.rfid_status >> 5) & 0x0F;
    msg->terrain_crossing_highland = (data.rfid_status >> 9) & 0x0F;
    msg->terrain_crossing_road = (data.rfid_status >> 13) & 0x0F;
    msg->fort = (data.rfid_status >> 17) & 0x01;
    msg->outpost = (data.rfid_status >> 18) & 0x01;
    msg->supply_area = (data.rfid_status >> 19) & 0x01;
    msg->other_locations = (data.rfid_status >> 20) & 0x0FFF;

    pub_0x0209_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020A(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(DartCmdData), payload.size(), 0x020A)) return;

    DartCmdData data = {};
    std::memcpy(&data, payload.data(), sizeof(DartCmdData));

    auto msg = std::make_unique<rm_message::msg::DartCmd>();
    msg->dart_station_status = data.dart_station_status;
    msg->reserved = data.reserved;
    msg->target_switch_countdown = data.target_switch_countdown;
    msg->last_shoot_countdown = data.last_shoot_countdown;

    pub_0x020A_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020B(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(GroundPositionsData), payload.size(), 0x020B)) return;

    GroundPositionsData data = {};
    std::memcpy(&data, payload.data(), sizeof(GroundPositionsData));

    auto msg = std::make_unique<rm_message::msg::GroundPositions>();
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

    pub_0x020B_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020C(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(sizeof(RadarMarkData), payload.size(), 0x020C)) return;

    RadarMarkData data = {};
    std::memcpy(&data, payload.data(), sizeof(RadarMarkData));

    auto msg = std::make_unique<rm_message::msg::RadarMark>();
    msg->radar_mark_status = data.radar_mark_status;

    pub_0x020C_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020D(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(6, payload.size(), 0x020D)) return;

    auto msg = std::make_unique<rm_message::msg::SentryDecision>();
    msg->allow_bullet_count = *reinterpret_cast<const uint16_t*>(&payload[0]);
    msg->exchange_count = payload[2];
    msg->free_revive_available = payload[3];
    msg->immediate_revive_available = payload[4];
    msg->sentry_attitude = payload[5];
    msg->energy_gear_available = payload[6];

    pub_0x020D_->publish(std::move(msg));
}

void MsgPublisher::_parse_and_publish_0x020E(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(3, payload.size(), 0x020E)) return;

    auto msg = std::make_unique<rm_message::msg::RadarDecision>();
    msg->double_vulnerable_count = payload[0] & 0x03;
    msg->enemy_vulnerable = (payload[0] >> 2) & 0x01;
    msg->encryption_level = (payload[0] >> 3) & 0x03;

    pub_0x020E_->publish(std::move(msg));
}

// ============================================================
// Parser 实现 - 0x030x (交互数据与图传类)
// ============================================================

void MsgPublisher::_parse_and_publish_0x0304(const std::vector<uint8_t>& payload) {
    if (!_validate_payload_size(12, payload.size(), 0x0304)) return;

    auto msg = std::make_unique<rm_message::msg::RefRemoteControl>();
    msg->mouse_vx = *reinterpret_cast<const int16_t*>(&payload[0]);
    msg->mouse_vy = *reinterpret_cast<const int16_t*>(&payload[2]);
    msg->wheel_speed = *reinterpret_cast<const int16_t*>(&payload[4]);
    msg->left_button = payload[6];
    msg->right_button = payload[7];
    msg->w = payload[8];
    msg->s = payload[9];
    msg->a = payload[10];
    msg->d = payload[11];

    pub_0x0304_->publish(std::move(msg));
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
