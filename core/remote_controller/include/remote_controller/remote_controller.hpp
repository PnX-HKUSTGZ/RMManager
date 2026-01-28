#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/remote_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "std_msgs/msg/bool.hpp"
#include <remote_controller/remote_controller_parameters.hpp>

#ifndef RMREMOTE_CONTROLLER_HPP
#define RMREMOTE_CONTROLLER_HPP

namespace RM_REMOTE_CONTROLLER {

enum class REMOTE_CONTROL_BUTTON {
    STOP,
    KEYL,
    KEYR,
    KEYB,
    PRESSL,
    PRESSR,
    PRESSMID,
    W,
    S,
    A,
    D,
    SHIFT,
    CTRL,
    Q,
    E,
    R,
    F,
    G,
    Z,
    X,
    C,
    V,
    B
};

static const std::unordered_map<REMOTE_CONTROL_BUTTON, std::string> REMOTE_CONTROL_BUTTON_NAMES = {
    {REMOTE_CONTROL_BUTTON::STOP, "STOP"},
    {REMOTE_CONTROL_BUTTON::KEYL, "KEYL"},
    {REMOTE_CONTROL_BUTTON::KEYR, "KEYR"},
    {REMOTE_CONTROL_BUTTON::KEYB, "KEYB"},
    {REMOTE_CONTROL_BUTTON::PRESSL, "PRESSL"},
    {REMOTE_CONTROL_BUTTON::PRESSR, "PRESSR"},
    {REMOTE_CONTROL_BUTTON::PRESSMID, "PRESSMID"},
    {REMOTE_CONTROL_BUTTON::W, "W"},
    {REMOTE_CONTROL_BUTTON::S, "S"},
    {REMOTE_CONTROL_BUTTON::A, "A"},
    {REMOTE_CONTROL_BUTTON::D, "D"},
    {REMOTE_CONTROL_BUTTON::SHIFT, "SHIFT"},
    {REMOTE_CONTROL_BUTTON::CTRL, "CTRL"},
    {REMOTE_CONTROL_BUTTON::Q, "Q"},
    {REMOTE_CONTROL_BUTTON::E, "E"},
    {REMOTE_CONTROL_BUTTON::R, "R"},
    {REMOTE_CONTROL_BUTTON::F, "F"},
    {REMOTE_CONTROL_BUTTON::G, "G"},
    {REMOTE_CONTROL_BUTTON::Z, "Z"},
    {REMOTE_CONTROL_BUTTON::X, "X"},
    {REMOTE_CONTROL_BUTTON::C, "C"},
    {REMOTE_CONTROL_BUTTON::V, "V"},
    {REMOTE_CONTROL_BUTTON::B, "B"}
};

static const std::unordered_map<std::string, REMOTE_CONTROL_BUTTON> REMOTE_CONTROL_BUTTON_MAP = {
    {"STOP", REMOTE_CONTROL_BUTTON::STOP},
    {"KEYL", REMOTE_CONTROL_BUTTON::KEYL},
    {"KEYR", REMOTE_CONTROL_BUTTON::KEYR},
    {"KEYB", REMOTE_CONTROL_BUTTON::KEYB},
    {"PRESSL", REMOTE_CONTROL_BUTTON::PRESSL},
    {"PRESSR", REMOTE_CONTROL_BUTTON::PRESSR},
    {"PRESSMID", REMOTE_CONTROL_BUTTON::PRESSMID},
    {"W", REMOTE_CONTROL_BUTTON::W},
    {"S", REMOTE_CONTROL_BUTTON::S},
    {"A", REMOTE_CONTROL_BUTTON::A},
    {"D", REMOTE_CONTROL_BUTTON::D},
    {"SHIFT", REMOTE_CONTROL_BUTTON::SHIFT},
    {"CTRL", REMOTE_CONTROL_BUTTON::CTRL},
    {"Q", REMOTE_CONTROL_BUTTON::Q},
    {"E", REMOTE_CONTROL_BUTTON::E},
    {"R", REMOTE_CONTROL_BUTTON::R},
    {"F", REMOTE_CONTROL_BUTTON::F},
    {"G", REMOTE_CONTROL_BUTTON::G},
    {"Z", REMOTE_CONTROL_BUTTON::Z},
    {"X", REMOTE_CONTROL_BUTTON::X},
    {"C", REMOTE_CONTROL_BUTTON::C},
    {"V", REMOTE_CONTROL_BUTTON::V},
    {"B", REMOTE_CONTROL_BUTTON::B}
};

enum class REMOTE_CONTROL_ACTION_TYPE {
    NONE,
    ON_REALSED,
    ON_PRESSED,
    ON_TOGGLED,
    ON_REALSED_OFF,
    ON_PRESSED_OFF,
};

static const std::unordered_map<REMOTE_CONTROL_ACTION_TYPE, std::string> REMOTE_CONTROL_ACTION_TYPE_NAMES = {
    {REMOTE_CONTROL_ACTION_TYPE::NONE, "NONE"},
    {REMOTE_CONTROL_ACTION_TYPE::ON_REALSED, "ON_REALSED"},
    {REMOTE_CONTROL_ACTION_TYPE::ON_PRESSED, "ON_PRESSED"},
    {REMOTE_CONTROL_ACTION_TYPE::ON_TOGGLED, "ON_TOGGLED"},
    {REMOTE_CONTROL_ACTION_TYPE::ON_REALSED_OFF, "ON_REALSED_OFF"},
    {REMOTE_CONTROL_ACTION_TYPE::ON_PRESSED_OFF, "ON_PRESSED_OFF"}
};

static const std::unordered_map<std::string, REMOTE_CONTROL_ACTION_TYPE> REMOTE_CONTROL_ACTION_TYPE_MAP = {
    {"NONE", REMOTE_CONTROL_ACTION_TYPE::NONE},
    {"ON_REALSED", REMOTE_CONTROL_ACTION_TYPE::ON_REALSED},
    {"ON_PRESSED", REMOTE_CONTROL_ACTION_TYPE::ON_PRESSED},
    {"ON_TOGGLED", REMOTE_CONTROL_ACTION_TYPE::ON_TOGGLED},
    {"ON_REALSED_OFF", REMOTE_CONTROL_ACTION_TYPE::ON_REALSED_OFF},
    {"ON_PRESSED_OFF", REMOTE_CONTROL_ACTION_TYPE::ON_PRESSED_OFF}
};

class ButtonAction {
public:
    explicit ButtonAction(
        rclcpp::Node * node,
        REMOTE_CONTROL_BUTTON button,
        REMOTE_CONTROL_ACTION_TYPE action_type, 
        std::string topic, 
        bool content);

    void execute(const std::vector<REMOTE_CONTROL_ACTION_TYPE> & trigger_types);

    REMOTE_CONTROL_ACTION_TYPE get_action_type() const {
        return action_type_;
    }

    REMOTE_CONTROL_BUTTON get_button() const {
        return button_;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    REMOTE_CONTROL_ACTION_TYPE action_type_;
    REMOTE_CONTROL_BUTTON button_;
    bool content_;
};

class RemoteController : public rclcpp::Node{
public:
    RemoteController(std::string  name = "remote_controller");
    ~RemoteController();
private:
    // Parameter handler
    std::shared_ptr<remote_controller::ParamListener> param_listener_;
    std::shared_ptr<remote_controller::Params> params_;

    rclcpp::Subscription<rm_message::msg::RemoteControl>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr chasis_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_enable_pub_;

    // Watchdog timer
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_message_time_;
    bool watchdog_triggered_ = false;

    void cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg);

    void watchdogCallback();

    void sendVel(const rm_message::msg::RemoteControl::SharedPtr msg);

    void keyboardSendVel(const rm_message::msg::RemoteControl::SharedPtr msg);

    void sendEnableChasis(const rm_message::msg::RemoteControl::SharedPtr msg);

    void sendEnableArm(const rm_message::msg::RemoteControl::SharedPtr msg);

    float last_x_ = 0;
    float last_y_ = 0;
    float last_z_ = 0;

    std::string stop_button_;

    REMOTE_CONTROL_BUTTON keyboard_remote_control_exchange_button_;

    REMOTE_CONTROL_BUTTON keyboard_forward_button_;
    REMOTE_CONTROL_BUTTON keyboard_backward_button_;
    REMOTE_CONTROL_BUTTON keyboard_left_button_;
    REMOTE_CONTROL_BUTTON keyboard_right_button_;
    REMOTE_CONTROL_BUTTON keyboard_up_button_;
    REMOTE_CONTROL_BUTTON keyboard_down_button_;
    double keyboard_up_ratio_ = 1.5;
    double keyboard_down_ratio_ = 0.5;

    rclcpp::Time last_time_;

    // 储存某个按键上一次的状态，用于检测按键边沿
    std::unordered_map<REMOTE_CONTROL_BUTTON, uint8_t> last_button_states_ = {
        {REMOTE_CONTROL_BUTTON::STOP, 0},
        {REMOTE_CONTROL_BUTTON::KEYL, 0},
        {REMOTE_CONTROL_BUTTON::KEYR, 0},
        {REMOTE_CONTROL_BUTTON::KEYB, 0},
        {REMOTE_CONTROL_BUTTON::PRESSL, 0},
        {REMOTE_CONTROL_BUTTON::PRESSR, 0},
        {REMOTE_CONTROL_BUTTON::PRESSMID, 0},
        {REMOTE_CONTROL_BUTTON::W, 0},
        {REMOTE_CONTROL_BUTTON::S, 0},
        {REMOTE_CONTROL_BUTTON::A, 0},
        {REMOTE_CONTROL_BUTTON::D, 0},
        {REMOTE_CONTROL_BUTTON::SHIFT, 0},
        {REMOTE_CONTROL_BUTTON::CTRL, 0},
        {REMOTE_CONTROL_BUTTON::Q, 0},
        {REMOTE_CONTROL_BUTTON::E, 0},
        {REMOTE_CONTROL_BUTTON::R, 0},
        {REMOTE_CONTROL_BUTTON::F, 0},
        {REMOTE_CONTROL_BUTTON::G, 0},
        {REMOTE_CONTROL_BUTTON::Z, 0},
        {REMOTE_CONTROL_BUTTON::X, 0},
        {REMOTE_CONTROL_BUTTON::C, 0},
        {REMOTE_CONTROL_BUTTON::V, 0},
        {REMOTE_CONTROL_BUTTON::B, 0}
    };

    // 储存某个按钮是否被刚好被按下的，触发式
    std::unordered_map<REMOTE_CONTROL_BUTTON, bool> button_release_off_ = {
        {REMOTE_CONTROL_BUTTON::STOP, false},
        {REMOTE_CONTROL_BUTTON::KEYL, false},
        {REMOTE_CONTROL_BUTTON::KEYR, false},
        {REMOTE_CONTROL_BUTTON::KEYB, false},
        {REMOTE_CONTROL_BUTTON::PRESSL, false},
        {REMOTE_CONTROL_BUTTON::PRESSR, false},
        {REMOTE_CONTROL_BUTTON::PRESSMID, false},
        {REMOTE_CONTROL_BUTTON::W, false},
        {REMOTE_CONTROL_BUTTON::S, false},
        {REMOTE_CONTROL_BUTTON::A, false},
        {REMOTE_CONTROL_BUTTON::D, false},
        {REMOTE_CONTROL_BUTTON::SHIFT, false},
        {REMOTE_CONTROL_BUTTON::CTRL, false},
        {REMOTE_CONTROL_BUTTON::Q, false},
        {REMOTE_CONTROL_BUTTON::E, false},
        {REMOTE_CONTROL_BUTTON::R, false},
        {REMOTE_CONTROL_BUTTON::F, false},
        {REMOTE_CONTROL_BUTTON::G, false},
        {REMOTE_CONTROL_BUTTON::Z, false},
        {REMOTE_CONTROL_BUTTON::X, false},
        {REMOTE_CONTROL_BUTTON::C, false},
        {REMOTE_CONTROL_BUTTON::V, false},
        {REMOTE_CONTROL_BUTTON::B, false}
    };

    // 储存某个按钮是否被刚好被释放的，触发式
    std::unordered_map<REMOTE_CONTROL_BUTTON, bool> button_pressed_off_ = {
        {REMOTE_CONTROL_BUTTON::STOP, false},
        {REMOTE_CONTROL_BUTTON::KEYL, false},
        {REMOTE_CONTROL_BUTTON::KEYR, false},
        {REMOTE_CONTROL_BUTTON::KEYB, false},
        {REMOTE_CONTROL_BUTTON::PRESSL, false},
        {REMOTE_CONTROL_BUTTON::PRESSR, false},
        {REMOTE_CONTROL_BUTTON::PRESSMID, false},
        {REMOTE_CONTROL_BUTTON::W, false},
        {REMOTE_CONTROL_BUTTON::S, false},
        {REMOTE_CONTROL_BUTTON::A, false},
        {REMOTE_CONTROL_BUTTON::D, false},
        {REMOTE_CONTROL_BUTTON::SHIFT, false},
        {REMOTE_CONTROL_BUTTON::CTRL, false},
        {REMOTE_CONTROL_BUTTON::Q, false},
        {REMOTE_CONTROL_BUTTON::E, false},
        {REMOTE_CONTROL_BUTTON::R, false},
        {REMOTE_CONTROL_BUTTON::F, false},
        {REMOTE_CONTROL_BUTTON::G, false},
        {REMOTE_CONTROL_BUTTON::Z, false},
        {REMOTE_CONTROL_BUTTON::X, false},
        {REMOTE_CONTROL_BUTTON::C, false},
        {REMOTE_CONTROL_BUTTON::V, false},
        {REMOTE_CONTROL_BUTTON::B, false}
    };

    // 储存某个按钮的状态，其每被触发一次就为就从false变为true，再次触发就变为false
    std::unordered_map<REMOTE_CONTROL_BUTTON, bool> button_toggled_ = {
        {REMOTE_CONTROL_BUTTON::STOP, false},
        {REMOTE_CONTROL_BUTTON::KEYL, false},
        {REMOTE_CONTROL_BUTTON::KEYR, false},
        {REMOTE_CONTROL_BUTTON::KEYB, false},
        {REMOTE_CONTROL_BUTTON::PRESSL, false},
        {REMOTE_CONTROL_BUTTON::PRESSR, false},
        {REMOTE_CONTROL_BUTTON::PRESSMID, false},
        {REMOTE_CONTROL_BUTTON::W, false},
        {REMOTE_CONTROL_BUTTON::S, false},
        {REMOTE_CONTROL_BUTTON::A, false},
        {REMOTE_CONTROL_BUTTON::D, false},
        {REMOTE_CONTROL_BUTTON::SHIFT, false},
        {REMOTE_CONTROL_BUTTON::CTRL, false},
        {REMOTE_CONTROL_BUTTON::Q, false},
        {REMOTE_CONTROL_BUTTON::E, false},
        {REMOTE_CONTROL_BUTTON::R, false},
        {REMOTE_CONTROL_BUTTON::F, false},
        {REMOTE_CONTROL_BUTTON::G, false},
        {REMOTE_CONTROL_BUTTON::Z, false},
        {REMOTE_CONTROL_BUTTON::X, false},
        {REMOTE_CONTROL_BUTTON::C, false},
        {REMOTE_CONTROL_BUTTON::V, false},
        {REMOTE_CONTROL_BUTTON::B, false}
    };

    std::unordered_map<REMOTE_CONTROL_BUTTON, bool> button_current_ = {
        {REMOTE_CONTROL_BUTTON::STOP, false},
        {REMOTE_CONTROL_BUTTON::KEYL, false},
        {REMOTE_CONTROL_BUTTON::KEYR, false},
        {REMOTE_CONTROL_BUTTON::KEYB, false},
        {REMOTE_CONTROL_BUTTON::PRESSL, false},
        {REMOTE_CONTROL_BUTTON::PRESSR, false},
        {REMOTE_CONTROL_BUTTON::PRESSMID, false},
        {REMOTE_CONTROL_BUTTON::W, false},
        {REMOTE_CONTROL_BUTTON::S, false},
        {REMOTE_CONTROL_BUTTON::A, false},
        {REMOTE_CONTROL_BUTTON::D, false},
        {REMOTE_CONTROL_BUTTON::SHIFT, false},
        {REMOTE_CONTROL_BUTTON::CTRL, false},
        {REMOTE_CONTROL_BUTTON::Q, false},
        {REMOTE_CONTROL_BUTTON::E, false},
        {REMOTE_CONTROL_BUTTON::R, false},
        {REMOTE_CONTROL_BUTTON::F, false},
        {REMOTE_CONTROL_BUTTON::G, false},
        {REMOTE_CONTROL_BUTTON::Z, false},
        {REMOTE_CONTROL_BUTTON::X, false},
        {REMOTE_CONTROL_BUTTON::C, false},
        {REMOTE_CONTROL_BUTTON::V, false},
        {REMOTE_CONTROL_BUTTON::B, false}
    };

    // 储存每个按键对应的action内容
    std::vector<std::shared_ptr<ButtonAction>> button_actions_;
    // 更新按钮状态
    void updateButtonStates(const rm_message::msg::RemoteControl::SharedPtr msg);

};

} // namespace RM_REMOTE_CONTROLLER

#endif // RMREMOTE_CONTROLLER_HPP