#ifndef RM_BUTTON_ACTION__RM_BUTTON_ACTION_HPP_
#define RM_BUTTON_ACTION__RM_BUTTON_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rm_button_action/rm_button_action_parameters.hpp"

namespace rm_button_action
{

class RmButtonAction : public rclcpp::Node{
public:
    RmButtonAction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RmButtonAction() = default;

private:
    enum class ActionType {
        NONE,
        BOOL,
        FLOAT32,
        FLOAT64
    };

    struct Action {
        ActionType type = ActionType::NONE;
        std::string topic;
        bool bool_value = false;
        double float_value = 0.0;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float32_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr float64_pub;

        bool enabled() const {
            return type != ActionType::NONE && !topic.empty();
        }
    };

    void input_callback(const std_msgs::msg::Bool::SharedPtr msg);

    ActionType parse_action_type(const std::string & type) const;
    const char * action_type_name(ActionType type) const;

    Action build_action(
        const std::string & topic,
        const std::string & type,
        bool bool_value,
        double float_value);

    std::vector<Action> build_action_list(
        const std::vector<std::string> & topics,
        const std::vector<std::string> & types,
        const std::vector<bool> & bool_values,
        const std::vector<double> & float_values,
        const std::string & list_name);

    void execute_action(const Action & action);
    void execute_trigger_action(
        const std::vector<Action> & actions,
        size_t & index,
        const std::string & trigger_name);

    // 参数lisener
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::string input_topic_;

    Action press_state_action_;
    Action release_state_action_;

    std::vector<Action> press_edge_actions_;
    std::vector<Action> release_edge_actions_;

    size_t press_edge_index_ = 0;
    size_t release_edge_index_ = 0;

    bool last_state_ = false;
    bool have_last_state_ = false;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr input_sub_;
};

} // namespace rm_button_action

#endif  // RM_BUTTON_ACTION__RM_BUTTON_ACTION_HPP_
