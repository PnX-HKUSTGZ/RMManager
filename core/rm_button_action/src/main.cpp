#include "rm_button_action/rm_button_action.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_button_action::RmButtonAction>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
