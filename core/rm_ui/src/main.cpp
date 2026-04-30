#include "rm_ui/ui.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_ui::RmUi>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
