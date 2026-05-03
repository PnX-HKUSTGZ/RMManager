#include "rm_ui/ui.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_ui::RmUi>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 4u);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
