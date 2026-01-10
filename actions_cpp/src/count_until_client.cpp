#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;

class CountUntilClientNode : public rclcpp::Node // MODIFY NAME
{
public:
    CountUntilClientNode() : Node("count_until_client") // MODIFY NAME
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(
            this,
            "count_until");
    }

    void send_goal(int target_number, double period)
    {
        // Wait until the action server is available
        count_until_client_->wait_for_action_server();
        // Create a goal message
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        count_until_client_->async_send_goal(goal);
    }
        
private:
    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>(); // MODIFY NAME
    node->send_goal(6, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
