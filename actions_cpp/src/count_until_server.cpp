#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"
using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGolalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;
class CountUntilServerNode : public rclcpp::Node // MODIFY NAME
{
public:
    CountUntilServerNode() : Node("count_until_server") // MODIFY NAME
    {
        count_until_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::goal_callback, this, _1, _2),
            std::bind(&CountUntilServerNode::cancel_callback, this,_1),
            std::bind(&CountUntilServerNode::handle_accepted_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Action Server has been started.");   

    }

private:
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal ");
        if(goal->target_number <=0)
        {
            RCLCPP_INFO(this->get_logger(), "Rejected a goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Accepted the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<CountUntilGolalHandle> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted_callback(
        const std::shared_ptr<CountUntilGolalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<CountUntilGolalHandle> goal_handle)
    {
        // Get request goal
        int target_number = goal_handle->get_goal()->target_number;
        double period = goal_handle->get_goal()->period;
        rclcpp::Rate loop_rate(1.0 / period);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        int counter = 0;
        auto feedback = std::make_shared<CountUntil::Feedback>();
        for (int i = 0; i < target_number; i++)
        {
            counter++;
            RCLCPP_INFO(this->get_logger(), "%d", counter);
            feedback->current_number = counter;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
        //Set final state and resut
        auto result = std::make_shared<CountUntil::Result>();
        result->reached_number = counter;
        goal_handle->succeed(result);
    }
        
        
    rclcpp_action::Server<my_robot_interfaces::action::CountUntil>::SharedPtr count_until_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
