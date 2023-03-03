#include "example_interfaces/action/fibonacci.hpp"
#include <rcl_action/action_server.h>
#include <rcl_action/default_qos.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmw/qos_profiles.h>
namespace {
#define let const auto
#define let_mut auto
// calls
/**
 * The defaults are:
 *
 * - goal_service_qos = rmw_qos_profile_services_default;
 * - cancel_service_qos = rmw_qos_profile_services_default;
 * - result_service_qos = rmw_qos_profile_services_default;
 * - feedback_topic_qos = rmw_qos_profile_default;
 * - status_topic_qos = rcl_action_qos_profile_status_default;
 * - allocator = rcl_get_default_allocator();
 * - result_timeout = RCUTILS_S_TO_NS(15 * 60);  // 15 minutes
 */

// const rcl_action_server_options_t rmw_qos_profile_actions = {
//     .goal_service_qos = rmw_qos_profile_services_default,
//     .cancel_service_qos = rmw_qos_profile_services_default,
//     .result_service_qos = rmw_qos_profile_services_default,
//     .feedback_topic_qos = rmw_qos_profile_sensor_data,
//     .status_topic_qos = rcl_action_qos_profile_status_default,
//     .allocator = rcl_get_default_allocator(),
//     .result_timeout = RCL_S_TO_NS(15 * 60),
// };
} // namespace

namespace action_example {
class MinimalActionServer : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit MinimalActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("minimal_action_server", options) {
    using namespace std::placeholders;

    let qos_option = rcl_action_server_get_default_options();

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "fibonacci",
        std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
        std::bind(&MinimalActionServer::handle_cancel, this, _1),
        std::bind(&MinimalActionServer::handle_accepted, this, _1), qos_option
        // rmw_qos_profile_services_hist_keep_all
    );
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Fibonacci::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d",
                goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto &sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      // goal_handle->reset();
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    let_mut t = std::thread{std::bind(&MinimalActionServer::execute, this, _1),
                            goal_handle};
    t.detach();
  }
}; // class MinimalActionServer

} // namespace action_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(action_example::MinimalActionServer)