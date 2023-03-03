#include <atomic>
#include <cinttypes>
#include <example_interfaces/action/fibonacci.hpp>
#include <future>
#include <queue>
#include <rcl_action/action_client.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp>
#include <rmw/qos_profiles.h>
namespace {
#define let const auto
#define let_mut auto

} // namespace

namespace action_example {

class MinimalActionClient : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit MinimalActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("minimal_action_client", node_options), goal_done_(false) {

    let qos_option = rcl_action_client_get_default_options();

    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "fibonacci", nullptr, qos_option);

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MinimalActionClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;
    if (std::atomic_flag_test_and_set(&remote_busy_)) {
      return;
    }

    if (!goal_handle_futures_.empty()) {
      let &future = goal_handle_futures_.front();
      using namespace std::chrono_literals;

      // let ret_code = rclcpp::spin_until_future_complete(
      //     this->get_node_base_interface(), future, 100ms);
      let ret_code = future.wait_for(100ms);
      if (ret_code == std::future_status::ready) {
        goal_handle_futures_.pop();
      } else if (ret_code == std::future_status::timeout) {
        std::atomic_flag_clear(&remote_busy_);
        return;
      } else {
        throw std::runtime_error("Unexpected future return code");
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MinimalActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MinimalActionClient::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    goal_handle_futures_.emplace(std::move(goal_handle_future));
  }

private:
  using GoalFuture = std::shared_future<std::shared_ptr<
      rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>>>;
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  std::queue<GoalFuture> goal_handle_futures_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic_flag remote_busy_ = false;
  bool goal_done_;

  void goal_response_callback(GoalHandleFibonacci::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void
  feedback_callback(GoalHandleFibonacci::SharedPtr,
                    const std::shared_ptr<const Fibonacci::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(),
                "Next number in sequence received: %" PRId32,
                feedback->sequence.back());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
      RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
    }

    std::atomic_flag_clear(&remote_busy_);
  }
}; // class MinimalActionClient
} // namespace action_example
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(action_example::MinimalActionClient)