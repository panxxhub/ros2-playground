#include <class_loader/class_loader.hpp>
#include <cstdio>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_components/node_factory.hpp>

namespace {
#define let const auto
#define let_mut auto
// constexpr auto ACTION_SERVER_LIB = "";

} // namespace

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  let loader = std::make_unique<class_loader::ClassLoader>("");
  let classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();

  rclcpp::Logger logger = rclcpp::get_logger("fib_action_server");
  RCLCPP_INFO(logger, "hello world action_example package");

  RCLCPP_INFO(logger, "classes.size(): %ld", classes.size());
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;
  node_wrappers.reserve(classes.size());

  for (let &it : classes) {

    // printf("class_name: %s\n", class_name.c_str());
    RCLCPP_INFO(logger, "class_name: %s", it.c_str());
    let node_factory =
        loader->createInstance<rclcpp_components::NodeFactory>(it);
    auto wrapper = node_factory->create_node_instance(rclcpp::NodeOptions());
    let node = wrapper.get_node_base_interface();
    node_wrappers.emplace_back(std::move(wrapper));
    executor.add_node(node);
  }

  // load the node( the node is registered through macro)

  executor.spin();
  rclcpp::shutdown();

  printf("hello world action_example package\n");
  return 0;
}
