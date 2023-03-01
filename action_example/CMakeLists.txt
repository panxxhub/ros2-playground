cmake_minimum_required(VERSION 3.8)
project(action_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_components
  example_interfaces
)

foreach(dependency ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(fib_components SHARED)
target_sources(fib_components
  PRIVATE
  src/fib_server.cpp
  src/fib_client.cpp
)
target_include_directories(fib_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fib_components PUBLIC c_std_11 cxx_std_20) # Require C99 and C++17
ament_target_dependencies(fib_components ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_nodes(fib_components
  "action_example::MinimalActionServer"
  "action_example::MinimalActionClient"
)

add_executable(fib_main)
target_sources(fib_main
  PRIVATE
  src/fib_main.cpp
)
target_link_libraries(fib_main)
ament_target_dependencies(fib_main ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(class_loader REQUIRED)
target_link_libraries(fib_main
  "-Wl,--no-as-needed"
  fib_components
  "-Wl,--as-needed"
)
ament_target_dependencies(fib_main class_loader)

# install(DIRECTORY include/
# DESTINATION include
# )
install(
  TARGETS
  fib_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS fib_main
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
