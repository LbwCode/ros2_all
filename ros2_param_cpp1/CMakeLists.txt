cmake_minimum_required(VERSION 3.8)
project(ros2_param_cpp1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查询依赖 
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

# 找本地代码 
add_executable(param_1 src/param_1.cpp)
add_executable(param_2 src/param_2.cpp)
add_executable(param_3 src/param_3.cpp)

# 找第三方库 
ament_target_dependencies(param_1 rclcpp)
ament_target_dependencies(param_2 rclcpp)
ament_target_dependencies(param_3 rclcpp)

# 生成名
install(TARGETS
  param_1
  param_2
  param_3
  DESTINATION lib/${PROJECT_NAME})
  
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