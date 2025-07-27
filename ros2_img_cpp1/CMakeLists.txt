cmake_minimum_required(VERSION 3.8)
project(ros2_img_cpp1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找工具
# 查找依赖（包含视频处理所需库）
find_package(sensor_msgs REQUIRED)       # 图像消息类型
find_package(image_transport REQUIRED)   # 图像传输工具
find_package(cv_bridge REQUIRED)         # OpenCV与ROS图像转换
find_package(OpenCV REQUIRED)            # OpenCV库

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# 1. 视频发布节点（video_publisher.cpp）
# 编译配方 (蛋糕名 鸡蛋、面粉) 
add_executable(video_publisher src/video_publisher.cpp)
# 工具箱(蛋糕名 搅拌机、烤箱)
ament_target_dependencies(
  video_publisher
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

# 2. 视频订阅节点（video_subscriber.cpp）
add_executable(video_subscriber src/video_subscriber.cpp)
ament_target_dependencies(
  video_subscriber
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

# 货架 蛋糕1 蛋糕2
install(TARGETS
  video_publisher
  video_subscriber
  DESTINATION lib/${PROJECT_NAME})

  
# 测试
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
