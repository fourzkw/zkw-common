cmake_minimum_required(VERSION 3.8)
project(armor_check)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

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

include_directories(./include/armor_check)

aux_source_directory(./src/ori_node ORI_SRC)
aux_source_directory(./src/tra_node TRA_SRC)
aux_source_directory(./src/yolo_node YOLO_SRC)

add_executable(ori_node ./src/ori_node/main.cpp)
ament_target_dependencies(ori_node rclcpp OpenCV std_msgs)
target_link_libraries( ori_node ${OpenCV_LIBS} )

add_executable(tra_node ./src/tra_node/main.cpp)
ament_target_dependencies(tra_node rclcpp OpenCV std_msgs)
target_link_libraries( tra_node ${OpenCV_LIBS} )

add_executable(yolo_node ./src/yolo_node/main.cpp)
ament_target_dependencies(yolo_node rclcpp OpenCV std_msgs)
target_link_libraries( yolo_node ${OpenCV_LIBS} )

install(TARGETS
  ori_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tra_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  yolo_node
  DESTINATION lib/${PROJECT_NAME}
)