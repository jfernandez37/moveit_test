cmake_minimum_required(VERSION 3.8)
project(moveit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp 
  moveit_ros_planning_interface
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Install the config directory to the package share directory
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/moveit_test_node.py
  nodes/individual_moveit_test_node.py
  nodes/cpp_test.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


# Add nist competitor library
add_library(robot_commander SHARED
  src/robot_commander.cpp
)
target_include_directories(robot_commander PUBLIC
  include
)

ament_target_dependencies(robot_commander ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(robot_commander_node nodes/robot_commander_node.cpp)
target_include_directories(robot_commander_node PRIVATE include)
target_link_libraries(robot_commander_node
robot_commander
)
ament_target_dependencies(robot_commander_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_libraries(
  robot_commander
)

install(TARGETS robot_commander
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_include_directories(
  include
)

install(TARGETS
  robot_commander_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
