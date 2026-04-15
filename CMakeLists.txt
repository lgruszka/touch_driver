cmake_minimum_required(VERSION 2.8.3)
project(touch_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(FILES
  TouchButtonEvent.msg
  TouchFeedback.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Check for OpenHaptics
find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h)
if(NOT OPENHAPTICS_INCLUDE_DIR)
  message(WARNING "OpenHaptics not found. touch_node will not be built.")
  return()
endif()

include_directories(${OPENHAPTICS_INCLUDE_DIR})

add_executable(touch_node src/touch_node.cpp)
target_link_libraries(touch_node
  HD HDU
  rt ncurses
  ${catkin_LIBRARIES}
)
add_dependencies(touch_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS touch_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
