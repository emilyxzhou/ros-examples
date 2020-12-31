cmake_minimum_required(VERSION 2.8.3)
project(actions_examples)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  rospy
  std_msgs
)

add_action_files(
  FILES
  RollDie.action
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_example_action_client.test)
  add_rostest(test/test_example_action_server.test)
endif()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
