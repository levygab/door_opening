cmake_minimum_required(VERSION 3.0.2)
project(opening_door)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cb_base_navigation_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
  robot_skills
  rospy
  roslint
  message_generation
  std_msgs
  pcl_ros
)

catkin_python_setup()

add_service_files(
  DIRECTORY srv
  FILES door_info.srv 
)

generate_messages( DEPENDENCIES std_msgs geometry_msgs)
###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros message_runtime std_msgs cb_base_navigation_msgs pcl_ros
)


###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_open_the_door_service
  src/service_server.cpp
)
add_executable(${PROJECT_NAME}_handle_locate
  src/handle_locate.cpp
)
add_executable(${PROJECT_NAME}_test_markers
  src/test_markers.cpp
)
add_dependencies(${PROJECT_NAME}_open_the_door_service ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}_open_the_door_service
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_handle_locate
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test_markers
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_open_the_door_service
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_handle_locate
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_test_markers
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    launch
    param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

roslint_python(
   src/open_the_door_main.py
)
#############
## Testing ##
#############
# roslint_python(
#    src/movingTherobot.py
#    src/open_the_door_main.py
# )

