cmake_minimum_required(VERSION 2.8.3)
project(mavros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  pluginlib
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  geographic_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  angles
  libmavconn
  rosconsole_bridge
  eigen_conversions
  mavros_msgs

##下面是自己修改的消息
  serial
  message_generation

)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

## Find GeographicLib
find_package(GeographicLib REQUIRED)

## Check if the datasets are installed
include(CheckGeographicLibDatasets)

include(EnableCXX11)
include(MavrosMavlink)

# detect if sensor_msgs has BatteryState.msg
# http://answers.ros.org/question/223769/how-to-check-that-message-exists-with-catkin-for-conditional-compilation-sensor_msgsbatterystate/
list(FIND sensor_msgs_MESSAGE_FILES "msg/BatteryState.msg" BATTERY_STATE_MSG_IDX)
if(${BATTERY_STATE_MSG_IDX} GREATER -1)
  add_definitions(
    -DHAVE_SENSOR_MSGS_BATTERYSTATE_MSG
  )
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

add_message_files(
  FILES
  Data_Drone_State.msg
  Data_Throw_target.msg
  Key.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  #nav_msgs
  std_msgs
  sensor_msgs
)










###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavros
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater pluginlib roscpp sensor_msgs nav_msgs geographic_msgs std_msgs tf2_ros geometry_msgs libmavconn message_runtime eigen_conversions mavros_msgs
  DEPENDS Boost EIGEN3 GeographicLib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(mavros
  src/lib/uas_data.cpp
  src/lib/uas_stringify.cpp
  src/lib/uas_timesync.cpp
  src/lib/enum_to_string.cpp
  src/lib/enum_sensor_orientation.cpp
  src/lib/ftf_frame_conversions.cpp
  src/lib/ftf_quaternion_utils.cpp
  src/lib/mavros.cpp
  src/lib/mavlink_diag.cpp
  src/lib/rosconsole_bridge.cpp
)
target_link_libraries(mavros
  atomic
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_library(mavros_plugins
  src/plugins/dummy.cpp
  src/plugins/sys_status.cpp
  src/plugins/sys_time.cpp
  src/plugins/imu.cpp
  src/plugins/param.cpp
  src/plugins/waypoint.cpp
  src/plugins/rc_io.cpp
  src/plugins/command.cpp
  src/plugins/local_position.cpp
  src/plugins/global_position.cpp
  src/plugins/setpoint_position.cpp
  src/plugins/safety_area.cpp
  src/plugins/3dr_radio.cpp
  src/plugins/setpoint_velocity.cpp
  src/plugins/setpoint_accel.cpp
  src/plugins/setpoint_attitude.cpp
  src/plugins/setpoint_raw.cpp
  src/plugins/vfr_hud.cpp
  src/plugins/ftp.cpp
  src/plugins/actuator_control.cpp
  src/plugins/manual_control.cpp
  src/plugins/altitude.cpp
  src/plugins/hil.cpp
  src/plugins/home_position.cpp
)
add_dependencies(mavros_plugins
  mavros
)
target_link_libraries(mavros_plugins
  mavros
  ${catkin_LIBRARIES}
)


############################### Declare a cpp executable
add_executable(visual_visiion
  src/visual_visiion.cpp
)
add_dependencies(visual_visiion
  mavros
)
target_link_libraries(visual_visiion
  mavros
  ${catkin_LIBRARIES}
)

############################### Declare a cpp executable
add_executable(catch_mission
  src/catch_mission.cpp
)
add_dependencies(catch_mission
  mavros
)
target_link_libraries(catch_mission
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(position_estimator
  src/position_estimator.cpp
)
add_dependencies(position_estimator
  mavros
)
target_link_libraries(position_estimator
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(collision_avoidance
  src/collision_avoidance.cpp
)
add_dependencies(collision_avoidance
  mavros
)
target_link_libraries(collision_avoidance
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(track
  src/track.cpp
)
add_dependencies(track
  mavros
)
target_link_libraries(track
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(track_land
  src/track_land.cpp
)
add_dependencies(track_land
  mavros
)
target_link_libraries(track_land
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(Ultra_Sonic
  src/Ultra_Sonic.cpp
)
add_dependencies(Ultra_Sonic
  mavros
)
target_link_libraries(Ultra_Sonic
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(position_control
  src/position_control.cpp
)
add_dependencies(position_control
  mavros
)
target_link_libraries(position_control
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(position_control_vins
  src/position_control_vins.cpp
)
add_dependencies(position_control_vins
  mavros
)
target_link_libraries(position_control_vins
  mavros
  ${catkin_LIBRARIES}
)
############################### Declare a cpp executable
add_executable(imu_transform
  src/imu_transform.cpp
)
add_dependencies(imu_transform
  mavros
)
target_link_libraries(imu_transform
  mavros
  ${catkin_LIBRARIES}
)

############################### Declare a cpp executable
add_executable(move
  src/move.cpp
)
add_dependencies(move
  mavros
)
target_link_libraries(move
  mavros
  ${catkin_LIBRARIES}
)

#########UNDER SSS REVISED############
############################### Declare a cpp executable
add_executable(mavros_example_1
  src/mavros_example_1.cpp
)
add_dependencies(mavros_example_1
  mavros
)
target_link_libraries(mavros_example_1
  mavros
  ${catkin_LIBRARIES}
)
########mavros_example_2
add_executable(mavros_example_2
  src/mavros_example_2.cpp
)
add_dependencies(mavros_example_2
  mavros
)
target_link_libraries(mavros_example_2
  mavros
  ${catkin_LIBRARIES}
)


##################mavros_example_3
add_executable(mavros_example_3
  src/mavros_example_3.cpp
)
add_dependencies(mavros_example_3
  mavros
)
target_link_libraries(mavros_example_3
  mavros
  ${catkin_LIBRARIES}
)


#################GPS_test################
############GPS Information Test######
add_executable(GPS_test
  src/GPS_test.cpp
)
add_dependencies(GPS_test
  mavros
)
target_link_libraries(GPS_test
  mavros
  ${catkin_LIBRARIES}
)

############GPS Information Test######
add_executable(GPS_test_time
  src/GPS_test_time.cpp
)
add_dependencies(GPS_test_time
  mavros
)
target_link_libraries(GPS_test_time
  mavros
  ${catkin_LIBRARIES}
)


############offboard_node######
add_executable(offb_node
  src/offb_node.cpp
)
add_dependencies(offb_node
  mavros
)
target_link_libraries(offb_node
  mavros
  ${catkin_LIBRARIES}
)




############test_cv######
add_executable(test_cv
  src/test_cv.cpp
)
find_package(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_dependencies(test_cv
  mavros
)
target_link_libraries(test_cv
  mavros
  ${catkin_LIBRARIES}  ${OpenCV_LIBS}
)



############GPS_test_write######
add_executable(GPS_test_write
  src/GPS_test_write.cpp
)
add_dependencies(GPS_test_write
  mavros
)
target_link_libraries(GPS_test_write
  mavros
  ${catkin_LIBRARIES}
)


############take_off######
add_executable(take_off
  src/take_off.cpp
)
add_dependencies(take_off
  mavros
)
target_link_libraries(take_off
  mavros
  ${catkin_LIBRARIES}
)


############mission######
add_executable(mission
  src/mission.cpp
)
add_dependencies(mission
  mavros
)
target_link_libraries(mission
  mavros
  ${catkin_LIBRARIES}
)




############mission_add######
add_executable(mission_add
  src/mission_add.cpp
)
add_dependencies(mission_add
  mavros
)
target_link_libraries(mission_add
  mavros
  ${catkin_LIBRARIES}
)

############mission_add_2######
add_executable(mission_add_2
  src/mission_add_2.cpp
)
add_dependencies(mission_add_2
  mavros
)
target_link_libraries(mission_add_2
  mavros
  ${catkin_LIBRARIES}
)


############zhenchaji######
add_executable(zhenchaji
  src/zhenchaji.cpp
)
add_dependencies(zhenchaji
  mavros
)
target_link_libraries(zhenchaji
  mavros
  ${catkin_LIBRARIES}
)


############zhenchaji_goup######
add_executable(zhenchaji_goup
  src/zhenchaji_goup.cpp
)
add_dependencies(zhenchaji_goup
  mavros
)
target_link_libraries(zhenchaji_goup
  mavros
  ${catkin_LIBRARIES}
)



############toudan######
add_executable(toudan
  src/toudan.cpp
)
add_dependencies(toudan
  mavros
)
target_link_libraries(toudan
  mavros
  ${catkin_LIBRARIES}
)




############velocity_control######
add_executable(velocity_control
  src/velocity_control.cpp
)
add_dependencies(velocity_control
  mavros
)
target_link_libraries(velocity_control
  mavros
  ${catkin_LIBRARIES}
)


############velocity_control_key######
add_executable(velocity_control_key
  src/velocity_control_key.cpp
)
add_dependencies(velocity_control_key
  mavros
)
target_link_libraries(velocity_control_key
  mavros
  ${catkin_LIBRARIES}
)


############thrower_mission######
#add_executable(thrower_mission
#  src/thrower_mission.cpp
#)
#add_dependencies(thrower_mission
#  mavros
#)
#target_link_libraries(thrower_mission
#  mavros
#  ${catkin_LIBRARIES}
#)



############thrower_serial######
add_executable(thrower_serial
  src/thrower_serial.cpp
)
add_dependencies(thrower_serial
  mavros
)
target_link_libraries(thrower_serial
  mavros
  ${catkin_LIBRARIES}
)


############thrower_mission######
add_executable(thrower_mission
  src/thrower_mission.cpp
)
add_dependencies(thrower_mission
  mavros
)
target_link_libraries(thrower_mission
  mavros
  ${catkin_LIBRARIES}
)





############gps_transmitter######
add_executable(gps_transmitter
  src/gps_transmitter.cpp
)
add_dependencies(gps_transmitter
  mavros
)
target_link_libraries(gps_transmitter
  mavros
  ${catkin_LIBRARIES}
)




############accel_control######
#add_executable(accel_control
#  src/accel_control.cpp
#)
#add_dependencies(accel_control
#  mavros
#)
#target_link_libraries(accel_control
#  mavros
#  ${catkin_LIBRARIES}
#)



############pose_control######
#add_executable(pose_control
#  src/pose_control.cpp
#)
#add_dependencies(pose_control
#  mavros
#)
#target_link_libraries(pose_control
#  mavros
#  ${catkin_LIBRARIES}
#)



############waypoint_pose_control######
#add_executable(waypoint_pose_control
#  src/waypoint_pose_control.cpp
#)
#add_dependencies(waypoint_pose_control
#  mavros
#)
#target_link_libraries(waypoint_pose_control
#  mavros
#  ${catkin_LIBRARIES}
#)

############velocity_monitor######
add_executable(velocity_monitor
  src/velocity_monitor.cpp
)
add_dependencies(velocity_monitor
  mavros
)
target_link_libraries(velocity_monitor
  mavros
  ${catkin_LIBRARIES}
)





############Leader_Follow_control######
add_executable(Leader_Follow_control
  src/Leader_Follow_control.cpp
)
add_dependencies(Leader_Follow_control
  mavros
)
target_link_libraries(Leader_Follow_control
  mavros
  ${catkin_LIBRARIES}
)


############toudanV2######
add_executable(toudanV2
  src/toudanV2.cpp
)
add_dependencies(toudanV2
  mavros
)
target_link_libraries(toudanV2
  mavros
  ${catkin_LIBRARIES}
)


############gps_receiver######
add_executable(gps_receiver
  src/gps_receiver.cpp
)
add_dependencies(gps_receiver
  mavros
)
target_link_libraries(gps_receiver
  mavros
  ${catkin_LIBRARIES}
)



#########ABOVE SSS REVISED############

add_executable(zhengfangxing
  src/zhengfangxing.cpp
)
add_dependencies(zhengfangxing
  mavros
)
target_link_libraries(zhengfangxing
  mavros
  ${catkin_LIBRARIES}
)

add_executable(mavros_node
  src/mavros_node.cpp
)
add_dependencies(mavros_node
  mavros
)
target_link_libraries(mavros_node
  mavros
  ${catkin_LIBRARIES}
)

add_executable(gcs_bridge
  src/gcs_bridge.cpp
)
target_link_libraries(gcs_bridge
  mavros
  ${catkin_LIBRARIES}
)




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
catkin_install_python(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS mavros mavros_plugins mavros_node gcs_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  mavros_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(libmavros-frame-conversions-test test/test_frame_conversions.cpp)
target_link_libraries(libmavros-frame-conversions-test mavros)

catkin_add_gtest(libmavros-sensor-orientation-test test/test_sensor_orientation.cpp)
target_link_libraries(libmavros-sensor-orientation-test mavros)

catkin_add_gtest(libmavros-quaternion-utils-test test/test_quaternion_utils.cpp)
target_link_libraries(libmavros-quaternion-utils-test mavros)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

endif()

# vim: ts=2 sw=2 et:
