cmake_minimum_required(VERSION 2.8.3)
project(ltu_actor_inputprocess_camadjust)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(cfg/CamPub.cfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES actor_input
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(cam_pub src/cam_pub.cpp)
add_executable(cam_repub src/cam_repub.cpp)

add_dependencies(cam_pub ${PROJECT_NAME}_gencfg)
add_dependencies(cam_repub ${PROJECT_NAME}_gencfg)

target_link_libraries(cam_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(cam_repub ${catkin_LIBRARIES} ${OpenCV_LIBS})
