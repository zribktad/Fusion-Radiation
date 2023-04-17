cmake_minimum_required(VERSION 3.1.2)
project(fusion_radiation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nodelet
  cmake_modules
  rad_msgs
  sensor_msgs
  mrs_msgs
  rad_utils
  mrs_lib
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  message_generation
  cv_bridge
  image_transport
  image_geometry
  gazebo_rad_msgs
  
)




add_service_files(
  FILES 
  ToggleService.srv
  ModelService.srv
  EstimationService.srv
  FusionService.srv
  )

  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(
    CATKIN_DEPENDS nodelet rospy std_msgs message_runtime rad_msgs mrs_lib rad_utils mrs_msgs sensor_msgs gazebo_rad_msgs
    DEPENDS Eigen
 )


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  )

  find_package(octomap REQUIRED)
  
 
  add_library(fusion_radiation
  src/fusion_radiation.cpp
  src/point.cpp
  src/cone.cpp
  src/point_visualizer.cpp
  src/sample_generator.cpp
  src/sample_filter.cpp
  src/KDTree.cpp
  src/image_filter.cpp
  src/fusion_test.cpp
)



  target_link_libraries(fusion_radiation
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  )

  add_dependencies(fusion_radiation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

