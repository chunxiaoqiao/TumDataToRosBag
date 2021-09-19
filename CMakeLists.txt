cmake_minimum_required(VERSION 3.0.2)
project(TumDataToRosBag)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  tf
        rosbag
)


catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executableFile)
add_executable(TumDataToRosBag
        src/TumDataToRosBag.cpp
        )
add_dependencies(TumDataToRosBag  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TumDataToRosBag
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        "/usr/include/eigen3"
        )