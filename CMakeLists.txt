cmake_minimum_required(VERSION 2.4.6)

project(lvr_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mesh_msgs
)

find_package(lvr REQUIRED)

add_definitions(${lvr_DEFINITIONS})

find_path(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

#find_package(PCL 1.0 REQUIRED)
#include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${lvr_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs mesh_msgs 
  INCLUDE_DIRS include
  DEPENDS lvr
  LIBRARIES lvr_ros_conversions
) 

add_library(lvr_ros_conversions
  src/conversions.cpp
  src/colors.cpp)
target_link_libraries(lvr_ros_conversions
  ${catkin_LIBRARIES}
  ${lvr_LIBRARIES}
)

add_executable(lvr_ros_reconstruction
  src/reconstruction.cpp
  src/conversions.cpp
  src/colors.cpp)

target_link_libraries(lvr_ros_reconstruction
  ${catkin_LIBRARIES}
  ${lvr_LIBRARIES}
)

add_dependencies(lvr_ros_reconstruction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(lvr_ros_reconstruction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(lvr_ros_conversions ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(lvr_ros_conversions ${catkin_EXPORT_TARGETS} ${PROJECT_NAME}_gencfg)

install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS lvr_ros_conversions lvr_ros_reconstruction
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
