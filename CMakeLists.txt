cmake_minimum_required(VERSION 2.4.6)

project(lvr_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mesh_msgs
)

find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

### add OpenMP flags
find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

add_definitions("-frounding-math")
add_definitions("-std=c++0x")
add_definitions(${PCL_DEFINITIONS})
add_definitions(-D_USE_PCL_)

FIND_PATH(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

#find_package(PCL 1.0 REQUIRED)
#include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

## todo we are working on a lvr find_package
set(LVR_DIR "/home/robot/ros/muffin-dry/mapping/lvr")
set(LVR_INCLUDE_DIRS
  ${LVR_DIR}/ext/psimpl
  ${LVR_DIR}/ext/rply
  ${LVR_DIR}/ext/stann
  ${LVR_DIR}/ext
  ${LVR_DIR}/include/liblvr
)
set(LVR_LIBRARY_DIRS
  ${LVR_DIR}/lib
)
set(_LVR_LIBRARIES
  lvr
)
foreach(lib ${_LVR_LIBRARIES})
  find_library(${lib}_FOUND ${lib} HINTS ${LVR_LIBRARY_DIRS})
  if(${lib}_FOUND)
    list(APPEND LVR_LIBRARIES ${${lib}_FOUND})
  else()
    message( FATAL_ERROR "${lib} not found" )
  endif()
endforeach()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LVR_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${CGAL_LIBRARY_DIRS}
  ${OPENGL_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${LVR_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp mesh_msgs 
  INCLUDE_DIRS include
  DEPENDS PCL OPENGL GLUT CGAL Boost LVR
  LIBRARIES lvr_ros_mesh_conversions
) 

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${CGAL_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${LVR_LIBRARIES}
)


### lvr_ros_converter
add_library(lvr_ros_mesh_conversions src/lvr_ros_mesh_conversions.cpp)
target_link_libraries(lvr_ros_mesh_conversions ${LINK_LIBS})

add_dependencies(lvr_ros_mesh_conversions ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(lvr_ros_mesh_conversions ${catkin_EXPORT_TARGETS} ${PROJECT_NAME}_gencfg)

install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS lvr_ros_mesh_conversions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
