cmake_minimum_required(VERSION 3.0.2)
project(obj_with_6dof)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    geometry_msgs
    nav_msgs
)

find_package(gazebo REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds)

add_library(SimMotion SHARED src/SimMotion.cc)
target_include_directories(SimMotion PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(SimMotion 
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(SimP3D SHARED src/SimP3D.cc)
target_include_directories(SimP3D PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(SimP3D 
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
)