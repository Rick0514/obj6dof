cmake_minimum_required(VERSION 3.0.2)
project(obj6dof)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
)

find_package(gazebo REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})

find_package(PCL REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds)

add_subdirectory(src/livox)

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

add_executable(sync src/sync.cpp)
target_include_directories(sync PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(sync
    ${catkin_LIBRARIES}
)

add_executable(mk_gt_map src/mk_gt_map.cpp)
target_include_directories(mk_gt_map PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(mk_gt_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)