set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(livox_laser_simulation SHARED livox_points_plugin.cpp livox_ode_multiray_shape.cpp)
target_include_directories(livox_laser_simulation PRIVATE 
    ${PCL_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(livox_laser_simulation
    ${catkin_LIBRARIES}
    RayPlugin
    libprotobuf.so.9
)
