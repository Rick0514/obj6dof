set(CMAKE_CXX_STANDARD 17)

find_package(spdlog REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rosbag
    nav_msgs
    sensor_msgs
    tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(calib calib.cpp)

target_include_directories(calib PRIVATE 
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(calib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    spdlog
    yaml-cpp
)

add_executable(_test_disturb _test_disturb.cpp)
target_include_directories(_test_disturb PRIVATE 
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(_test_disturb
    ${catkin_LIBRARIES}
)
target_compile_definitions(_test_disturb PUBLIC TEST_DISTURB)

