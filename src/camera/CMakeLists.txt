set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    tf2_ros
    image_transport
    camera_info_manager
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gz_cam SHARED gazebo_ros_camera.cpp gazebo_ros_camera_utils.cpp)
target_include_directories(gz_cam PRIVATE 
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(gz_cam
    ${catkin_LIBRARIES}
    CameraPlugin
)