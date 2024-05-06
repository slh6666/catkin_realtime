########### 版本要求
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

########### 项目名称
project(astar)

########### 添加系统库
find_package(OpenCV REQUIRED)

message(STATUS "Opnecv ;ibrary status: ")
message(STATUS "> version: ${OpenCV_VERSION} ")
message(STATUS "libraries: ${OpenCV_LIBS} ")
message(STATUS "> include: ${OpenCV_INCLUDE_DIRS}  ")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        std_msgs
        )

catkin_package()

########### 添加头文件
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
add_executable(${PROJECT_NAME} src/main.cpp src/Astar.cpp src/OccMapTransform.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})