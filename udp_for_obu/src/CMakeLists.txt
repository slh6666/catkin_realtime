cmake_minimum_required(VERSION 3.0.2)
project(simple_udp)


#find_package(catkin REQUIRED COMPONENTS
  #roscpp
  #std_msgs
#)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_udp
#  CATKIN_DEPENDS std_msgs
#)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/simple_udp.cpp
  src/main.cpp
)
#add_executable(main2
  #src/simple_udp.cpp
 # src/main.cpp
#)


target_link_libraries(${PROJECT_NAME}
  pthread
  ${catkin_LIBRARIES}
)
