cmake_minimum_required(VERSION 2.8.3)
project(path_planning_msgs)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)
add_message_files(
  FILES
  Curve.msg
  CurvePoint.msg
  MultiPath.msg
  MultiCurve.msg
  Path.msg
  PathPoint.msg
  Obs.msg
  MultiObs.msg
  BoundedCurve.msg
  BoundedCurvePoint.msg
  MotionPlanningCurve.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs
)
