cmake_minimum_required(VERSION 2.8.3)
project(rviz_interactive_tf)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  roslint
  visualization_msgs
  tf
)

set(
  ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)
roslint_cpp()
roslint_python()

catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(interactive_tf src/interactive_tf)
target_link_libraries(interactive_tf ${catkin_LIBRARIES})
