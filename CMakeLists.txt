cmake_minimum_required(VERSION 3.0.2)
project(rviz_interactive_tf)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  roslint
  tf
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

set(
  ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)
roslint_cpp()
set(ROSLINT_PYTHON_OPTS "--ignore=W503" "1>&2")
roslint_python()

catkin_package(
  CATKIN_DEPENDS
  interactive_markers
  roscpp
  tf
  visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(interactive_tf src/interactive_tf.cpp)
target_link_libraries(interactive_tf ${catkin_LIBRARIES})

install(TARGETS
  interactive_tf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/ddr_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
