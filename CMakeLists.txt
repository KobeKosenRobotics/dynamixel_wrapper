cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  dynamixel_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_wrapper
  CATKIN_DEPENDS dynamixel_sdk roscpp rospy std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/dynamixel_wrapper.cpp
)

add_dependencies(dynamixel_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_wrapper ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS dynamixel_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## examples ##

#add_executable(position_control src/examples/position_control.cpp)
#target_link_libraries(position_control ${catkin_LIBRARIES})

add_executable(jacobi src/jacobi.cpp)
target_link_libraries(jacobi ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(read_motor_angular src/read_motor_angular.cpp)
target_link_libraries(read_motor_angular ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(open_manipulator_controller src/open_manipulator_controller.cpp)
target_link_libraries(open_manipulator_controller ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(jacobi2 src/jacobi2.cpp)
target_link_libraries(jacobi2 ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(arm_sequence src/arm_sequence.cpp)
target_link_libraries(arm_sequence ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(liner_interpolation src/liner_interpolation.cpp)
target_link_libraries(liner_interpolation ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(jacobi3 src/jacobi3.cpp)
target_link_libraries(jacobi3 ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(orientation src/orientation.cpp)
target_link_libraries(orientation ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(check_inverse src/check_inverse.cpp)
target_link_libraries(check_inverse ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(quasi_orientation src/quasi_orientation.cpp)
target_link_libraries(quasi_orientation ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(motor src/motor.cpp)
target_link_libraries(motor ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(exc_calculator src/exc_calculator.cpp)
target_link_libraries(exc_calculator ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(exc_simulator src/exc_simulator.cpp)
target_link_libraries(exc_simulator ${catkin_LIBRARIES} "/usr/include/eigen3")

add_executable(exc_motor src/exc_motor.cpp)
target_link_libraries(exc_motor ${catkin_LIBRARIES} "/usr/include/eigen3")