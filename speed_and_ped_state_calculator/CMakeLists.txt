cmake_minimum_required(VERSION 3.0.2)
project(speed_and_ped_state_calculator)

# add_compile_options(-std=c++11 -O3 -g -Wall)
add_compile_options(-std=c++17 -O3 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
nav_msgs
geometry_msgs

pedsim_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES speed_and_ped_state_calculator
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/speed_and_ped_state_calculator.cpp)
add_executable(${PROJECT_NAME}_node src/speed_and_ped_state_calculator_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})