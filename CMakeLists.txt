#set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 2.8.3)
project(cptl_ros)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cavas_msgs)

catkin_package()

catkin_package(
  CATKIN_DEPENDS
  roscpp cavas_msgs 
  INCLUDE_DIRS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${cavas_msgs_INCLUDE_DIRS})

###########
## Build ##
###########
#add_definitions(-std=c++0x -L/usr/lib/libwiringPi.so -lwiringPi -lpthread)

## Declare a cpp library
add_library(cptl_ros_driver src/cptl_ros_driver.cpp)
target_link_libraries(cptl_ros_driver ${catkin_LIBRARIES} -lbcm2835)

## Declare a cpp executable
add_executable(cptl_ros src/cptl_ros.cpp)
target_link_libraries(cptl_ros cptl_ros_driver ${catkin_LIBRARIES} -lbcm2835)

## Mark executables and/or libraries for installation
install(TARGETS cptl_ros cptl_ros_driver
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)



