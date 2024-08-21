cmake_minimum_required(VERSION 2.8.3)
project(sas_robot_driver_denso)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Werror=return-type)
add_definitions(-D_USE_LINUX_API)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sas_common
    sas_clock
    sas_robot_driver
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sas_robot_driver_denso
    CATKIN_DEPENDS roscpp rospy sas_common sas_clock sas_robot_driver
    # DEPENDS dqrobotics
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    src/
    src/bcap
    src/bcap/bCAPClient
    src/modules
    ${catkin_INCLUDE_DIRS}
    )

add_library(robot_mutex
    src/modules/robot_mutex.cpp
        test/test_robot_mutex.cpp
)

target_link_libraries(robot_mutex
    -lpthread -lrt
)


#### test ####
add_executable(test_robot_mutex
    test/test_robot_mutex.cpp
)
target_link_libraries(test_robot_mutex
    robot_mutex
    -lpthread
)
install(TARGETS test_robot_mutex
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_library(sas_robot_driver_denso
    src/sas_driver_bcap.cpp
    src/sas_robot_driver_denso.cpp

    src/bcap/bCAPClient/bcap_client.c

    src/bcap/bcap_common.c
    src/bcap/dn_com.c
    src/bcap/dn_common.c
    src/bcap/dn_device.c
    src/bcap/dn_tcp.c
    src/bcap/dn_udp.c
    src/bcap/dn_thread.c
    src/bcap/dn_socket.c
    )

add_executable(sas_robot_driver_denso_node
    src/sas_robot_driver_denso_node.cpp)

add_dependencies(sas_robot_driver_denso ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sas_robot_driver_denso
    dqrobotics
    robot_mutex
    ${catkin_LIBRARIES}
    )

target_link_libraries(sas_robot_driver_denso_node
    sas_robot_driver_denso
    ${catkin_LIBRARIES}
    )
    
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS sas_robot_driver_denso_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
