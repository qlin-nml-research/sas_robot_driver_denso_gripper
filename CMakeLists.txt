cmake_minimum_required(VERSION 2.8.3)
project(rosilo_robot_driver_denso)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

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
    rosilo_clock
    rosilo_robot_driver
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
    LIBRARIES rosilo_robot_driver_denso
    CATKIN_DEPENDS roscpp rospy rosilo_clock rosilo_robot_driver
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
    ${catkin_INCLUDE_DIRS}
    )

add_library(rosilo_robot_driver_denso
    src/rosilo_driver_bcap.cpp
    src/rosilo_robot_driver_denso.cpp

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

add_dependencies(rosilo_robot_driver_denso ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rosilo_robot_driver_denso
    ${catkin_LIBRARIES}
    )
