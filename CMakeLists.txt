cmake_minimum_required(VERSION 2.8)
project(conveyor_manipulation_planner)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(
    catkin
    REQUIRED
    COMPONENTS
    	sbpl_collision_checking
        sbpl_kdl_robot_model
        moveit_core
        moveit_ros_planning_interface)

find_package(smpl REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(fcl REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem serialization)


catkin_package()

add_definitions(-DSV_PACKAGE_NAME="conveyor_manipulation_planner")


include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

add_library(
    conveyor_manipulation_planner
    src/conveyor_manip_checker.cpp
    src/conveyor_kdl_robot_model.cpp
    src/conveyor_manip_lattice.cpp
    src/conveyor_manip_lattice_action_space.cpp
    src/conveyor_manip_lattice_egraph.cpp
    src/conveyor_manip_heuristic.cpp
    src/conveyor_object_model.cpp
    src/conveyor_object_lattice.cpp
    #src/conveyor_planner_interface.cpp
    src/conveyor_planner.cpp
    src/hkey_dijkstra.cpp)
target_link_libraries(conveyor_manipulation_planner ${catkin_LIBRARIES})
target_link_libraries(conveyor_manipulation_planner ${orocos_kdl_LIBRARIES})
target_link_libraries(conveyor_manipulation_planner smpl::smpl)
target_link_libraries(conveyor_manipulation_planner fcl)
target_link_libraries(conveyor_manipulation_planner ${Boost_LIBRARIES})

#add_executable(
#    callPlannerConveyor
#    src/call_planner_conveyor.cpp
#    src/collision_space_scene.cpp
#    src/conveyor_kdl_robot_model.cpp
#    src/conveyor_planner_interface.cpp)
#target_link_libraries(callPlannerConveyor ${catkin_LIBRARIES} smpl::smpl conveyor_manipulation_planner)

#install(
#    TARGETS callPlannerConveyor
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(
    callPlannerConveyorZTP
    src/call_planner_conveyor_ztp.cpp
    src/collision_space_scene.cpp
    src/conveyor_kdl_robot_model.cpp)
target_link_libraries(callPlannerConveyorZTP ${catkin_LIBRARIES} smpl::smpl conveyor_manipulation_planner)

install(
    TARGETS callPlannerConveyorZTP
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})