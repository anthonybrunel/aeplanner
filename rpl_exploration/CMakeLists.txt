cmake_minimum_required(VERSION 2.8.3)
project(rpl_exploration)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)


catkin_package()



#add_action_files(DIRECTORY action FILES FlyTo.action)
#generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)



include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})



cs_add_library(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})




cs_add_executable(rpl_exploration_rotors src/rpl_exploration_rotors.cpp )
target_link_libraries(rpl_exploration_rotors ${catkin_LIBRARIES}  ${catkin_LIBRARIES} )
add_dependencies(rpl_exploration_rotors ${catkin_EXPORTED_TARGETS})
add_dependencies(rpl_exploration_rotors rpl_exploration_generate_messages_cpp)

cs_install()
cs_export()

