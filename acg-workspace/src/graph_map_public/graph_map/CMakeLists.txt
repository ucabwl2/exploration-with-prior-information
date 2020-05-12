cmake_minimum_required(VERSION 2.8.3)
project(graph_map)

set (CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE release)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
add_definitions(-std=c++11)
endif()
find_package(octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    geometry_msgs
#    cmake_modules
    ndt_registration
    ndt_map
	velodyne_pointcloud
#    ndt_mcl
    ndt_localization
    ndt_rviz
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    ndt_map
    tf_conversions
    tf
    genmsg
    tf_conversions
    roslz4
    )
#find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
add_message_files(
  FILES
  GraphMapMsg.msg
  FactorMsg.msg
  NodeMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs ndt_map
)




catkin_package(
#DEPENDS Eigen libpcl-all-dev gnuplot
#   CATKIN_DEPENDS ndt_registration ndt_map roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs cmake_modules eigen_conversions pcl_ros
    DEPENDS OCTOMAP EIGEN PCL
    CATKIN_DEPENDS ndt_registration ndt_map roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs eigen_conversions pcl_ros ndt_rviz velodyne_pointcloud ndt_localization
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

include_directories(${OCTOMAP_INCLUDE_DIRS})


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



set (${PROJECT_NAME}_LIB_SRCS
    src/graph_map/voxelgrid.cpp
    src/graph_map/graph_map.cpp
    src/graph_map/graph_map_navigator.cpp
    src/graph_map/map_node.cpp
    src/graph_map/map_type.cpp
    src/graph_map_fuser.cpp
    src/graph_map/reg_type.cpp
    src/ndt/ndt_map_type.cpp
    src/ndt/ndtd2d_reg_type.cpp
    src/ndt/ndt_map_param.cpp
    src/graphfactory.cpp
    src/visualization/graph_plot.cpp
    src/lidarUtils/lidar_utilities.cpp
    src/template/template_map_type.cpp
    src/template/template_reg_type.cpp
    src/graph_map/factor.cpp
    src/ndt_dl/ndtdl_map_type.cpp
    src/ndt_dl/ndtdl_reg_type.cpp
    src/ndt_dl/ndtdl_map_param.cpp
    src/narf/narf_map_type.cpp
    src/narf/narf_reg_type.cpp
    src/octomap/octomap_map_type.cpp
    )

#link_directories(${OCTOMAP_LIBRARY_DIRS})
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
add_executable(test_graph test/test_graph.cpp )
add_executable(predict src/test_integration.cpp )
#add_executable(test_particle_filter test/test_particle_filter.cpp )
add_executable(test_serialization test/test_serialization.cpp )
add_executable(show_map src/show_map.cpp )
add_executable(graph_map_fuser_node src/graph_map_fuser_node.cpp )
#<<<<<<< HEAD
#add_executable(graph_map_publisher src/graph_map_publisher.cpp )
#add_executable(graph_map_publisher_localization src/graph_map_publisher_localization.cpp )
add_executable(ndt_map_publisher_mcl_test src/ndt_map_publisher_mcl_test.cpp)
add_executable(test_simple test/test_simple.cpp)
#=======
#add_executable(test_grid test/test_grid.cpp )
#>>>>>>> port-kinetic

target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES} pcl_common pcl_octree)

##
#
target_link_libraries(show_map ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )

target_link_libraries(test_graph ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

target_link_libraries(predict ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )

#<<<<<<< HEAD
#target_link_libraries(test_particle_filter ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
#target_link_libraries(test_serialization ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
#target_link_libraries(graph_map_fuser_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
#target_link_libraries(graph_map_publisher ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
#target_link_libraries(graph_map_publisher_localization ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
target_link_libraries(ndt_map_publisher_mcl_test ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
target_link_libraries(test_simple ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})


#add_dependencies(graph_map_publisher graph_map_generate_messages_cpp)
#add_dependencies(graph_map_publisher_localization graph_map_generate_messages_cpp)

#target_link_libraries(test_particle_filter ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  )
target_link_libraries(test_serialization ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  )
target_link_libraries(graph_map_fuser_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  )
#target_link_libraries(test_grid ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
#>>>>>>> port-kinetic

#  ${Eigen_LIBRARIES}
#)
#target_link_libraries(executable_name $PCL_COMMON_LIBRARIES $PCL_IO_LIBRARIES)
#

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#	message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()




    install(DIRECTORY include/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      )

    install(TARGETS graph_map_fuser_node ${PROJECT_NAME}
	    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	   )

