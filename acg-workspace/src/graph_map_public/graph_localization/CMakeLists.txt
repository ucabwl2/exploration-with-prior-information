cmake_minimum_required(VERSION 2.8.3)
project(graph_localization)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    add_definitions(-std=c++11)
endif()

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    geometry_msgs
    cmake_modules
    ndt_registration
    ndt_offline
    ndt_map
    ndt_localization
    graph_map
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
    eigen_conversions
    ndt_generic
    )
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS signals)
catkin_package(
    DEPENDS Eigen libpcl-all-dev
    CATKIN_DEPENDS  graph_map ndt_registration ndt_map roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs cmake_modules eigen_conversions pcl_ros ndt_localization ndt_offline ndt_generic
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
    src/localization_type.cpp
    src/localization_factory.cpp
    src/mcl_ndt/mcl_ndt.cpp
    src/reg_localization_type/reg_localization_type.cpp
    src/pose_queue.cpp
    )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  )

add_executable(test_localization test/test_localization.cpp )
target_link_libraries(test_localization ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

#add_executable(graph_localization_node src/graph_localization_node.cpp)
#target_link_libraries(graph_localization_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(graph_localization_node src/graph_localization_node2.cpp)
target_link_libraries(graph_localization_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})


add_executable(ndt_mcl_localization_graph src/ndt_mcl_localization.cpp)
target_link_libraries(ndt_mcl_localization_graph ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(graph_localization_offline src/graph_localization_offline.cpp)
target_link_libraries(graph_localization_offline ${PROJECT_NAME} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_INCLUDE_DIRS} ${JPEG_LIBRARIES})


install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)
