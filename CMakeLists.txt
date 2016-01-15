cmake_minimum_required (VERSION 2.8)
project(point_cloud_registration)

#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg geometry_msgs)
find_package(catkin_simple REQUIRED)
find_package(VTK REQUIRED)
# find_package(Boost COMPONENTS thread system filesystem program_options date_time REQUIRED)
#find_package(PCL 1.7 REQUIRED)
#find_package(VTK REQUIRED)
#find_package(Ceres REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#include_directories(${CMAKE_CURRENT_BINARY_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#cs_add_library(dense_sparse_simulator src/dense_sparse_simulator.cc)
cs_add_library(${PROJECT_NAME}
  src/point_cloud_registration.cc
  include/point_cloud_registration/weight_updater_callback.hpp
  include/point_cloud_registration/point_cloud_registration_params.hpp
  include/point_cloud_registration/probabilistic_weights.hpp
  include/point_cloud_registration/error_term.hpp)

catkin_add_gtest(${PROJECT_NAME}_test
	test/test_main.cc
        test/ProbabilisticWeightsTest.cc
        test/PointCloudRegistrationTest.cc)

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#cs_add_executable(point_cloud_registration_simulator src/point_cloud_registration_simulator.cc)
#cs_add_executable(point_cloud_registration_ros src/point_cloud_registration_ros.cc)
#cs_add_executable(registration_visualizer src/registration_visualizer.cc)
#cs_add_executable(icp_registration src/icp_registration.cc)

#target_link_libraries(point_cloud_registration_ros ${PROJECT_NAME})

#target_link_libraries(point_cloud_registration_simulator dense_sparse_simulator ${PROJECT_NAME})

cs_install()
cs_export()
