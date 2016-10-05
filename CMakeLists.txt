cmake_minimum_required (VERSION 2.8)
project(probabilistic_point_cloud_registration)

find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/point_cloud_registration.cc
  include/point_cloud_registration/weight_updater_callback.hpp
  include/point_cloud_registration/point_cloud_registration_params.hpp
  include/point_cloud_registration/probabilistic_weights.hpp
  include/point_cloud_registration/error_term.hpp)

add_executable(point_cloud_registration src/point_cloud_registration_ex.cc)
#add_executable(icp_registration src/icp_registration.cc)
#add_executable(ndt_registration src/ndt_registration.cc)

add_executable(${PROJECT_NAME}_test
        test/test_main.cc
        test/ProbabilisticWeightsTest.cc
        test/PointCloudRegistrationTest.cc)

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${GTEST_LIBRARIES} ${CERES_LIBRARIES} pthread)
target_link_libraries(point_cloud_registration ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
