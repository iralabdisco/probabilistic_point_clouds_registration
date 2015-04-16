cmake_minimum_required (VERSION 2.8)
project(point_cloud_registration)

find_package(catkin_simple REQUIRED)
find_package(Boost COMPONENTS thread system filesystem program_options date_time REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)
find_package(Ceres REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

cs_add_library(dense_sparse_simulator src/dense_sparse_simulator.cc)
cs_add_library(ceres_residuals src/reprojectionError.cc)
cs_add_executable(point_cloud_registration_simulator src/point_cloud_registration_simulator.cc)
cs_add_executable(point_cloud_registration src/point_cloud_registration.cc)

target_link_libraries(point_cloud_registration_simulator dense_sparse_simulator ceres_residuals ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(point_cloud_registration ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

cs_install()
cs_export()
