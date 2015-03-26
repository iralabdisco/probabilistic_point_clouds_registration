cmake_minimum_required (VERSION 2.8)
project(map_merging)

find_package(catkin_simple REQUIRED)
find_package(Boost COMPONENTS thread system filesystem program_options date_time REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}))

cs_add_library(${PROJECT_NAME} src/dense_sparse_simulator.cc)
cs_add_executable(aslam_map_merging_simulator src/aslam_map_merging_simulator.cc)
cs_add_executable(aslam_map_merging src/aslam_map_merging.cc)

target_link_libraries(aslam_map_merging_simulator ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(aslam_map_merging ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

cs_install()
cs_export()
