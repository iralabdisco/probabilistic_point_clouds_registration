cmake_minimum_required (VERSION 2.8)
project(probabilistic_point_cloud_registration)

find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Ceres REQUIRED)
#find_package(GTest REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(include ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(lib${PROJECT_NAME}
  src/prob_point_cloud_registration.cc
  include/prob_point_cloud_registration/prob_point_cloud_registration_iteration.hpp
  include/prob_point_cloud_registration/weight_updater_callback.hpp
  include/prob_point_cloud_registration/prob_point_cloud_registration_params.hpp
  include/prob_point_cloud_registration/probabilistic_weights.hpp
  include/prob_point_cloud_registration/error_term.hpp
  include/prob_point_cloud_registration/utilities.hpp
  include/prob_point_cloud_registration/output_stream.hpp)

add_executable(${PROJECT_NAME} src/prob_point_cloud_registration_ex.cc)
add_executable(tester src/tester.cc)

#add_executable(${PROJECT_NAME}_test
#        test/test_main.cc
#        test/ProbabilisticWeightsTest.cc
#        test/PointCloudRegistrationTest.cc)

#target_link_libraries(${PROJECT_NAME}_test lib${PROJECT_NAME} ${GTEST_LIBRARIES} ${CERES_LIBRARIES} pthread)
target_link_libraries(${PROJECT_NAME} lib${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(tester lib${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS lib${PROJECT_NAME} DESTINATION lib)
install(FILES include/prob_point_cloud_registration/prob_point_cloud_registration_params.hpp
  include/prob_point_cloud_registration/prob_point_cloud_registration.h
  include/prob_point_cloud_registration/output_stream.hpp
  include/prob_point_cloud_registration/prob_point_cloud_registration_iteration.hpp DESTINATION include)
