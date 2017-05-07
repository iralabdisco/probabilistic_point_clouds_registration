cmake_minimum_required (VERSION 2.8)
project(probabilistic_point_cloud_registration)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(include ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/point_cloud_registration.cc
  include/point_cloud_registration/point_cloud_registration_iteration.hpp
  include/point_cloud_registration/weight_updater_callback.hpp
  include/point_cloud_registration/point_cloud_registration_params.hpp
  include/point_cloud_registration/probabilistic_weights.hpp
  include/point_cloud_registration/error_term.hpp
  include/point_cloud_registration/utilities.hpp
  include/point_cloud_registration/output_stream.hpp)

#add_library(${PROJECT_NAME}_pso
#  include/point_cloud_registration/pso_registration.hpp
#  include/point_cloud_registration/particle.hpp
#  include/point_cloud_registration/point_cloud_registration_iteration.hpp
#  include/point_cloud_registration/weight_updater_callback.hpp
#  include/point_cloud_registration/point_cloud_registration_params.hpp
#  include/point_cloud_registration/probabilistic_weights.hpp
#  include/point_cloud_registration/error_term.hpp
#  include/point_cloud_registration/utilities.hpp)

#set_target_properties(${PROJECT_NAME}_pso PROPERTIES LINKER_LANGUAGE CXX)

add_executable(point_cloud_registration src/point_cloud_registration_ex.cc)
add_executable(pso_registration src/pso.cc)
add_executable(metric_tester src/metric_tester.cc)
add_executable(icp_registration src/icp_registration.cc)
add_executable(obj_fun_analyzer src/obj_fun_analyzer.cpp)
#add_executable(ndt_registration src/ndt_registration.cc)

add_executable(${PROJECT_NAME}_test
        test/test_main.cc
        test/ProbabilisticWeightsTest.cc
        test/PointCloudRegistrationTest.cc)

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${GTEST_LIBRARIES} ${CERES_LIBRARIES} pthread)
target_link_libraries(point_cloud_registration ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pso_registration ${CERES_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(metric_tester ${PCL_LIBRARIES})
target_link_libraries(icp_registration ${PCL_LIBRARIES})
target_link_libraries(obj_fun_analyzer ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

