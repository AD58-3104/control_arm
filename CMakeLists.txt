project(control_arm)
include(FetchContent) # If not included already

FetchContent_Declare(sciplot
  GIT_REPOSITORY https://github.com/sciplot/sciplot.git
  GIT_TAG master)

FetchContent_GetProperties(sciplot)
if(NOT sciplot_content_POPULATED)
  FetchContent_Populate(sciplot)
endif()

FetchContent_Declare(dynamixelsdk
  GIT_REPOSITORY https://github.com/ROBOTIS-GIT/DynamixelSDK.git 
  GIT_TAG 3.7.51)

FetchContent_GetProperties(dynamixelsdk)
if(NOT sciplot_content_POPULATED)
  FetchContent_Populate(dynamixelsdk)
endif()

message(STATUS "sciplot_SOURCE_DIR: ${sciplot_SOURCE_DIR}")
message(STATUS "dynamixelsdk_SOURCE_DIR: ${dynamixelsdk_SOURCE_DIR}")

message(STATUS "sciplot_content_SOURCE_DIR: ${sciplot_content_SOURCE_DIR}")
message(STATUS "dynamixelsdk_content_SOURCE_DIR: ${dynamixelsdk_content_SOURCE_DIR}")

include_directories(${sciplot_SOURCE_DIR}/sciplot ${dynamixelsdk_SOURCE_DIR}/c++/include/dynamixel_sdk)
file(GLOB_RECURSE DXL_SOURCES ${dynamixelsdk_SOURCE_DIR}/c++/src/dynamixel_sdk/*.cpp)
file(GLOB_RECURSE SCPLOT_SOURCES ${sciplot_SOURCE_DIR}/sciplot/*)

find_package(Eigen3 REQUIRED)
add_executable(test_matplotlib test_matplotlib.cpp ${SCPLOT_SOURCES})
add_executable(test_move test_move.cpp ${DXL_SOURCES} robot_model.hpp)
add_executable(reset_zero reset_zero.cpp ${DXL_SOURCES} robot_model.hpp)
add_executable(test_geome test_geome.cpp  robot_model.hpp ${SCPLOT_SOURCES})
add_executable(test_arm test_arm.cpp  robot_model.hpp robot_arm.hpp ${DXL_SOURCES} ${SCPLOT_SOURCES})
add_executable(solve_kine test_kinefunc.cpp  robot_model.hpp robot_arm.hpp ${DXL_SOURCES} ${SCPLOT_SOURCES})
add_executable(check_model check_model.cpp  robot_model.hpp robot_arm.hpp ${DXL_SOURCES} ${SCPLOT_SOURCES})

target_compile_options(solve_kine PRIVATE  -march=native -O2)
target_compile_options(test_arm PRIVATE  -march=native -O2)

list(APPEND targets test_move test_geome test_arm solve_kine reset_zero check_model)

foreach(itm IN LISTS targets)
  target_compile_features(${itm} PRIVATE cxx_std_20)
  target_compile_options(${itm} PRIVATE -fdiagnostics-color)
  target_include_directories(${itm} PRIVATE ${EIGEN3_INCLUDE_DIRS} ${dynamixelsdk_SOURCE_DIR} ${sciplot_SOURCE_DIR})
  target_link_libraries(${itm} PRIVATE Eigen3::Eigen pthread)
endforeach(itm IN LISTS ${targets})
