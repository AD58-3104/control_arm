project(control_arm)

find_package(sciplot REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(test_matplotlib test_matplotlib.cpp)
target_link_libraries(test_matplotlib PRIVATE sciplot::sciplot Eigen3::Eigen)
target_include_directories(test_matplotlib PRIVATE ${SCIPLOT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})