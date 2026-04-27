cmake_minimum_required(VERSION 3.16)

project(VBEE)

find_package(Eigen3 REQUIRED)

add_library(VBEE STATIC
    src/observability_models/discrete_boundary.cc
    src/vbee.cc
)
target_link_libraries(VBEE PRIVATE Eigen3::Eigen)
target_include_directories(VBEE PUBLIC include)

add_executable(vbee_demo src/main.cc)
target_link_libraries(vbee_demo PRIVATE VBEE Eigen3::Eigen)