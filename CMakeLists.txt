cmake_minimum_required(VERSION 3.20)

project(filter)

set(CMAKE_CXX_STANDARD 17)

add_executable(filter main.cpp)

target_include_directories(filter PRIVATE ${CMAKE_SOURCE_DIR}/simulator)

find_package(Eigen3 REQUIRED)
target_link_libraries(filter Eigen3)


install(TARGETS filter DESTINATION ${CMAKE_SOURCE_DIR}/bin)