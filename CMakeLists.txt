cmake_minimum_required(VERSION 3.20)

project(filter)

set(CMAKE_CXX_STANDARD 17)

add_executable(simulador main.cpp)

target_include_directories(simulador PRIVATE ${CMAKE_SOURCE_DIR}/simulator)

list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include/eigen3")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
#target_link_libraries(simulador Eigen3)


install(TARGETS simulador DESTINATION ${CMAKE_SOURCE_DIR}/bin)
