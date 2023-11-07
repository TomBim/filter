cmake_minimum_required(VERSION 3.20)

project(FILTER)

set(CMAKE_CXX_STANDARD 17)

add_executable(simulador main.cpp)

set(WARNING_FLAGS -Werror -Weffc++)

#target_compile_options(simulador PRIVATE WARNING_FLAGS)

add_subdirectory(consts)
add_subdirectory(filter)
add_subdirectory(simulator)

list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include/eigen3")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
target_link_libraries(simulador constsLib simLib filterLib)


install(TARGETS simulador DESTINATION ${CMAKE_SOURCE_DIR}/bin)
