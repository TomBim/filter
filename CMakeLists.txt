cmake_minimum_required(VERSION 3.20)

project(FILTER)

set(CMAKE_CXX_STANDARD 17)

add_executable(simulador main.cpp)

set(WARNING_FLAGS -Werror -Weffc++)

#target_compile_options(simulador PRIVATE WARNING_FLAGS)


# check if DBG_MODE is defined
if (DEFINED DBG_MODE)
    message(WARNING "Debug mode is set to ${DBG_MODE}")
    add_compile_definitions(DBG_MODE=${DBG_MODE})
endif()

# check if FILTER_VERSION is defined
if (DEFINED FILTER_V_MAJOR)
    if (DEFINED FILTER_V_MINOR)
        message(WARNING "Filter version is set to ${FILTER_V_MAJOR}.${FILTER_V_MINOR}")
        add_compile_definitions(FILTER_V_MAJOR=${FILTER_V_MAJOR})
        add_compile_definitions(FILTER_V_MINOR=${FILTER_V_MINOR})
    else()
        message(WARNING "Filter version not defined")
    endif()
else()
    message(WARNING "Filter version not defined")
endif()

# check if the filter type is defined
if (DEFINED NO_FILTER)
    message(WARNING "NO_FILTER is set to ${NO_FILTER}")
    add_compile_definitions(NO_FILTER=${NO_FILTER})
endif()
if (DEFINED KALMAN)
    message(WARNING "KALMAN is set to ${KALMAN}")
    add_compile_definitions(KALMAN=${KALMAN})
endif()
if (DEFINED INFO)
    message(WARNING "INFO is set to ${INFO}")
    add_compile_definitions(INFO=${INFO})
endif()


# add subdirectories
add_subdirectory(consts)
add_subdirectory(filter)
add_subdirectory(simulator)

# link libs
list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include/eigen3")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
target_link_libraries(simulador constsLib simLib filterLib)

# installing xD
install(TARGETS simulador DESTINATION ${CMAKE_SOURCE_DIR}/bin)