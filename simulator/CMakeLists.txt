add_library(simLib STATIC
    cmd_signal.cpp
    controller.cpp
    perf_measurers.cpp
    robot.cpp
    simulator.cpp
    some_functions.cpp
)
target_link_libraries(simLib constsLib filterLib)