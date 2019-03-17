project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
    src/main.cpp
    src/CarState.cpp
    src/CarState.hpp
    src/Coordinate2D.cpp
    src/Coordinate2D.hpp
    src/Coordinates2D.cpp
    src/Coordinates2D.hpp
    src/Environment.cpp
    src/Environment.hpp
    src/Lane.cpp
    src/Lane.hpp
    src/Prediction.cpp
    src/Prediction.hpp
    src/Predictions.cpp
    src/Predictions.hpp
    src/Track.cpp
    src/Track.hpp
    src/Trajectory.cpp
    src/Trajectory.hpp
    src/Vehicle.cpp
    src/Vehicle.hpp
    src/Spline/spline.h
    )


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

link_directories(/usr/local/lib/uWS)

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
