project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/pathplanner.cpp src/pathplanner.h src/maptransformer.cpp src/maptransformer.h src/math_helper.cpp src/math_helper.h src/speed_helper.cpp src/speed_helper.h src/spline.cpp src/trajectory_helper.cpp src/trajectory_helper.h src/trajectory.cpp src/trajectory.h src/log_helper.cpp src/log_helper.h src/acceleration_profile.cpp src/acceleration_profile.h src/collision_checker.cpp src/collision_checker.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
