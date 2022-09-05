cmake_minimum_required(VERSION 3.7)

project(RacecaR)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

add_executable(RacecaR RacecaR.cpp)
target_link_libraries(RacecaR ${SDL2_LIBRARIES})