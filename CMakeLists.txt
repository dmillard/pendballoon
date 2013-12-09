cmake_minimum_required(VERSION 2.6)
project(pendballoon)

set(CMAKE_BUILD_TYPE Debug)

set(pendballoon_VERSION_MAJOR 0)
set(pendballoon_VERSION_MINOR 0)

add_executable(pendballoon src/pendballoon.cpp)
add_executable(pendulum src/pendulum.cpp)

# set local module dir
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

add_subdirectory(${CMAKE_SOURCE_DIR}/sfml)
include_directories(${CMAKE_SOURCE_DIR}/sfml/include)
target_link_libraries(pendballoon sfml-graphics sfml-window ${SFML_LIBRARIES})
target_link_libraries(pendulum sfml-graphics sfml-window ${SFML_LIBRARIES})

set(BUILD_DEMOS OFF CACHE BOOL "Build the demo applications")
add_subdirectory(${CMAKE_SOURCE_DIR}/chipmunk)
include_directories(${CMAKE_SOURCE_DIR}/chipmunk/include)
target_link_libraries(pendballoon chipmunk ${CHIPMUNK_LIBRARIES})
target_link_libraries(pendulum chipmunk ${CHIPMUNK_LIBRARIES})
