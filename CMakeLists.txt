cmake_minimum_required(VERSION 2.6)
project(pendballoon)

set(CMAKE_BUILD_TYPE Debug)

set(pendballoon_VERSION_MAJOR 0)
set(pendballoon_VERSION_MINOR 0)

add_executable(pendballoon src/main.cpp)

# set local module dir
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

# load sfml
find_package(SFML 2 REQUIRED system window graphics network audio)
if(SFML_FOUND)
    include_directories(${SFML_INCLUDE_DIR})
    target_link_libraries(pendballoon ${SFML_LIBRARIES})
endif()

# load chipmunk
find_package(Chipmunk REQUIRED)
if(CHIPMUNK_FOUND)
    include_directories(${CHIPMUNK_INCLUDE_DIR})
    target_link_libraries(pendballoon ${CHIPMUNK_LIBRARIES})
endif()
