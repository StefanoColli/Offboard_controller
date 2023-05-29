cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(helical_trajectory)

add_executable(helical_trajectory
    helical_trajectory.cpp
)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(helical_trajectory
    MAVSDK::mavsdk
    Threads::Threads
)

if(NOT MSVC)
    add_compile_options(helical_trajectory PRIVATE -Wall -Wextra)
else()
    add_compile_options(helical_trajectory PRIVATE -WX -W2)
endif()
