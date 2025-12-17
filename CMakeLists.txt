cmake_minimum_required(VERSION 3.29)
project(rrt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

add_subdirectory(src)

option(BUILD_EXAMPLES "Build examples" ON)

if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()


install(DIRECTORY include/ DESTINATION include)