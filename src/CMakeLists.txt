add_library(rrt STATIC
        ../examples/euclidean_2d_space.cpp
)

target_sources(rrt PRIVATE
    ${CMAKE_SOURCE_DIR}/include/rrt/state_space.hpp
    ${CMAKE_SOURCE_DIR}/include/rrt/planner.hpp
    ${CMAKE_SOURCE_DIR}/include/rrt/vanilla_planner.hpp
    ${CMAKE_SOURCE_DIR}/include/rrt/parallel_planner.hpp
)

target_include_directories(rrt
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
