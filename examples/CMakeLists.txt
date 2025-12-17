FetchContent_Declare(
        nanobench
        GIT_REPOSITORY https://github.com/martinus/nanobench.git
        GIT_TAG v4.3.11
)

FetchContent_MakeAvailable(nanobench)

add_executable(bench_vanilla
        bench/benchmark.cpp
        euclidean_2d_space.cpp
        euclidean_2d_space.hpp
)

target_link_libraries(bench_vanilla
        PRIVATE
        rrt
        nanobench
)

target_compile_features(bench_vanilla PRIVATE cxx_std_23)

add_executable(bench_parallel
        bench/benchmark.cpp
        euclidean_2d_space.cpp
        euclidean_2d_space.hpp
)

target_link_libraries(bench_parallel
        PRIVATE
        rrt
        nanobench
)

target_compile_features(bench_parallel PRIVATE cxx_std_23)

target_compile_definitions(bench_parallel
        PRIVATE
        USE_PARALLEL_PLANNER
        NUM_WORKERS=6
        RESERVE_HINT=2048
)

add_executable(bench
        bench/benchmark.cpp
        euclidean_2d_space.cpp
        euclidean_2d_space.hpp
)

target_link_libraries(bench
        PRIVATE
        rrt
        nanobench
)

target_compile_features(bench PRIVATE cxx_std_23)

# Examples
add_executable(rrt_example_vanilla
        vanilla_example.cpp
        euclidean_2d_space.cpp
        euclidean_2d_space.hpp
)

target_link_libraries(rrt_example_vanilla
        PRIVATE rrt
)

add_executable(rrt_example_parallel
        euclidean_2d_space.cpp
        euclidean_2d_space.hpp
        parallel_example.cpp
)

target_link_libraries(rrt_example_parallel
        PRIVATE rrt
)