cmake_minimum_required(VERSION 3.20)
project(admc LANGUAGES CXX)

option(ADMC_ENABLE_WARNINGS "Enable extra compiler warnings" ON)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_library(admc_core INTERFACE)
target_include_directories(admc_core INTERFACE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(admc_core INTERFACE cxx_std_20)

if (ADMC_ENABLE_WARNINGS)
    target_compile_options(admc_core INTERFACE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Wconversion>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->)
endif()

add_executable(core_tests tests/test_math.cpp)
target_link_libraries(core_tests PRIVATE admc_core)

add_executable(solver_policy_tests tests/test_solver_policies.cpp)
target_link_libraries(solver_policy_tests PRIVATE admc_core)

add_executable(simple_pgs_tests tests/test_simple_pgs.cpp)
target_link_libraries(simple_pgs_tests PRIVATE admc_core)

add_executable(baseline_tests tests/test_baseline_solver.cpp)
target_link_libraries(baseline_tests PRIVATE admc_core)

add_executable(tile_warmstart_tests tests/test_tile_warmstart.cpp)
target_link_libraries(tile_warmstart_tests PRIVATE admc_core)

add_executable(builder_determinism_tests tests/test_builder_determinism.cpp)
target_link_libraries(builder_determinism_tests PRIVATE admc_core)

add_executable(simple_bench apps/simple_bench.cpp)
target_link_libraries(simple_bench PRIVATE admc_core)

enable_testing()
add_test(NAME core_tests COMMAND $<TARGET_FILE:core_tests>)
add_test(NAME solver_policy_tests COMMAND $<TARGET_FILE:solver_policy_tests>)
add_test(NAME simple_pgs_tests COMMAND $<TARGET_FILE:simple_pgs_tests>)
add_test(NAME baseline_tests COMMAND $<TARGET_FILE:baseline_tests>)
add_test(NAME tile_warmstart_tests COMMAND $<TARGET_FILE:tile_warmstart_tests>)
add_test(NAME builder_determinism_tests COMMAND $<TARGET_FILE:builder_determinism_tests>)
add_test(
    NAME bench_smoke
    COMMAND Python3::Interpreter ${CMAKE_SOURCE_DIR}/tests/bench_smoke_check.py
            $<TARGET_FILE:simple_bench> ${CMAKE_BINARY_DIR}/bench_smoke.csv
)
