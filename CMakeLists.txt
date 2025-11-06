cmake_minimum_required(VERSION 3.20)
project(admc LANGUAGES CXX)

option(ADMC_ENABLE_WARNINGS "Enable extra compiler warnings" ON)

add_library(admc_core INTERFACE)
target_include_directories(admc_core INTERFACE ${CMAKE_SOURCE_DIR}/include)
target_compile_features(admc_core INTERFACE cxx_std_20)

if (ADMC_ENABLE_WARNINGS)
    target_compile_options(admc_core INTERFACE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Wconversion>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->)
endif()

add_executable(core_tests tests/test_core.cpp)
target_link_libraries(core_tests PRIVATE admc_core)

enable_testing()
add_test(NAME core_tests COMMAND $<TARGET_FILE:core_tests>)
