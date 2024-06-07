cmake_minimum_required(VERSION 3.28)
project(MHP_LibLinearOptimizer)
include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

add_library(MHP_LibLinearOptimizer STATIC library.cpp
        ElectricalVehicle.h
        ElectricalVehicle.cpp)

target_link_libraries(
        MHP_LibLinearOptimizer
        PRIVATE
        /usr/local/lib/libglpk.a
)

# Automatic testing
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

include(GoogleTest)

add_executable(
        lib_tests
        lib_tests.cpp
)
target_link_libraries(
        lib_tests
        GTest::gtest_main
        MHP_LibLinearOptimizer
)

gtest_discover_tests(lib_tests)