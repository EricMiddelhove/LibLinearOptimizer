cmake_minimum_required(VERSION 3.28)
project(MHP_LibLinearOptimizer)
include(FetchContent)
include(ExternalProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

ExternalProject_Add(glpk
        URL https://ftp.gnu.org/gnu/glpk/glpk-5.0.tar.gz
        SOURCE_DIR glpk-src
        CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/glpk-src/configure --disable-shared
        BUILD_COMMAND make
)

add_library(MHP_LibLinearOptimizer STATIC library.cpp
        ElectricalVehicle.h
        ElectricalVehicle.cpp)

target_link_libraries(
        MHP_LibLinearOptimizer
        PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/glpk-prefix/src/glpk-build/src/.libs/libglpk.a
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