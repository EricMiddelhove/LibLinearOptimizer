# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-src"
  "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-build"
  "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-subbuild/glpk-populate-prefix"
  "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-subbuild/glpk-populate-prefix/tmp"
  "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-subbuild/glpk-populate-prefix/src/glpk-populate-stamp"
  "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-subbuild/glpk-populate-prefix/src"
  "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-subbuild/glpk-populate-prefix/src/glpk-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-subbuild/glpk-populate-prefix/src/glpk-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/ericmiddelhove/Development/MHP-LibLinearOptimizer/cmake-build-debug/_deps/glpk-subbuild/glpk-populate-prefix/src/glpk-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
