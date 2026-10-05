cmake_minimum_required(VERSION 3.16...3.31)
project(systolique LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(SYSTOLIQUE_WERROR "Treat compiler warnings in the project's sources as errors" OFF)
option(SYSTOLIQUE_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

# Warnings for the project's own sources (not for Verilator's generated code).
set(SYSTOLIQUE_WARNINGS -Wall -Wextra)
if(SYSTOLIQUE_WERROR)
  list(APPEND SYSTOLIQUE_WARNINGS -Werror)
endif()
if(SYSTOLIQUE_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

enable_testing()
set(SYSTOLIQUE_REF_DIR ${PROJECT_SOURCE_DIR}/tests/reference/gemmini_rtl)
set(SYSTOLIQUE_CONFIGS default dim4 tiled ws_tree os16)

# ---------------------------------------------------------------- core library (standard C++ only)
add_library(systolique
  src/config.cpp
  src/types.cpp)
target_include_directories(systolique PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_options(systolique PRIVATE ${SYSTOLIQUE_WARNINGS})
