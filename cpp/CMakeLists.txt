# AP Tuner, C++ version. Builds with MSVC, GCC and Clang (incl. llvm-mingw) on Windows, Linux and macOS.
#   cmake -G Ninja -S cpp -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build && ctest --test-dir build --output-on-failure
cmake_minimum_required(VERSION 3.21)
project(aptuner VERSION 0.2.0 LANGUAGES CXX
        DESCRIPTION "Log-based and live ArduPilot rate-PID and notch-filter tuning")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(APTUNER_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
# MinGW builds link the C++ runtime statically so the .exe runs on a PC without the toolchain
option(APTUNER_STATIC_RUNTIME "Link the C/C++ runtime statically (MinGW / MSVC)" ON)
option(APTUNER_BUILD_TESTS "Build the test runner" ON)

# common warnings / flags, applied per target through aptuner_options
add_library(aptuner_options INTERFACE)
if(MSVC)
  target_compile_options(aptuner_options INTERFACE /W4 /permissive- /utf-8 /Zc:__cplusplus /EHsc)
  target_compile_definitions(aptuner_options INTERFACE _USE_MATH_DEFINES NOMINMAX WIN32_LEAN_AND_MEAN)
  if(APTUNER_WARNINGS_AS_ERRORS)
    target_compile_options(aptuner_options INTERFACE /WX)
  endif()
  if(APTUNER_STATIC_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  endif()
else()
  target_compile_options(aptuner_options INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(APTUNER_WARNINGS_AS_ERRORS)
    target_compile_options(aptuner_options INTERFACE -Werror)
    # GCC reports "maybe uninitialized" and "stringop-overflow" for code it cannot fully see through (std::optional,
    # vector fills); it changes between GCC versions, so those stay warnings, not errors, on GCC
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(aptuner_options INTERFACE -Wno-error=maybe-uninitialized -Wno-error=stringop-overflow)
    endif()
  endif()
  if(WIN32)
    target_compile_definitions(aptuner_options INTERFACE NOMINMAX WIN32_LEAN_AND_MEAN)
    if(APTUNER_STATIC_RUNTIME)
      target_link_options(aptuner_options INTERFACE -static)
    endif()
  endif()
endif()

configure_file(core/include/aptuner/version.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/generated/aptuner/version.hpp @ONLY)

add_subdirectory(core)
add_subdirectory(io)
add_subdirectory(cli)
add_subdirectory(app)

if(APTUNER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
