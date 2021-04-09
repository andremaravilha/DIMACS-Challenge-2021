cmake_minimum_required(VERSION 3.13)


# ==============================================================================
# Project info

project(DIMACS_Challenge_2021
        VERSION "0.0.1"
        DESCRIPTION "12th Implementation Challenge on Vehicle Routing Problems"
        HOMEPAGE_URL "https://github.com/andremaravilha/DIMACS-Challenge-2021")


# ==============================================================================
# C++ standard and other settings

set(CXX ON)
enable_language(CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

# If build type is not set, then use "Release"
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set MS Visual Studio compiler with static runtime libraries, if MSVS is the compiler
if(MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()


# ==============================================================================
# Paths to search for headers and libraries

include_directories(
        ${PROJECT_SOURCE_DIR}/lib/)


# ==============================================================================
# Source files

set(SOURCE_FILES
        src/main.cpp)


# ==============================================================================
# Targets

add_executable(solver_cvrp ${SOURCE_FILES})
target_link_libraries(solver_cvrp)
