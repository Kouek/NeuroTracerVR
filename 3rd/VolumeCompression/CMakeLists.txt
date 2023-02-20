cmake_minimum_required(VERSION 3.14)

project(VoxelCompression C CXX)
find_package(CUDA REQUIRED)

option(BUILD_TEST "whether build test" OFF)
option(BUILD_TOOL "whether build tool" ON)


find_package(CUDA REQUIRED)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CUDA_STANDARD 14)

set(PROJECT_THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
set(NVCODEC_DIR ${PROJECT_THIRD_PARTY_DIR}/nvcodec)
set(NVCODEC_INCLUDE_DIR ${NVCODEC_DIR}/include)
set(NVCODEC_LIB_DIR ${NVCODEC_DIR}/lib)


set(LibTiff_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_party/libtiff/include)
#set(LibTiff_LIBS ${PROJECT_SOURCE_DIR}/third_party/libtiff/lib/tiff.lib)
find_library(
        LibTiff_LIBS
        NAMES tif.lib tiffd.lib tiffxx.lib tiffxxd.lib
        HINTS ${PROJECT_SOURCE_DIR}/third_party/libtiff/lib
)

include_directories(include third_party)

add_subdirectory(src)

if(BUILD_TEST)
    add_subdirectory(test)
endif()

if(BUILD_TOOL)
    add_subdirectory(tools)
endif()
