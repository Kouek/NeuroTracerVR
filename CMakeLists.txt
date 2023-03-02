cmake_minimum_required(VERSION 3.16)

project("NeuroTracerVR" LANGUAGES CXX C CUDA)

option(${PROJECT_NAME}_BUILD_TEST "Build all demos and tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Set a fixed path for dll copying in Windows
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_LIST_DIR}/bin")

set(THIRDPARTY_DIR "${CMAKE_CURRENT_LIST_DIR}/3rd")
set(INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")

# cfg injection
configure_file("${INCLUDE_DIR}/cmake_in" "${INCLUDE_DIR}/cmake_in.h")

# CUDA dep
## No need to find_package("CUDA" REQUIRED) any more since CMake 3.9
message(STATUS "CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES: ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")

# OpenVR dep
set(OPENVR_INC_DIR "${THIRDPARTY_DIR}/openvr/headers")
set(OPENVR_BIN_DIR "${THIRDPARTY_DIR}/openvr/bin")
#   shared libs
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
	if(CMAKE_CL_64)
		message(STATUS "Build OpenVR on Windows 64 bit, copying dll is needed")
		file(COPY "${OPENVR_BIN_DIR}/win64/openvr_api.dll" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/Debug")
		set(OPENVR_LIBS_PREFIX "Win64")
	else()
		message(STATUS "Build OpenVR on Windows 32 bit, copying dll is needed")
		file(COPY "${OPENVR_BIN_DIR}/win32/openvr_api.dll" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/Debug")
		set(OPENVR_LIBS_PREFIX "Win32")
	endif()
else()
	message(FATAL_ERRO "System out of Windows have not been tested yet.")
endif()
#   static libs
find_library(
	OPENVR_LIBS REQUIRED
	NAMES
	"openvr_api"
	PATHS
	"${THIRDPARTY_DIR}/openvr/lib"
	PATH_SUFFIXES
	${OPENVR_LIBS_PREFIX}
)

# imgui dep
aux_source_directory("${THIRDPARTY_DIR}/imgui" IMGUI_SRC)

# other dep
foreach(dep "glm" "spdlog" "glfw" "VolumeCompression")
	add_subdirectory("${THIRDPARTY_DIR}/${dep}")
endforeach()

include_directories("${THIRDPARTY_DIR}/glad/include")
include_directories(${THIRDPARTY_DIR})
include_directories(${INCLUDE_DIR})

add_subdirectory("src")
if(${${PROJECT_NAME}_BUILD_TEST})
	add_subdirectory("test")
endif()
