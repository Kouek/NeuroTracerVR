set(TARGET_NAME "NeuroTracerVR")

foreach(
	dir "" "gui" "renderer" "renderer/comp")
	string(CONCAT dir "${CMAKE_CURRENT_LIST_DIR}/" ${dir})
	file(GLOB CPP "${dir}/*.cpp")
	file(GLOB HPP "${dir}/*.hpp")
	file(GLOB CU "${dir}/*.cu")
	list(APPEND SRCS ${CPP} ${HPP} ${CU})
	message(STATUS "Search in folder ${dir},
		found source files: ${CPP} ${HPP} ${CU}")
endforeach()

foreach(
	src
	"Volume/volume_impl.cpp"
    "Volume/block_loader.cpp"
    "Volume/volume_cache_impl.cpp"
	"IO/reader_impl.cpp"
    "Common/cuda_context.cpp"
	"Common/image.cpp"
    "Common/library.cpp"
    "Common/library_reposity.cpp"
	"Common/plugin_loader.cpp"
)
	list(APPEND SRCS "${CMAKE_CURRENT_LIST_DIR}/renderer/comp/vs_core/${src}")
endforeach()

add_executable(
	${TARGET_NAME}
	${SRCS}
	"${THIRDPARTY_DIR}/glad/src/glad.c"
)
set_target_properties(
	${TARGET_NAME}
	PROPERTIES
	CUDA_SEPARABLE_COMPILATION ON
	CUDA_RESOLVE_DEVICE_SYMBOLS ON
)
target_compile_options(
	${TARGET_NAME}
	PRIVATE
	$<$<COMPILE_LANGUAGE:CUDA>:
		--use_fast_math
		--extended-lambda
    >
)
target_link_libraries(
	${TARGET_NAME}
	PRIVATE
	${OPENVR_LIBS}
	"glm::glm"
	"spdlog::spdlog"
	"glfw"
	"voxel_uncompress"
)
target_include_directories(
	${TARGET_NAME}
	PRIVATE
	${OPENVR_INC_DIR}
	${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
	"${INCLUDE_DIR}/vs_core" # VolumeSlicer header-only source file
	"${CMAKE_CURRENT_LIST_DIR}/renderer/comp/vs_core" # VolumeSlicer inner header
)
