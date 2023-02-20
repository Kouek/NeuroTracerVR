list(
        APPEND
        VOXEL_UNCOMPRESS_SRC
        voxel_uncompress.cpp
        nvdec/NvDecoder.cpp
)
cuda_add_library(voxel_uncompress STATIC ${VOXEL_UNCOMPRESS_SRC})

target_include_directories(voxel_uncompress PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${NVCODEC_INCLUDE_DIR}
        ${CUDA_INCLUDE_DIRS})
if(WIN32)
    set(NVDECODE_LIBS ${NVCODEC_LIB_DIR}/x64/nvcuvid.lib)
else()
    set(NVDECODE_LIBS
            ${NVCODEC_LIB_DIR}/linux/stubs/x86_64/libnvcuvid.so
#            ${NVCODEC_LIB_DIR}/linux/stubs/x86_64/libnvcuvid-encode.so
            )
endif()
target_link_libraries(voxel_uncompress ${CUDA_CUDA_LIBRARY} ${NVDECODE_LIBS})
set_target_properties(
        voxel_uncompress
        PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_ARCHITECTURES 75
)
