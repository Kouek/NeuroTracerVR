message(STATUS ${CUDA_INCLUDE_DIRS})
target_include_directories(voxel_compress PUBLIC ${CUDA_INCLUDE_DIRS})
message(STATUS ${CUDA_CUDA_LIBRARY})
target_link_libraries(voxel_compress ${CUDA_CUDA_LIBRARY})
set_target_properties(
        voxel_compress
        PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_ARCHITECTURES 75
)