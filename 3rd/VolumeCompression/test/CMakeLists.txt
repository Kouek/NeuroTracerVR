include(${PROJECT_SOURCE_DIR}/third_party/TinyTiff.cmake)

add_executable(test_compress test_compress.cpp)
target_link_libraries(test_compress PUBLIC voxel_compress)

add_executable(test_uncompress test_uncompress.cpp)
target_link_libraries(test_uncompress PUBLIC voxel_uncompress voxel_compress)


add_executable(test_struct_size test_struct_size.cpp)

add_executable(test_json test_json.cpp)
target_link_libraries(test_json PRIVATE TinyTIFF)


add_executable(test_raw2tif test_raw2tif.cpp)
target_link_libraries(test_raw2tif PRIVATE TinyTIFF)

add_executable(test_tif2raw test_tif2raw.cpp)
target_link_libraries(test_tif2raw PRIVATE TinyTIFF)

add_executable(test_raw_diff test_raw_diff.cpp)

add_executable(test_read_tif_from_raw test_read_tif_from_raw.cpp)
target_link_libraries(test_read_tif_from_raw PRIVATE TinyTIFF)

add_executable(test_merge_tif test_merge_tif.cpp)
target_link_libraries(test_merge_tif PRIVATE TinyTIFF)


add_executable(test_block_volume_cmp test_block_volume_cmp.cpp)
target_include_directories(test_block_volume_cmp PUBLIC ${VolumeReader_INCLUDE_DIR})
target_link_libraries(test_block_volume_cmp PRIVATE voxel_compress VolumeReader)

add_executable(test_tif2h264 test_tif2h264.cpp)
target_include_directories(test_tif2h264 PRIVATE )
target_link_libraries(test_tif2h264 PRIVATE TinyTIFF)

add_executable(test_libtiff test_libtiff.cpp)
target_include_directories(test_libtiff PRIVATE ${LibTiff_INCLUDE_DIR})
target_link_libraries(test_libtiff PRIVATE ${LibTiff_LIBS})

add_executable(test_tif_same test_tif_same.cpp)
target_link_libraries(test_tif_same PRIVATE voxel_compress)

add_executable(test_extract_block_from_h264 test_extract_block_from_h264.cpp)
target_link_libraries(test_extract_block_from_h264 PRIVATE voxel_uncompress)