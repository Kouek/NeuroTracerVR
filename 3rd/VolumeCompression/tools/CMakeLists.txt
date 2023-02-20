

include(${PROJECT_SOURCE_DIR}/third_party/TinyTiff.cmake)

add_executable(VolumeResample VolumeResample.cpp)
target_link_libraries(VolumeResample PRIVATE TinyTIFF)

add_executable(VolumeTifToRaw VolumeTifToRaw.cpp)
target_link_libraries(VolumeTifToRaw PRIVATE TinyTIFF)

add_executable(VolumeToH264 VolumeToH264.cpp)
target_link_libraries(VolumeToH264 PRIVATE voxel_compress)

find_package(OpenMP)
add_executable(VolumeTifToH264 VolumeTifToH264.cpp)

target_link_libraries(VolumeTifToH264 PRIVATE voxel_compress OpenMP::OpenMP_CXX)


add_executable(ExtractBlockFromRawVolume ExtractBlockFromRawVolume.cpp)
