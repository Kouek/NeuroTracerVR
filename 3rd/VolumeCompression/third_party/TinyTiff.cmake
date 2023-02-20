option(TinyTIFF_BUILD_SHARED_LIBS "Build as shared library" OFF)
option(TinyTIFF_BUILD_STATIC_LIBS "Build as static library" ON)
option(TinyTIFF_BUILD_TESTS "Build the tests and examples" OFF)
include(FetchContent)
FetchContent_Declare(
        TinyTIFF
#        GIT_REPOSITORY https://github.com/jkriege2/TinyTIFF.git
        GIT_REPOSITORY https://github.com/wyzwzz/TinyTIFF.git
#        GIT_TAG 3.0.0.0
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(TinyTIFF)