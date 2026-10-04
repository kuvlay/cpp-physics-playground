cmake_minimum_required(VERSION 3.24)
project(CollisionPlayground LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build raylib examples" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(collision_playground src/main.cpp src/physics.cpp)
target_link_libraries(collision_playground PRIVATE raylib)

if(EMSCRIPTEN)
    set_target_properties(collision_playground PROPERTIES
        OUTPUT_NAME "index" SUFFIX ".html"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/site")
    target_link_options(collision_playground PRIVATE
        "--shell-file=${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sGL_ENABLE_GET_PROC_ADDRESS=1"
        "-sASSERTIONS=1")
    set_property(TARGET collision_playground APPEND PROPERTY
        LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html")
endif()
