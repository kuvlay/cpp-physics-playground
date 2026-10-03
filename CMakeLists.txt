cmake_minimum_required(VERSION 3.24)
project(CollisionPlayground LANGUAGES C CXX)

# raylib is written in C; our application uses standard C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build raylib examples" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(collision_playground src/main.cpp)
target_link_libraries(collision_playground PRIVATE raylib)
