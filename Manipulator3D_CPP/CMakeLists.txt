cmake_minimum_required(VERSION 3.20)
project(Manipulator3D LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ---- raylib ----
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.0
)

# raylib options
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(raylib)

add_executable(Manipulator3D
  src/main.cpp
  src/robot/RobotArm.cpp
  src/sim/Trajectory.cpp
  src/ui/Overlay.cpp
  src/render/DrawUtils.cpp
)

target_include_directories(Manipulator3D PRIVATE src)
target_link_libraries(Manipulator3D PRIVATE raylib)

# On some platforms you may need extra libs; raylib usually handles this.
