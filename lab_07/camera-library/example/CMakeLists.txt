cmake_minimum_required(VERSION 3.10)
project(example CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# uncomment if not add_subdirectory()
#find_package(tek5030 CONFIG REQUIRED)

add_executable(cv-example main-cvcam.cpp)
add_executable(dual-example main-dualcam.cpp)
add_executable(kitti-example main-kitticam.cpp)

target_link_libraries(cv-example    PRIVATE tek5030::tek5030)
target_link_libraries(dual-example  PRIVATE tek5030::tek5030)
target_link_libraries(kitti-example PRIVATE tek5030::tek5030)
