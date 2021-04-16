cmake_minimum_required(VERSION 3.5.1)
project(lab_7_stereo)

if(MSVC)
  set(compiler_options /W4)
  add_definitions(/D_USE_MATH_DEFINES)
else ()
  set(compiler_options -Wall -Wextra -Wpedantic)
endif ()

# Add libraries.
find_package(OpenCV 4 CONFIG REQUIRED)
find_package(tek5030 CONFIG REQUIRED)

add_executable(${PROJECT_NAME}
  cv_stereo_matcher_wrap.h
  lab_7.h
  sparse_stereo_matcher.h
  stereo_calibration.h
  viewer_3d.h
  visualization.h
  #
  lab_7.cpp
  main.cpp
  sparse_stereo_matcher.cpp
  stereo_calibration.cpp
  viewer_3d.cpp
  visualization.cpp
  cv_stereo_matcher_wrap.cpp
  )
#
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${OpenCV_LIBS}
  tek5030::tek5030
  )

target_compile_options(${PROJECT_NAME} PRIVATE ${compiler_options})
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 14
  )
