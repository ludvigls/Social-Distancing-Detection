include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(${CMAKE_BINARY_DIR}/conan_paths.cmake OPTIONAL)
include(${CMAKE_CURRENT_SOURCE_DIR}/build/conan_paths.cmake OPTIONAL)

if(DEFINED ENV{CMAKE_PREFIX_PATH})
  string(REPLACE ":" ";" CURRENT_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
  list(APPEND CMAKE_PREFIX_PATH ${CURRENT_PREFIX_PATH})
  list(REMOVE_DUPLICATES CMAKE_PREFIX_PATH)
endif()

#################################################################
### Set properties on the CMake target containing the library ###
#################################################################
macro(add_library_boilerplate _target_name)
  set(_TARGET_NAME ${_target_name})
  message(STATUS "* add_library_boilerplate for project '${_TARGET_NAME}'")

  add_library(${CMAKE_PROJECT_NAME}::${_TARGET_NAME} ALIAS ${_TARGET_NAME})

  # Assuming all header files are under 'include' (as they should be)
  target_include_directories(${_TARGET_NAME} PUBLIC         # Set include directories conditionally:
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # if proj_DIR is build directory
    $<INSTALL_INTERFACE:include>                            # if proj_DIR is install directory
    )

  set_target_properties(${_TARGET_NAME} PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_STANDARD 17
    LINKER_LANGUAGE CXX
    POSITION_INDEPENDENT_CODE ON
    SOVERSION ${PROJECT_VERSION}
    INSTALL_RPATH $ORIGIN
    )

  target_compile_features(${_TARGET_NAME}
    PUBLIC cxx_std_17
    )

  # Lowercase all installed library files
  string(TOLOWER "${_TARGET_NAME}" output_name)
  set_target_properties(${_TARGET_NAME} PROPERTIES OUTPUT_NAME ${output_name})

  target_compile_options(${_TARGET_NAME} PRIVATE
    -Wall
    -Wcast-align
    -Wcast-qual
    -Werror
    -Wextra
    -Wfloat-conversion
    -Winit-self
    -Winit-self
    -Wlogical-op
    -Wmissing-declarations
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wpedantic
    -Wpointer-arith
    -Wshadow
    -Wsuggest-override
    -Wuninitialized
    -Wunknown-pragmas
    -Wunreachable-code
    -Wunused-local-typedefs
    )

  list(APPEND project_targets ${_TARGET_NAME})
  list(REMOVE_DUPLICATES project_targets)

endmacro()

###################################################
### Configure build- and install directories.   ###
### This is where the boilerplate begins 4real! ###
### No hard coded names should be necessary.    ###
###################################################
macro(add_cmake_boilerplate)

  # 1) Set variables that will be used below.
  if (NOT DEFINED project_targets)
    set(project_targets ${PROJECT_NAME})
  endif ()

  string(TOLOWER ${CMAKE_PROJECT_NAME} package_name)
  set(config_install_dir  "share/cmake/${PROJECT_NAME}/")
  set(include_install_dir "include")
  set(namespace           "${CMAKE_PROJECT_NAME}::")
  set(project_config_in   "${CMAKE_CURRENT_LIST_DIR}/cmake/config.cmake.in")
  set(project_config      "${package_name}-config.cmake")
  set(targets_export_name "${package_name}-targets")
  set(version_config      "${package_name}-config-version.cmake")
  set(CMAKE_DEBUG_POSTFIX "d")

  # 2) Create file '<project>-config.cmake'
  # Should be used instead of the plain 'configure_file'-command.
  configure_package_config_file(
    ${project_config_in}                      # input
    ${CMAKE_BINARY_DIR}/${project_config}     # output
    INSTALL_DESTINATION ${config_install_dir} # has no visible effect, but must match
    # destination in install commands below
    NO_CHECK_REQUIRED_COMPONENTS_MACRO        # Enabled since this library has no components
    NO_SET_AND_CHECK_MACRO                    # We are not setting variables containing paths
  )

  # 3) Create file '<project>-config-version.cmake'
  write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/${version_config}
    COMPATIBILITY SameMinorVersion
  )

  # 4) Copy find_package helpers to build tree

  # 5) Generate an export file for the **build tree**
  export(
    TARGETS ${project_targets}
    NAMESPACE ${namespace}
    FILE ${CMAKE_BINARY_DIR}/${targets_export_name}.cmake
  )

  # --- INSTALL --- #
  # What comes below applies to when the 'install' target is built,
  # e.g. when you run 'make install'.

  # 6) Install the config- and version files.
  install(FILES
    ${CMAKE_BINARY_DIR}/${project_config}
    ${CMAKE_BINARY_DIR}/${version_config}
    ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
    DESTINATION ${config_install_dir}
    )

  # 7) COPY header files to installation tree
  install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
  )

  # 8) Associate the target (i.e. the library) with an export.
  # An export is a CMake entity, just like targets,
  # and is not to be confused with the exported FILE in step 5.
  install(
    TARGETS ${project_targets}
    EXPORT  ${targets_export_name} # associates the target with the named export.
    LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION  ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

  # 9) Generate and install an export file for the **installation tree**
  # Must match the export created in step 8.
  install(
    EXPORT      ${targets_export_name}
    NAMESPACE   ${namespace}
    DESTINATION ${config_install_dir}
  )
endmacro()
