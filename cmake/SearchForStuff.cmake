include (FindPkgConfig)

# Detect the architecture
include (${project_cmake_dir}/TargetArch.cmake)
target_architecture(ARCH)
message(STATUS "Building for arch: ${ARCH}")

#################################################
# Find tinyxml2.
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(TinyXML2 REQUIRED)

################################################
# Find urdfdom parser. Logic:
#
#  1. if USE_INTERNAL_URDF is unset, try to use system installation, fallback to internal copy
#  2. if USE_INTERNAL_URDF is set to True, use the internal copy
#  3. if USE_INTERNAL_URDF is set to False, force to search system installation, fail on error

if (NOT PKG_CONFIG_FOUND)
  if (NOT DEFINED USE_INTERNAL_URDF)
    BUILD_WARNING("Couldn't find pkg-config for urdfdom, using internal copy")
    set(USE_INTERNAL_URDF true)
  elseif(NOT USE_INTERNAL_URDF)
    BUILD_ERROR("Couldn't find pkg-config for urdfdom")
  endif()
endif()

if (NOT DEFINED USE_INTERNAL_URDF OR NOT USE_INTERNAL_URDF)
  # check for urdfdom with pkg-config
  pkg_check_modules(URDF urdfdom>=1.0)

  if (NOT URDF_FOUND)
    find_package(urdfdom)
    if (urdfdom_FOUND)
      set(URDF_INCLUDE_DIRS ${urdfdom_INCLUDE_DIRS})
      # ${urdfdom_LIBRARIES} already contains absolute library filenames
      set(URDF_LIBRARY_DIRS "")
      set(URDF_LIBRARIES ${urdfdom_LIBRARIES})
    elseif (NOT DEFINED USE_INTERNAL_URDF)
      message(STATUS "Couldn't find urdfdom >= 1.0, using internal copy")
      set(USE_INTERNAL_URDF true)
    else()
      BUILD_ERROR("Couldn't find the urdfdom >= 1.0 system installation")
    endif()
  else()
    # what am I doing here? pkg-config and cmake
    set(URDF_INCLUDE_DIRS ${URDF_INCLUDEDIR})
    set(URDF_LIBRARY_DIRS ${URDF_LIBDIR})
  endif()
endif()

#################################################
# Find ign command line utility:
find_package(ignition-tools)
if (IGNITION-TOOLS_BINARY_DIRS)
  message(STATUS "Looking for ignition-tools-config.cmake - found")
else()
  BUILD_WARNING ("ignition-tools not found, for command line utilities and for \
generating aggregated SDFormat descriptions for sdformat.org, please install \
ignition-tools.")
endif()

################################################
# Find the Python interpreter for running the
# check_test_ran.py script
find_package(PythonInterp 3 QUIET)

################################################
# Find psutil python package for memory tests
find_python_module(psutil)
if(NOT PY_PSUTIL)
  BUILD_WARNING("Python psutil package not found. Memory leak tests will be skipped")
endif()

################################################
# Find Valgrind for checking memory leaks in the
# tests
find_program(VALGRIND_PROGRAM NAMES valgrind PATH ${VALGRIND_ROOT}/bin)
option(SDFORMAT_RUN_VALGRIND_TESTS "Run sdformat tests with Valgrind" FALSE)
mark_as_advanced(SDFORMAT_RUN_VALGRIND_TESTS)
if (SDFORMAT_RUN_VALGRIND_TESTS AND NOT VALGRIND_PROGRAM)
  BUILD_WARNING("valgrind not found. Memory check tests will be skipped.")
endif()

################################################
# Find ruby executable to produce xml schemas
find_program(RUBY ruby)
if (NOT RUBY)
    BUILD_ERROR ("Ruby version 1.9 is needed to build xml schemas")
else()
    message(STATUS "Found ruby executable: ${RUBY}")
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

########################################
# Find ignition cmake2
# Only for using the testing macros, not really
# being use to configure the whole project
find_package(ignition-cmake2 2.3 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

########################################
# Find ignition math
# Set a variable for generating ProjectConfig.cmake
find_package(ignition-math6 QUIET)
if (NOT ignition-math6_FOUND)
  message(STATUS "Looking for ignition-math6-config.cmake - not found")
  BUILD_ERROR ("Missing: Ignition math (libignition-math6-dev)")
else()
  set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})
  message(STATUS "Looking for ignition-math${IGN_MATH_VER}-config.cmake - found")
endif()
