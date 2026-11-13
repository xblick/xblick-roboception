# Roboception rc_visard stereo cameras as Durchblick streams (README.md).
#
#   cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<xblick>;<zenoh>" \
#         -DRC_GENICAM_API_DIR=<SDK>/lib/rc_genicam_api [-DCUDAToolkit_ROOT=<CUDA toolkit>]
#
# Targets: xblick-roboception (the camera node), test_rc_offline (no camera needed; ctest). Installs the library as
# the CMake package xblick_rc (xblick_rc::xblick_rc), for programs that build on the node.
cmake_minimum_required(VERSION 3.22)
project(xblick_roboception VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
# installed programs find this project's libraries and keep the directories of the libraries they link (the SDK)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

# Durchblick's 0.5 line (its CMake package accepts its own minor version only): CameraInfo, StaticTransformPublisher,
# DiagnosticsPublisher, RVL for depth, ring keys with the host; the point cloud node for the tests
find_package(xblick 0.5 REQUIRED OPTIONAL_COMPONENTS points)
# Roboception's GenICam SDK (RC_GENICAM_API_DIR=<SDK>/lib/rc_genicam_api)
find_package(RC_GENICAM_API 2.6 REQUIRED)
# its GenICam libraries (libGCBase_..., beside librc_genicam_api) are needed by librc_genicam_api, which has no RPATH of
# its own: the programs get the directory as DT_RPATH (--disable-new-dtags below), which, unlike RUNPATH, also serves
# the libraries they load, so that they run without LD_LIBRARY_PATH
get_filename_component(RC_GENICAM_API_LIBDIR "${RC_GENICAM_API_DIR}/.." ABSOLUTE)
list(APPEND CMAKE_BUILD_RPATH "${RC_GENICAM_API_LIBDIR}")
list(APPEND CMAKE_INSTALL_RPATH "${RC_GENICAM_API_LIBDIR}")

add_library(xblick_rc STATIC src/stereo.cpp src/source.cpp src/node.cpp src/controls.cpp)
target_include_directories(xblick_rc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                                            $<INSTALL_INTERFACE:include/xblick_rc>)
target_link_libraries(xblick_rc PUBLIC xblick::xblick rc_genicam_api::rc_genicam_api)
target_compile_options(xblick_rc PRIVATE -Wall -Wextra)
target_link_options(xblick_rc INTERFACE "-Wl,--disable-new-dtags")

add_executable(xblick-roboception src/main.cpp)
target_link_libraries(xblick-roboception PRIVATE xblick_rc)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_rc_offline tests/test_offline.cpp)
  target_link_libraries(test_rc_offline PRIVATE xblick_rc)
  if(xblick_points_FOUND)
    # Durchblick's point cloud node against the SDK's point cloud (it needs a GPU)
    find_program(RC_POINTS_PROGRAM xblick-points HINTS "${xblick_DIR}/../../../bin" NO_DEFAULT_PATH REQUIRED)
    target_compile_definitions(test_rc_offline PRIVATE RC_POINTS_PROGRAM="${RC_POINTS_PROGRAM}")
  endif()
  add_test(NAME offline COMMAND test_rc_offline)
endif()

install(TARGETS xblick-roboception RUNTIME DESTINATION bin)

# The CMake package xblick_rc: the node's library with its headers
install(TARGETS xblick_rc EXPORT xblick_rcTargets ARCHIVE DESTINATION lib)
install(FILES src/node.hpp src/source.hpp src/stereo.hpp src/session.hpp src/controls.hpp DESTINATION include/xblick_rc)
install(EXPORT xblick_rcTargets NAMESPACE xblick_rc:: DESTINATION lib/cmake/xblick_rc)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/xblick_rcConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/xblick_rcConfig.cmake
                              INSTALL_DESTINATION lib/cmake/xblick_rc)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/xblick_rcConfigVersion.cmake
                                 COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/xblick_rcConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/xblick_rcConfigVersion.cmake
        DESTINATION lib/cmake/xblick_rc)
