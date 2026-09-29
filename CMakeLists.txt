# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5...3.31)

# App descriptor, telemetry and OTA comparisons share one version source.
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/main/include/config.h" RTK_FW_VERSION_LINE
     REGEX "^#define FW_VERSION \"[^\"]+\"")
string(REGEX REPLACE "^#define FW_VERSION \"([^\"]+)\"$" "\\1"
       PROJECT_VER "${RTK_FW_VERSION_LINE}")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rtkdata-fw)

#set(CMAKE_BUILD_TYPE Debug)

#add_compile_options(-g -O0)
