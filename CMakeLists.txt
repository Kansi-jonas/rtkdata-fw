# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5...3.31)

# Board / image selection. Default: the OnoLink Wi-Fi fleet (1.x), exactly as
# before. -DRTK_BOARD=k_evk_v4 builds the Ethernet board (2.x): its own project
# name (update.c refuses an OTA image whose project name differs), its own
# version define, and its own sdkconfig INSIDE the build dir, so it never
# rewrites the Wi-Fi build's sdkconfig. Use a separate build dir:
#   idf.py -B build_eth -DRTK_BOARD=k_evk_v4 build
if(NOT DEFINED RTK_BOARD)
    set(RTK_BOARD "onolink")
endif()
if(RTK_BOARD STREQUAL "k_evk_v4")
    set(RTK_VERSION_DEFINE "FW_VERSION_ETH")
    set(RTK_PROJECT_NAME "rtkdata-fw-eth")
    set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.k_evk_v4")
    set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
elseif(RTK_BOARD STREQUAL "k_evk_v41")
    # Revision after schematic K_EVK_V4.0_SCH_1013: REF_CLK In oscillator,
    # GNSS reset on GPIO13, Wi-Fi LED on GPIO32. Its own project name and OTA
    # channel, so a V4.0 image (other pin map) can never be installed on it.
    #   idf.py -B build_eth41 -DRTK_BOARD=k_evk_v41 build
    set(RTK_VERSION_DEFINE "FW_VERSION_ETH41")
    set(RTK_PROJECT_NAME "rtkdata-fw-eth41")
    set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults;${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.k_evk_v41")
    set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
elseif(RTK_BOARD STREQUAL "onolink")
    set(RTK_VERSION_DEFINE "FW_VERSION")
    set(RTK_PROJECT_NAME "rtkdata-fw")
else()
    message(FATAL_ERROR "unknown RTK_BOARD '${RTK_BOARD}' (onolink | k_evk_v4 | k_evk_v41)")
endif()

# -DRTK_TEST_PAGE=1 adds the board test page (Ethernet boards only) for the
# hardware vendor. Never for an image that goes to a customer or onto OTA.
#   idf.py -B build_eth41_test -DRTK_BOARD=k_evk_v41 -DRTK_TEST_PAGE=1 build
if(RTK_TEST_PAGE)
    if(RTK_BOARD STREQUAL "onolink")
        message(FATAL_ERROR "RTK_TEST_PAGE is for the Ethernet boards only")
    endif()
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.testpage")
endif()

# App descriptor, telemetry and OTA comparisons share one version source.
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/main/include/config.h" RTK_FW_VERSION_LINE
     REGEX "^#define ${RTK_VERSION_DEFINE} \"[^\"]+\"")
string(REGEX REPLACE "^#define ${RTK_VERSION_DEFINE} \"([^\"]+)\"$" "\\1"
       PROJECT_VER "${RTK_FW_VERSION_LINE}")
# A test-page build must never pass for a customer image (review 2026-11-06,
# O9). The app descriptor says "<version>-test"; the OTA identity gate
# compares it with the manifest version, so a test .bin published to a
# channel is refused by every device. The project name stays, so a unit that
# runs a test build still takes the normal OTA to a production image.
if(RTK_TEST_PAGE)
    set(PROJECT_VER "${PROJECT_VER}-test")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
if(RTK_BOARD STREQUAL "k_evk_v4")
    # board.h cross-checks this against the Kconfig choice (stale build dir guard)
    idf_build_set_property(COMPILE_DEFINITIONS "RTK_CMAKE_BOARD_K_EVK_V4=1" APPEND)
elseif(RTK_BOARD STREQUAL "k_evk_v41")
    idf_build_set_property(COMPILE_DEFINITIONS "RTK_CMAKE_BOARD_K_EVK_V41=1" APPEND)
endif()
if(RTK_TEST_PAGE)
    # board_test.c cross-checks this against CONFIG_RTK_TEST_PAGE
    idf_build_set_property(COMPILE_DEFINITIONS "RTK_CMAKE_TEST_PAGE=1" APPEND)
endif()
project(${RTK_PROJECT_NAME})

#set(CMAKE_BUILD_TYPE Debug)

#add_compile_options(-g -O0)
