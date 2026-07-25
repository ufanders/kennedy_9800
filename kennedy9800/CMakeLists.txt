# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(kennedy9800)

# tusb_config.h lives in main/, but CMake only propagates include dirs along
# the dependency graph (main -> tinyusb, not the reverse), so the raw
# tinyusb component's own compile step can't see it unless we add it here.
idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_SOURCE_DIR}/main" APPEND)