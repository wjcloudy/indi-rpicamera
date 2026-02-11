cmake_minimum_required(VERSION 3.16)
project(indi_rpicamera CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

set(RPICAMERA_VERSION_MAJOR 1)
set(RPICAMERA_VERSION_MINOR 0)

# ---- Find dependencies ----
find_package(INDI REQUIRED COMPONENTS driver)
find_package(CFITSIO REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

# libcamera via pkg-config (standard on Raspberry Pi OS)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# ---- Generate config.h and driver XML from templates ----
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_rpicamera.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_rpicamera.xml
)

# ---- Include paths ----
include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${INDI_INCLUDE_DIR}
    ${CFITSIO_INCLUDE_DIR}
)

# ---- Source files ----
set(RPICAMERA_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_rpicamera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_rpicamera_driver.cpp
)

# ---- Build driver executable ----
add_executable(indi_rpicamera ${RPICAMERA_SRCS})

target_link_libraries(indi_rpicamera
    ${INDI_LIBRARIES}
    ${CFITSIO_LIBRARIES}
    ${ZLIB_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    PkgConfig::LIBCAMERA
)

# ---- Install ----
install(TARGETS indi_rpicamera RUNTIME DESTINATION bin)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_rpicamera.xml
    DESTINATION ${INDI_DATA_DIR}
)
