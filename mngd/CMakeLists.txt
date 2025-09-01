cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(COMP8800_Managed)
target_sources(app PRIVATE 
    src/main.c
    ../general/mesh.c
)

zephyr_include_directories(../general)

include_directories(
    ../general
    ${ZEPHYR_BASE}/../modules/crypto/mbedtls/include
    ${ZEPHYR_BASE}/../modules/crypto/mbedtls/include/psa
)
zephyr_library_include_directories(.)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR}/mngd/zephyr/include/generated)