# P4 Audio Producer — ESP-IDF v5.5.2, target ESP32-P4
cmake_minimum_required(VERSION 3.16)

# Version check: ensure ESP-IDF v5.5.2
# Check BEFORE including project.cmake (which may set its own version)
if(DEFINED ENV{IDF_PATH})
    # Use git command directly from IDF_PATH
    execute_process(
        COMMAND git describe --tags
        WORKING_DIRECTORY "$ENV{IDF_PATH}"
        OUTPUT_VARIABLE IDF_VERSION_CHECK
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE GIT_RESULT
    )
    if(GIT_RESULT OR IDF_VERSION_CHECK STREQUAL "")
        message(FATAL_ERROR "Failed to detect ESP-IDF version from $ENV{IDF_PATH}. Required: v5.5.2")
    endif()
    if(NOT IDF_VERSION_CHECK MATCHES ".*5\\.5\\.2.*")
        message(FATAL_ERROR "Wrong ESP-IDF version: '${IDF_VERSION_CHECK}'. Required: v5.5.2. Set IDF_PATH to ESP-IDF v5.5.2 installation.")
    endif()
    message(STATUS "ESP-IDF version check: ${IDF_VERSION_CHECK}")
else()
    message(FATAL_ERROR "IDF_PATH not set. Required: ESP-IDF v5.5.2")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p4_audio_producer)

# Verify ESP-IDF version after project.cmake (it may set IDF_VER)
if(DEFINED IDF_VER)
    if(NOT IDF_VER MATCHES ".*5\\.5\\.2.*")
        message(FATAL_ERROR "ESP-IDF version mismatch: IDF_VER='${IDF_VER}' (from project.cmake) does not match v5.5.2 requirement")
    endif()
    message(STATUS "ESP-IDF version (from project.cmake): ${IDF_VER}")
endif()
