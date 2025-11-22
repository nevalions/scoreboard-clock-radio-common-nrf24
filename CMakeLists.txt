cmake_minimum_required(VERSION 3.16)

# Create the radio-common static library
add_library(radio_common STATIC
    src/radio_common.c
)

# Include directories
target_include_directories(radio_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Platform-specific dependencies
if(ESP_PLATFORM)
    # ESP-IDF specific settings
    idf_component_register(
        SRCS "src/radio_common.c"
        INCLUDE_DIRS "include"
        REQUIRES driver
    )
else()
    # Generic platform settings
    target_compile_definitions(radio_common PUBLIC
        -DRADIO_COMMON_STANDALONE
    )
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
    target_compile_options(radio_common PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()

# Debug information
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(radio_common PUBLIC
        -DRADIO_DEBUG_ENABLED=1
    )
else()
    target_compile_definitions(radio_common PUBLIC
        -DRADIO_DEBUG_ENABLED=0
    )
endif()