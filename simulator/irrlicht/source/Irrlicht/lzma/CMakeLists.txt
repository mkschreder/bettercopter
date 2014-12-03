cmake_minimum_required(VERSION 2.8)

# Sources
file(GLOB Library_SOURCES *.c)
file(GLOB Library_HEADER *.h)

include_directories("../../../include")

# Executable
add_library(lzma ${Library_SOURCES} ${Library_HEADERS})

add_custom_command(TARGET lzma POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "liblzma.a"      # <--this is in-file
        "../")                 # <--this is out-file path
