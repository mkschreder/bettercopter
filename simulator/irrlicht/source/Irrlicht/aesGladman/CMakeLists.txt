cmake_minimum_required(VERSION 2.8)

# Sources
file(GLOB Library_SOURCES *.cpp)
file(GLOB Library_HEADER *.h)

include_directories("../../../include")

set (CMAKE_CXX_FLAGS "-Wall -pipe -fno-exceptions -fno-rtti -fstrict-aliasing -g -D_DEBUG  -I/usr/X11R6/include -DIRRLICHT_EXPORTS=1")

# Executable
add_library(aes ${Library_SOURCES} ${Library_HEADERS})

add_custom_command(TARGET aes POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "libaes.a"      # <--this is in-file
        "../")                 # <--this is out-file path
