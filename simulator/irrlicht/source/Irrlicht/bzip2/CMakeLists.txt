cmake_minimum_required(VERSION 2.8)

# Sources
file(GLOB Library_SOURCES *.cpp)
file(GLOB Library_HEADER *.h)

include_directories("../../../include")

set (CMAKE_CXX_FLAGS "-Wall -pipe -fno-exceptions -fno-rtti -fstrict-aliasing -g -D_DEBUG  -I/usr/X11R6/include -DIRRLICHT_EXPORTS=1")

# Executable
add_library(bz2 ${Library_SOURCES} ${Library_HEADERS})
