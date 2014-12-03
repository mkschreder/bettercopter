cmake_minimum_required(VERSION 2.8)

# Sources
file(GLOB Library_SOURCES *.cpp aesGladman/*.cpp zlib/*.c libpng/*.c lzma/*.c)
file(GLOB Library_HEADER *.h jpeglib/*.h aesGladman/*.h lzma/*.h)

include_directories("../../include")
include_directories(".")

#find_package("libjpeg")
#find_package("zlib")
#find_package("libpng")

set (CMAKE_CXX_FLAGS "-Wall -pipe -fno-exceptions -fno-rtti -fstrict-aliasing -O2  -I/usr/X11R6/include -DIRRLICHT_EXPORTS=1")

# Executable
add_library(irrlicht ${Library_SOURCES} ${Library_HEADERS})

target_link_libraries(irrlicht "jpeg")
