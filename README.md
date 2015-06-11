# darwin-updates

To build software:

    cmake .
    make

Good luck!

Recipe for converting a directory:

    git mv Makefile Makefile.old

then add a CMakeLists.txt by converting stuff in Makefile to CMake syntax.

Make sure to hook up your new CMakeLists.txt to upper directory by inserting add_subdirectory calls when appropriate.

Look at CMakeLists.txt in /Linux/build and /Linux/project/demo for examples.
