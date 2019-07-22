# Copyright (c) 2013 Nathan Osman
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Determines whether or not the compiler supports C++11

# Adapted from:
# https://github.com/nathan-osman/CXX11-CMake-Macros
#
# Pass:
#   msvc 14+
#   g++ 4.9.0+
#   clang 3.4+
#
# Earlier versions of these compilers might support C++11 but the purposes
# of osgEarth we are using these minimum requirements.

macro(check_for_cxx11_compiler _VAR)
    set(${_VAR})
    if((MSVC AND NOT ${MSVC_VERSION} VERSION_LESS 1900) OR
       (CMAKE_COMPILER_IS_GNUCXX AND NOT ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 4.9.0) OR
       (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND NOT ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 3.4))
        set(${_VAR} 1)

        message(STATUS "Checking for C++11 compiler - available")

        # enable C++11 compilation if available
        set(CMAKE_CXX_STANDARD 11)
        add_definitions(-DOSGEARTH_CXX11)

        # is GCC < 5, use the old ABI for binary compatibility
        if (CMAKE_COMPILER_IS_GNUCXX AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 5.0)
            add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
        endif()
    else()
        message(STATUS "Checking for C++11 compiler - unavailable")
    endif()
endmacro()
