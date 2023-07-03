# Finddraco
#
# Locates draco and sets the following variables:
#
# draco_FOUND draco_INCLUDE_DIRS draco_LIBARY_DIRS draco_LIBRARIES
# draco_VERSION_STRING
#
# draco_FOUND is set to YES only when all other variables are successfully
# configured.

unset(draco_FOUND)
unset(draco_INCLUDE_DIRS)
unset(draco_LIBRARY_DIRS)
unset(draco_LIBRARIES)

mark_as_advanced(draco_FOUND)
mark_as_advanced(draco_INCLUDE_DIRS)
mark_as_advanced(draco_LIBRARY_DIRS)
mark_as_advanced(draco_LIBRARIES)

set(draco_version_file_no_prefix "draco/core/draco_version.h")

# Set draco_INCLUDE_DIRS
find_path(draco_INCLUDE_DIRS NAMES "draco/core/status.h")

# Find the library.
if(BUILD_SHARED_LIBS)
  find_library(draco_LIBRARIES NAMES draco.dll libdraco.dylib libdraco.so)
else()
  find_library(draco_LIBRARIES NAMES draco.lib libdraco.a)
endif()

# Store path to library.
get_filename_component(draco_LIBRARY_DIRS ${draco_LIBRARIES} DIRECTORY)

if(draco_INCLUDE_DIRS
   AND draco_LIBRARY_DIRS
   AND draco_LIBRARIES)
  set(draco_FOUND YES)
endif()
