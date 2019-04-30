include(SelectLibraryConfigurations)
include(FindPackageHandleStandardArgs)

find_path(WEBP_INCLUDE_DIR "webp/decode.h"
 	      PATH_SUFFIXES include
          )
          
find_library(WEBP_LIBRARY_RELEASE NAMES webp libwebp_a libwebp
             HINTS "${WEBP_DIR}"
             PATH_SUFFIXES "${PATH_SUFFIXES}" ${LIBRARY_PATH_SUFFIXES}
			)


find_library(WEBP_LIBRARY_DEBUG NAMES webp_debug libwebp libwebp_a libwebp
             HINTS "${WEBP_DIR}"
             PATH_SUFFIXES "${PATH_SUFFIXES}" ${LIBRARY_PATH_SUFFIXES} 
			)

select_library_configurations(WEBP)

find_package_handle_standard_args(WEBP DEFAULT_MSG
                                  WEBP_INCLUDE_DIR)

mark_as_advanced(WEBP_INCLUDE_DIR)

set( WEBP_FOUND "NO" )
if( WEBP_LIBRARY AND WEBP_INCLUDE_DIR )
    set( WEBP_FOUND "YES" )
endif()