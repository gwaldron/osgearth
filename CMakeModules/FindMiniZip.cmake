# Locate minizip library
# This module defines
# MINIZIP_LIBRARY
# MINIZIP_FOUND, if false, do not try to link to libzip 
# MINIZIP_INCLUDE_DIR, where to find the headers
#

# prefer pkg-config
IF(UNIX)
	INCLUDE(FindPkgConfig)
	IF(PKG_CONFIG_FOUND)
		pkg_check_modules(MINIZIP QUIET minizip)
		IF(MINIZIP_FOUND)
			# pkgconfig does not define the singular names
			SET(MINIZIP_LIBRARY ${MINIZIP_LIBRARIES} CACHE STRING "Minizip libraries to link against")
			SET(MINIZIP_INCLUDE_DIR ${MINIZIP_INCLUDE_DIRS} CACHE STRING "Minizip include dirs to look for headers")
			link_directories(${MINIZIP_LIBRARY_DIRS})
			ADD_DEFINITIONS(-DOSGEARTH_HAVE_MINIZIP)
		ENDIF(MINIZIP_FOUND)
	ENDIF(PKG_CONFIG_FOUND)
ENDIF(UNIX)

# fallback logic
IF(NOT MINIZIP_FOUND)
	FIND_PATH(MINIZIP_INCLUDE_DIR zip.h
		${CMAKE_SOURCE_DIR}/src/3rdparty/minizip
		$ENV{MINIZIP_DIR}/include
		$ENV{MINIZIP_DIR}
		$ENV{OSGDIR}/include
		$ENV{OSGDIR}
		$ENV{OSG_ROOT}/include
		~/Library/Frameworks
		/Library/Frameworks
		/usr/local/include
		/usr/include
		/sw/include # Fink
		/opt/local/include # DarwinPorts
		/opt/csw/include # Blastwave
		/opt/include
		[HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/include
		/usr/freeware/include
	)

	FIND_LIBRARY(MINIZIP_LIBRARY 
		NAMES minizip
		PATHS
		${CMAKE_SOURCE_DIR}/src/3rdparty/minizip
		$ENV{MINIZIP_DIR}/lib
		$ENV{MINIZIP_DIR}
		$ENV{OSGDIR}/lib
		$ENV{OSGDIR}
		$ENV{OSG_ROOT}/lib
		~/Library/Frameworks
		/Library/Frameworks
		/usr/local/lib
		/usr/lib
		/sw/lib
		/opt/local/lib
		/opt/csw/lib
		/opt/lib
		[HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
		/usr/freeware/lib64
	)

	SET(MINIZIP_FOUND "NO")
	IF(MINIZIP_LIBRARY AND MINIZIP_INCLUDE_DIR)
		SET(MINIZIP_FOUND "YES")
		ADD_DEFINITIONS(-DOSGEARTH_HAVE_MINIZIP)
	ENDIF(MINIZIP_LIBRARY AND MINIZIP_INCLUDE_DIR)
ENDIF(NOT MINIZIP_FOUND)
