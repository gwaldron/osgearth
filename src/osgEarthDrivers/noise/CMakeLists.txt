#
# Noise generator plugin
#

set(TARGET_SRC
	NoiseExtension.cpp
	NoiseTerrainEffect.cpp
	${SHADERS_CPP} )
	
set(LIB_PUBLIC_HEADERS
	NoiseExtension
	NoiseOptions
	NoiseTerrainEffect)
	
set(TARGET_H
	${LIB_PUBLIC_HEADERS} )

set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil)
	
setup_plugin(osgearth_noise)

# to install public driver includes:
set(LIB_NAME noise)
