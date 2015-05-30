IF(SQLITE3_FOUND AND PROTOBUF_FOUND)

INCLUDE_DIRECTORIES( ${SQLITE3_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR} )

SET(TARGET_SRC
    FeatureSourceMVT.cpp    
    vector_tile.pb.cc
)

SET(TARGET_H   	
    MVTFeatureOptions
    vector_tile.pb.h
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthUtil)
SET(TARGET_LIBRARIES_VARS SQLITE3_LIBRARY PROTOBUF_LITE_LIBRARY)
SETUP_PLUGIN(osgearth_feature_mapnikvectortiles)


# to install public driver includes:
SET(LIB_NAME feature_mapnikvectortiles)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(SQLITE3_FOUND AND PROTOBUF_FOUND)