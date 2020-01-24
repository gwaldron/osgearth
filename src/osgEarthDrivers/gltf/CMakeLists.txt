SET(TARGET_H
    GLTFReader.h
    GLTFWriter.h
    B3DMReader.h
    B3DMWriter.h
)

SET(TARGET_SRC
    ReaderWriterGLTF.cpp
)

INCLUDE_DIRECTORIES(${OE_THIRD_PARTY_DIR}/tinygltf ${OE_THIRD_PARTY_DIR}/rapidjson/include/rapidjson)

IF(draco_FOUND)
    INCLUDE_DIRECTORIES( ${draco_INCLUDE_DIRS} )
    SET(TARGET_LIBRARIES_VARS draco_LIBRARIES )
ENDIF(draco_FOUND)

#### end var setup  ###
SETUP_PLUGIN(gltf)
