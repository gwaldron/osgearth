IF(draco_FOUND)

    SET(TARGET_SRC
        ReaderWriterDraco.cpp
    )

    INCLUDE_DIRECTORIES( ${draco_INCLUDE_DIRS} )
    SET(TARGET_LIBRARIES_VARS draco_LIBRARIES )

    #### end var setup  ###
    SETUP_PLUGIN(draco)
    
ENDIF(draco_FOUND)
