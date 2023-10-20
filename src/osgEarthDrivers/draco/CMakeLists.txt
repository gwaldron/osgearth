IF(draco_FOUND)

    SET(TARGET_SRC
        ReaderWriterDraco.cpp
    )

    INCLUDE_DIRECTORIES( ${draco_INCLUDE_DIRS} )
    SET(TARGET_LIBRARIES_VARS draco_LIBRARIES OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY )


    #### end var setup  ###
    SETUP_PLUGIN(draco)
    
ENDIF(draco_FOUND)
