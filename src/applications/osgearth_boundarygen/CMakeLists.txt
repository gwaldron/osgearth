INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )
SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY)

SET(TARGET_H
    BoundaryUtil
    VertexCollectionVisitor
)

SET(TARGET_SRC
    BoundaryUtil.cpp
    VertexCollectionVisitor.cpp
    boundarygen.cpp
)

#### end var setup  ###
SETUP_APPLICATION(osgearth_boundarygen)
