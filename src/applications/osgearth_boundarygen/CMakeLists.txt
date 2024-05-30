add_osgearth_app(
    TARGET osgearth_boundarygen
    SOURCES
        BoundaryUtil.cpp
        VertexCollectionVisitor.cpp
        boundarygen.cpp
    HEADERS
        BoundaryUtil
        VertexCollectionVisitor    
    FOLDER Tools )
