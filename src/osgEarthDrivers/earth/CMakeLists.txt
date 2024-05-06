add_osgearth_plugin(
    TARGET osgdb_earth
    SOURCES
        ReaderWriterOsgEarth.cpp
        EarthFileSerializer1.cpp
        EarthFileSerializer2.cpp
    HEADERS
        EarthFileSerializer
)        
