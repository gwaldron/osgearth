add_osgearth_library_as_plugin(
    TARGET osgdb_earth
    SOURCES
        ReaderWriterOsgEarth.cpp
        EarthFileSerializer1.cpp
        EarthFileSerializer2.cpp
    HEADERS
        EarthFileSerializer
)        
