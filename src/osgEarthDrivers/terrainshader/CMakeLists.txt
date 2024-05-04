add_osgearth_library_as_plugin(
    TARGET osgdb_osgearth_terrainshader
    PUBLIC_HEADERS
        TerrainShaderExtension
        TerrainShaderOptions
    SOURCES
        TerrainShaderExtension.cpp )
    