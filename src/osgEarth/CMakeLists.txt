# osgEarth Core Library

IF(OSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY)
ELSE ()
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY_STATIC)
ENDIF()

# Add __STDC_FORMAT_MACROS for use of PRIu64 macro
ADD_DEFINITIONS(-D__STDC_FORMAT_MACROS)

# CURL options
OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

# NVTT mipmap generation
option(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS "Use NVTT, if available, to generate texture mipmaps on the CPU" OFF)
mark_as_advanced(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)
if(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)
    add_definitions(-DOSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)
endif(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)

# TinyXML options
ADD_DEFINITIONS(-DTIXML_USE_STL)

# Builds the HTTPClient with WIN_INET instead of CURL
option(OSGEARTH_ENABLE_WININET_FOR_HTTP "Whether to use the WinInet library for HTTP requests (instead of cURL)" OFF)
mark_as_advanced(OSGEARTH_ENABLE_WININET_FOR_HTTP)
if (OSGEARTH_ENABLE_WININET_FOR_HTTP)
    ADD_DEFINITIONS(-DOSGEARTH_USE_WININET_FOR_HTTP)
endif ()

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
    IF(CURL_IS_STATIC)
        ADD_DEFINITIONS(-DCURL_STATICLIB)
        SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm wldap32)
    ENDIF(CURL_IS_STATIC)
    LIST(APPEND TARGET_EXTERNAL_LIBRARIES psapi)
ENDIF(WIN32)


LIST(APPEND TARGET_EXTERNAL_LIBRARIES meshoptimizer::meshoptimizer)


# Generate the Version header.
# set(OSGEARTH_SOVERSION_VALUE "${OSGEARTH_SOVERSION}")
set(OSGEARTH_VERSION_HEADER "${OSGEARTH_BUILDTIME_INCLUDE_DIR}/osgEarth/Version")
message(STATUS "Creating ${OSGEARTH_VERSION_HEADER}")
configure_file(Version.in ${OSGEARTH_VERSION_HEADER})

set(OSGEARTH_BUILDCONFIG_HEADER "${OSGEARTH_BUILDTIME_INCLUDE_DIR}/osgEarth/BuildConfig")
message(STATUS "Creating ${OSGEARTH_BUILDCONFIG_HEADER}")
configure_file(BuildConfig.in ${OSGEARTH_BUILDCONFIG_HEADER})

# Name all the library files
SET(LIB_NAME osgEarth)

set(TARGET_GLSL
    CascadeDraping.glsl
    Chonk.glsl
    Chonk.Culling.glsl
    DepthOffset.glsl
    Draping.glsl
    DrawInstancedAttribute.glsl
    GPUClamping.glsl
    GPUClamping.lib.glsl
    HexTiling.glsl
    Instancing.glsl
    LineDrawable.glsl
    MetadataNode.glsl
    WireLines.glsl
    PhongLighting.glsl
    PointDrawable.glsl
    Text.glsl
    Text_legacy.glsl
    ContourMap.glsl
    GeodeticGraticule.glsl
    LogDepthBuffer.glsl
    LogDepthBuffer.VertOnly.glsl
    ShadowCaster.glsl
    SimpleOceanLayer.glsl
    RTTPicker.glsl
    WindLayer.CS.glsl
    PBR.glsl
)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    Version.in
    Shaders.cpp.in)

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(LIB_PUBLIC_HEADERS    
    AGG.h
    AltitudeFilter
    AltitudeSymbol
    AnnotationData
    AnnotationLayer
    AnnotationNode
    AnnotationRegistry
    AnnotationSettings
    AnnotationUtils
    ArcGISServer
    ArcGISTilePackage
    AtlasBuilder
    AttributesFilter
    AutoClipPlaneHandler
    AutoScaleCallback
    BboxDrawable
    BBoxSymbol
    BillboardResource
    BillboardSymbol
    Bing
    Bounds
    BufferFilter
    BuildGeometryFilter
    BuildTextFilter
    Cache
    CacheBin
    CachePolicy
    CacheSeed
    Callbacks
    Callouts
    CameraUtils
    Capabilities
    CascadeDrapingDecorator
    CentroidFilter
    CesiumIon
    Chonk
    CircleNode
    ClampableNode
    ClampCallback
    Clamping
    ClampingTechnique
    ClipSpace
    ClusterNode
    Color
    ColorFilter
    Common
    Composite
    CompressedArray
    CompositeTiledModelLayer
    Config
    Containers
    ContourMap
    Controls
    ConvertTypeFilter
    Coverage
    CoverageLayer
    CoverageSymbol
    CropFilter
    CssUtils
    Cube
    CullingUtils
    DateTime
    DateTimeRange
    DebugImageLayer
    DecalLayer
    DepthOffset
    Draggers
    DrapeableNode
    DrapingCullSet
    DrapingTechnique
    DrawInstanced
    EarthManipulator
    ECEF
    Elevation
    ElevationLayer
    ElevationLOD
    ElevationPool
    ElevationQuery
    ElevationRanges
    EllipseNode
    Ellipsoid
    Endian
    Ephemeris
    ExampleResources
    Export
    Expression
    Extension
    ExtrudeGeometryFilter
    ExtrusionSymbol
    FadeEffect
    Feature
    FeatureCursor
    FeatureDisplayLayout
    FeatureElevationLayer
    FeatureImageLayer
    FeatureIndex
    FeatureModelGraph
    FeatureModelLayer
    FeatureModelSource
    FeatureNode
    FeatureRasterizer
    FeatureSDFLayer
    FeatureSource
    FeatureSourceIndexNode
    FileUtils
    Fill
    Filter
    FilterContext
    FilteredFeatureSource
    FlatteningLayer
    Formatter
    FractalElevationLayer
    FrameClock
    GARSGraticule
    GDAL
    GDALDEM
    GeoCommon
    GeoData
    GeodeticGraticule
    GeodeticLabelingEngine
    Geoid
    GeoMath
    Geometry
    GeometryClamper
    GeometryCloud
    GeometryCompiler
    GeometryFactory
    GeometryRasterizer
    GeometryUtils
    GeoPositionNode
    GeoPositionNodeAutoScaler
    GEOS
    GeoTransform
    GLSLChunker
    GLUtils
    GraticuleLabelingEngine
    HeightFieldUtils
    Horizon
    HorizonClipPlane
    HTM
    HTTPClient
    IconResource
    IconSymbol
    ImageLayer
    ImageMosaic
    ImageOverlay
    ImageOverlayEditor
    ImageToFeatureLayer
    ImageToHeightFieldConverter
    ImageUtils
    InstanceBuilder
    InstanceCloud
    InstanceResource
    InstanceSymbol
    IntersectionPicker
    IOTypes
    JoinPointsLinesFilter
    JsonUtils
    LabelNode
    LandCover
    LandCoverLayer
    LatLongFormatter
    Layer
    LayerReference
    LayerShader
    Lighting
    LinearLineOfSight
    LineDrawable
    LineFunctor
    LineOfSight
    LineSymbol
    LoadableNode
    LocalGeometryNode
    LocalTangentPlane
    Locators
    LogarithmicDepthBuffer
    Map
    MapboxGLGlyphManager
    MapboxGLImageLayer
    MapCallback
    MapModelChange
    MapNode
    MapNodeObserver
    MaterialLoader
    Math
    MBTiles
    MeasureTool
    MemCache
    Memory
    MeshConsolidator
    MeshFlattener
    MeshSubdivider
    MetadataNode
    MetaTile
    Metrics
    MGRSFormatter
    MGRSGraticule
    ModelLayer
    ModelNode
    ModelResource
    ModelSource
    ModelSymbol
    MVT
    NativeProgramAdapter
    NetworkMonitor
    NodeUtils
    NoiseTextureFactory
    Notify
    ObjectIDPicker
    ObjectIndex
    OGRFeatureSource
    OgrUtils
    optional
    OverlayDecorator
    PagedNode
    PatchLayer
    PhongLightingEffect
    Picker
    PlaceNode
    PluginLoader
    PointDrawable
    PointSymbol
    PolygonizeLines
    PolygonSymbol
    PowerlineLayer
    PrimitiveIntersector
    Profile
    Progress
    Query
    RadialLineOfSight
    Random
    RectangleNode
    RefinePolicy
    Registry
    RenderSymbol
    ResampleFilter
    Resource
    ResourceCache
    ResourceLibrary
    Revisioning
    RTTPicker
    ScaleFilter
    ScatterFilter
    SceneGraphCallback
    ScreenSpaceLayout
    ScreenSpaceLayoutCallout
    ScreenSpaceLayoutDeclutter
    ScreenSpaceLayoutImpl
    Script
    ScriptEngine
    ScriptFilter
    SDF
    SDF
    SelectExtentTool
    Session
    ShaderFactory
    ShaderGenerator
    ShaderLayer
    ShaderLoader
    ShaderMerger
    Shaders
    ShaderUtils
    Shadowing
    SimpleOceanLayer
    SimplePager
    SimplexNoise
    Skins
    Sky
    SkyView
    SpatialReference
    StarData
    StateSetCache
    StateTransition
    Status
    StringUtils
    Stroke
    Style
    StyleSelector
    StyleSheet
    SubstituteModelFilter
    Symbol
    Tags
    TDTiles
    Terrain
    TerrainConstraintLayer
    TerrainEffect
    TerrainEngineNode
    TerrainEngineRequirements
    TerrainLayer
    TerrainMeshLayer
    TerrainOptions
    TerrainProfile
    TerrainResources
    TerrainTileModel
    TerrainTileModelFactory
    TerrainTileNode
    TessellateOperator
    Tessellator
    Text
    TextSymbol
    TextSymbolizer
    TextureArena
    TextureBuffer
    TFS
    TFSPackager
    Threading
    ThreeDTilesLayer
    TileCache
    TiledFeatureModelLayer
    TiledModelLayer
    TileEstimator
    TileHandler
    TileIndex
    TileIndexBuilder    
    TileKey
    TileLayer
    TileMesher
    TileRasterizer
    TileSource
    TileSourceElevationLayer
    TileSourceImageLayer
    TileVisitor
    TimeControl
    TimeSeriesImage
    TMS
    TMSBackFiller
    TopologyGraph
    TrackNode
    TransformFilter
    Units
    URI
    Utils
    UTMGraticule
    UTMLabelingEngine
    VerticalDatum
    VideoLayer
    ViewFitter
    Viewpoint
    VirtualProgram
    VisibleLayer
    WFS
    WindLayer
    WireLines
    WMS
    XmlUtils
    XYZ
    XYZFeatureSource
    XYZModelLayer

    rtree.h
    weemesh.h
    weejobs.h

    ${OSGEARTH_VERSION_HEADER}
)

if(OSGEARTH_ENABLE_IMGUI)
    set(LIB_PUBLIC_HEADERS_IMGUI
        ImGui/ImGui
        ImGui/ImGuiApp
        ImGui/AnnotationsGUI
        ImGui/ContentBrowserGUI
        ImGui/LayersGUI
        ImGui/NetworkMonitorGUI
        ImGui/NotifyGUI
        ImGui/PickerGUI
        ImGui/SceneGraphGUI
        ImGui/SearchGUI
        ImGui/TextureInspectorGUI
        ImGui/ViewpointsGUI
        ImGui/SystemGUI
        ImGui/EnvironmentGUI
        ImGui/ShaderGUI
        ImGui/TerrainGUI
        ImGui/CameraGUI
        ImGui/RenderingGUI
        )
endif()

if(OSGEARTH_BUILD_LEGACY_CONTROLS_API)
    list(APPEND LIB_PUBLIC_HEADERS
        ActivityMonitorTool
        MouseCoordsTool)
endif()

IF (NOT TINYXML_FOUND)
    SET(LIB_PUBLIC_HEADERS
        ${LIB_PUBLIC_HEADERS}
        tinystr.h
        tinyxml.h
    )
ENDIF (NOT TINYXML_FOUND)

IF (NOT TINYXML_FOUND)
    SET(TINYXML_SRC
        tinystr.cpp
        tinyxml.cpp
        tinyxmlerror.cpp
        tinyxmlparser.cpp
    )
ENDIF (NOT TINYXML_FOUND)



SET(VERSION_GIT_SOURCE "")
find_package(Git)
if (GIT_FOUND)
    option(OSGEARTH_EMBED_GIT_SHA "Encode the SHA in the Version header code" OFF)
    if(OSGEARTH_EMBED_GIT_SHA)
        include(GetGitRevisionDescription)
        get_git_head_revision(GIT_REFSPEC OSGEARTH_GIT_SHA1)
        if(OSGEARTH_GIT_SHA1)
            message(STATUS "Embedding git SHA = ${OSGEARTH_GIT_SHA1}")
            ADD_DEFINITIONS(-DOSGEARTH_EMBED_GIT_SHA)

            # auto-generate the VersionGit file to include the git SHA1 variable.
            configure_file(
                "${CMAKE_CURRENT_SOURCE_DIR}/VersionGit.cpp.in"
                "${CMAKE_CURRENT_BINARY_DIR}/VersionGit.cpp"
                @ONLY)

            set(VERSION_GIT_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/VersionGit.cpp")
        endif()
    endif()
endif()

set(TARGET_SRC
    AltitudeFilter.cpp
    AltitudeSymbol.cpp
    AnnotationData.cpp
    AnnotationLayer.cpp
    AnnotationNode.cpp
    AnnotationRegistry.cpp
    AnnotationSettings.cpp
    AnnotationUtils.cpp
    ArcGISServer.cpp
    ArcGISTilePackage.cpp
    AtlasBuilder.cpp
    AttributesFilter.cpp
    AutoClipPlaneHandler.cpp
    BboxDrawable.cpp
    BBoxSymbol.cpp
    BillboardResource.cpp
    BillboardSymbol.cpp
    Bing.cpp
    Bounds.cpp
    BufferFilter.cpp
    BuildGeometryFilter.cpp
    BuildTextFilter.cpp
    Cache.cpp
    CacheBin.cpp
    CachePolicy.cpp
    CacheSeed.cpp
    Callouts.cpp
    CameraUtils.cpp
    Capabilities.cpp
    CascadeDrapingDecorator.cpp
    CentroidFilter.cpp
    CesiumIon.cpp
    Chonk.cpp
    CircleNode.cpp
    ClampableNode.cpp
    ClampCallback.cpp
    Clamping.cpp
    ClampingTechnique.cpp
    ClipSpace.cpp
    ClusterNode.cpp
    Color.cpp
    ColorFilter.cpp
    Composite.cpp
    CompositeTiledModelLayer.cpp
    Compressors.cpp
    CompressedArray.cpp
    Config.cpp
    ContourMap.cpp
    Controls.cpp
    ConvertTypeFilter.cpp
    CoverageLayer.cpp
    CoverageSymbol.cpp
    CropFilter.cpp
    CssUtils.cpp
    Cube.cpp
    CullingUtils.cpp
    DateTime.cpp
    DateTimeRange.cpp
    DebugImageLayer.cpp
    DecalLayer.cpp
    DepthOffset.cpp
    Draggers.cpp
    DrapeableNode.cpp
    DrapingCullSet.cpp
    DrapingTechnique.cpp
    DrawInstanced.cpp
    EarthManipulator.cpp
    ECEF.cpp
    Elevation.cpp
    ElevationLayer.cpp
    ElevationLOD.cpp
    ElevationPool.cpp
    ElevationQuery.cpp
    ElevationRanges.cpp
    EllipseNode.cpp
    Ellipsoid.cpp
    Ephemeris.cpp
    ExampleResources.cpp
    Expression.cpp
    Extension.cpp
    ExtrudeGeometryFilter.cpp
    ExtrusionSymbol.cpp
    FadeEffect.cpp
    Feature.cpp
    FeatureCursor.cpp
    FeatureDisplayLayout.cpp
    FeatureElevationLayer.cpp
    FeatureImageLayer.cpp
    FeatureModelGraph.cpp
    FeatureModelLayer.cpp
    FeatureModelSource.cpp
    FeatureNode.cpp
    FeatureRasterizer.cpp
    FeatureSDFLayer.cpp
    FeatureSource.cpp
    FeatureSourceIndexNode.cpp
    FileGDBFeatureSource.cpp
    FileUtils.cpp
    Fill.cpp
    Filter.cpp
    FilterContext.cpp
    FilteredFeatureSource.cpp
    FlatteningLayer.cpp
    FractalElevationLayer.cpp
    FrameClock.cpp
    GARSGraticule.cpp
    GDAL.cpp
    GDALDEM.cpp
    GeoData.cpp
    GeodeticGraticule.cpp
    GeodeticLabelingEngine.cpp
    Geoid.cpp
    GeoMath.cpp
    Geometry.cpp
    GeometryClamper.cpp
    GeometryCloud.cpp
    GeometryCompiler.cpp
    GeometryFactory.cpp
    GeometryRasterizer.cpp
    GeometryUtils.cpp
    GeoPositionNode.cpp
    GeoPositionNodeAutoScaler.cpp
    GEOS.cpp
    GeoTransform.cpp
    GLSLChunker.cpp
    GLUtils.cpp
    GraticuleLabelingEngine.cpp
    HeightFieldUtils.cpp
    Horizon.cpp
    HorizonClipPlane.cpp
    HTM.cpp
    HTTPClient.cpp
    IconResource.cpp
    IconSymbol.cpp
    ImageLayer.cpp
    ImageMosaic.cpp
    ImageOverlay.cpp
    ImageOverlayEditor.cpp
    ImageToFeatureLayer.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    InstanceBuilder.cpp
    InstanceCloud.cpp
    InstanceResource.cpp
    InstanceSymbol.cpp
    IntersectionPicker.cpp
    IOTypes.cpp
    JoinPointsLinesFilter.cpp
    JsonUtils.cpp
    LabelNode.cpp
    LandCover.cpp
    LandCoverLayer.cpp
    LatLongFormatter.cpp
    Layer.cpp
    LayerShader.cpp
    Lighting.cpp
    LinearLineOfSight.cpp
    LineDrawable.cpp
    LineSymbol.cpp
    LocalGeometryNode.cpp
    LocalTangentPlane.cpp
    LogarithmicDepthBuffer.cpp
    Map.cpp
    MapboxGLGlyphManager.cpp
    MapboxGLImageLayer.cpp
    MapCallback.cpp
    MapNode.cpp
    MaterialLoader.cpp
    Math.cpp
    MBTiles.cpp
    MeasureTool.cpp
    MemCache.cpp
    Memory.cpp
    MeshConsolidator.cpp
    MeshFlattener.cpp
    MeshSubdivider.cpp
    MetadataNode.cpp
    MetaTile.cpp
    Metrics.cpp
    MGRSFormatter.cpp
    MGRSGraticule.cpp
    ModelLayer.cpp
    ModelNode.cpp
    ModelResource.cpp
    ModelSource.cpp
    ModelSymbol.cpp
    MVT.cpp
    NetworkMonitor.cpp
    NodeUtils.cpp
    NoiseTextureFactory.cpp
    Notify.cpp
    ObjectIDPicker.cpp
    ObjectIndex.cpp
    OGRFeatureSource.cpp
    OgrUtils.cpp
    OverlayDecorator.cpp
    PagedNode.cpp
    PatchLayer.cpp
    PhongLightingEffect.cpp
    PlaceNode.cpp
    PointDrawable.cpp
    PointSymbol.cpp
    PolygonizeLines.cpp
    PolygonSymbol.cpp
    PowerlineLayer.cpp
    PrimitiveIntersector.cpp
    Profile.cpp
    Progress.cpp
    Query.cpp
    RadialLineOfSight.cpp
    Random.cpp
    RectangleNode.cpp
    Registry.cpp
    RenderSymbol.cpp
    ResampleFilter.cpp
    Resource.cpp
    ResourceCache.cpp
    ResourceLibrary.cpp
    Revisioning.cpp
    RTTPicker.cpp
    ScaleFilter.cpp
    ScatterFilter.cpp
    SceneGraphCallback.cpp
    ScreenSpaceLayout.cpp
    ScriptEngine.cpp
    ScriptFilter.cpp
    SDF.cpp
    SDF.cpp
    SelectExtentTool.cpp
    Session.cpp
    ShaderFactory.cpp
    ShaderGenerator.cpp
    ShaderLayer.cpp
    ShaderLoader.cpp
    ShaderMerger.cpp
    ShaderUtils.cpp
    Shadowing.cpp
    SimpleOceanLayer.cpp
    SimplePager.cpp
    SimplexNoise.cpp
    Skins.cpp
    Sky.cpp
    SkyView.cpp
    SpatialReference.cpp
    StateSetCache.cpp
    Status.cpp
    StringUtils.cpp
    Stroke.cpp
    Style.cpp
    StyleSelector.cpp
    StyleSheet.cpp
    SubstituteModelFilter.cpp
    Symbol.cpp
    TDTiles.cpp
    Terrain.cpp
    TerrainConstraintLayer.cpp
    TerrainEngineNode.cpp
    TerrainMeshLayer.cpp
    TerrainOptions.cpp
    TerrainProfile.cpp
    TerrainResources.cpp
    TerrainTileModel.cpp
    TerrainTileModelFactory.cpp
    TessellateOperator.cpp
    Tessellator.cpp
    Text.cpp
    TextSymbol.cpp
    TextSymbolizer.cpp
    TextureArena.cpp
    TextureBuffer.cpp
    TextureBufferSerializer.cpp
    TFS.cpp
    TFSPackager.cpp
    Threading.cpp
    ThreeDTilesLayer.cpp
    TileCache.cpp
    TiledFeatureModelLayer.cpp
    TiledModelLayer.cpp
    TileEstimator.cpp
    TileHandler.cpp
    TileIndex.cpp
    TileIndexBuilder.cpp
    TileKey.cpp
    TileLayer.cpp
    TileMesher.cpp
    TileRasterizer.cpp
    TileSource.cpp
    TileSourceElevationLayer.cpp
    TileSourceImageLayer.cpp
    TileVisitor.cpp
    TimeControl.cpp
    TimeSeriesImage.cpp
    TMS.cpp
    TMSBackFiller.cpp
    TopologyGraph.cpp
    TrackNode.cpp
    TransformFilter.cpp
    Units.cpp
    URI.cpp
    Utils.cpp
    UTMGraticule.cpp
    UTMLabelingEngine.cpp
    Version.cpp
    VerticalDatum.cpp
    VideoLayer.cpp
    ViewFitter.cpp
    Viewpoint.cpp
    VirtualProgram.cpp
    VisibleLayer.cpp
    WFS.cpp
    WindLayer.cpp
    WireLines.cpp
    WMS.cpp
    XmlUtils.cpp
    XYZ.cpp
    XYZFeatureSource.cpp
    XYZModelLayer.cpp
    
    ${SHADERS_CPP}
)

if(OSGEARTH_BUILD_LEGACY_CONTROLS_API)
    list(APPEND TARGET_SRC
        ActivityMonitorTool.cpp
        MouseCoordsTool.cpp)
endif()

# MSVC 2017 and 2019 need /bigobj on debug build
if(MSVC AND MSVC_VERSION LESS 1930)
    add_compile_options("$<$<CONFIG:Debug>:/bigobj>")    
endif()

IF(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE)
  IF(NOT OUT_OF_SOURCE_BUILD)
    INCLUDE_DIRECTORIES(${Protobuf_INCLUDE_DIR})
  ELSE()
    INCLUDE_DIRECTORIES(${Protobuf_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
  ENDIF()

  PROTOBUF_GENERATE_CPP(PROTO_VECTOR_TILE_CPP PROTO_VECTOR_TILE_H vector_tile.proto)
  PROTOBUF_GENERATE_CPP(PROTO_GLYPHS_CPP PROTO_GLYPHS_H glyphs.proto)
  SET(LIB_PUBLIC_HEADERS
    ${LIB_PUBLIC_HEADERS}
    ${PROTO_VECTOR_TILE_H}
    ${PROTO_GLYPHS_H}
  )
  SET(TARGET_SRC
    ${TARGET_SRC}
    ${PROTO_VECTOR_TILE_CPP}
    ${PROTO_GLYPHS_CPP}
  )
ENDIF()

if(OSGEARTH_ENABLE_GEOCODER)
    set(TARGET_SRC ${TARGET_SRC} Geocoder.cpp)
    set(LIB_PUBLIC_HEADERS ${LIB_PUBLIC_HEADERS} Geocoder)
endif()

message(STATUS "Building ${LIB_NAME} as a ${OSGEARTH_DYNAMIC_OR_STATIC} library")

ADD_LIBRARY(${LIB_NAME}
    ${OSGEARTH_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${LIB_PUBLIC_HEADERS_IMGUI}
    ${TINYXML_SRC}
    ${VERSION_GIT_SOURCE}
    ${TARGET_SRC}
    ${TARGET_GLSL}
    ${TARGET_IN} )

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

# spdlog support?
if (spdlog_FOUND)
    TARGET_LINK_LIBRARIES(${LIB_NAME} PRIVATE spdlog::spdlog)
    message(STATUS "Linking osgEarth with spdlog")
endif()

# TinyXML support?
IF (TINYXML_FOUND)
    INCLUDE_DIRECTORIES(${TINYXML_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} TINYXML_LIBRARY)
    get_directory_property(output INCLUDE_DIRECTORIES)
    message(STATUS ${output})
ENDIF (TINYXML_FOUND)

# MBTiles support?
IF(SQLITE3_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_SQLITE3)
    INCLUDE_DIRECTORIES(${SQLITE3_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} SQLITE3_LIBRARY)
ENDIF(SQLITE3_FOUND)

# Blosc support?
IF (BLOSC_FOUND)
ADD_DEFINITIONS(-DOSGEARTH_HAVE_BLOSC)
    INCLUDE_DIRECTORIES(${BLOSC_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} BLOSC_LIBRARY)
ENDIF(BLOSC_FOUND)

# GEOS support?
IF (GEOS_FOUND)
    INCLUDE_DIRECTORIES(${GEOS_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} GEOS_LIBRARY)
ENDIF(GEOS_FOUND)

# Protobuf/MVT?
IF(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE)  
  IF (PROTOBUF_USE_DLLS)
    ADD_DEFINITIONS(-DPROTOBUF_USE_DLLS)
  ENDIF()
  LINK_WITH_VARIABLES(${LIB_NAME} Protobuf_LIBRARIES)
ENDIF()

# ESRI FileGeodatabase?
IF(FILEGDB_FOUND)
    add_definitions(-DOSGEARTH_HAVE_FILEGDB)
    include_directories(${FILEGDB_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} FILEGDB_LIBRARY)
ENDIF()

IF(BLEND2D_FOUND)
    include_directories(${BLEND2D_INCLUDES})
    link_with_variables(${LIB_NAME} BLEND2D_LIBRARY)
ENDIF()

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ENDIF(WIN32)

IF (TRACY_FOUND)
    INCLUDE_DIRECTORIES(${TRACY_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} TRACY_LIBRARY)
ENDIF(TRACY_FOUND)

if(UNIX)
    target_link_libraries(${LIB_NAME} PUBLIC Threads::Threads)
endif()

OPTION(NRL_STATIC_LIBRARIES "Link osgEarth against static GDAL and cURL, including static OpenSSL, Proj4, JPEG, PNG, and TIFF." OFF)
if(NOT NRL_STATIC_LIBRARIES)
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGDB_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY CURL_LIBRARY GDAL_LIBRARY OSGMANIPULATOR_LIBRARY)
else(NOT NRL_STATIC_LIBRARIES)
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGDB_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY CURL_LIBRARY GDAL_LIBRARY OSGMANIPULATOR_LIBRARY SSL_EAY_RELEASE LIB_EAY_RELEASE TIFF_LIBRARY PROJ4_LIBRARY PNG_LIBRARY JPEG_LIBRARY)
endif(NOT NRL_STATIC_LIBRARIES)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
