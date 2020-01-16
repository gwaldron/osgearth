IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

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
OPTION(OSGEARTH_USE_WININET_FOR_HTTP "Whether to use the WinInet library for HTTP requests (instead of cURL)" OFF)
if (OSGEARTH_USE_WININET_FOR_HTTP)
    ADD_DEFINITIONS(-DOSGEARTH_USE_WININET_FOR_HTTP)
endif (OSGEARTH_USE_WININET_FOR_HTTP)

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

# Generate the BuildConfig header
if(CXX11_AVAILABLE)
    set(OSGEARTH_CXX11_DEFINE "#define OSGEARTH_CXX11")
endif()
set(OSGEARTH_BUILDCONFIG_HEADER "${OSGEARTH_BUILDTIME_INCLUDE_DIR}/osgEarth/BuildConfig.h")
message(STATUS "Creating ${OSGEARTH_BUILDCONFIG_HEADER}")
configure_file(
    BuildConfig.h.in
    ${OSGEARTH_BUILDCONFIG_HEADER})

# Name all the library files
SET(LIB_NAME osgEarth)

set(TARGET_GLSL
    CascadeDraping.glsl
    ClipPlane.glsl
    DepthOffset.glsl
    Draping.glsl
    GPUClamping.glsl
    GPUClamping.lib.glsl
    Instancing.glsl
    LineDrawable.glsl
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
)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    BuildConfig.h.in
    Shaders.cpp.in)

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(LIB_PUBLIC_HEADERS
    ArcGISServer
    ArcGISTilePackage
    Async
    Bing
    Bounds
    Cache
    CacheEstimator
    CacheBin
    CachePolicy
    CacheSeed
    Callouts
    CameraUtils
    Capabilities
    CesiumIon
    CascadeDrapingDecorator
    Clamping
    ClampableNode
    ClampingTechnique
    Color
    ColorFilter
    Common
    Composite
    Config
    Containers
    Cube
    CullingUtils
    DateTime
    DateTimeRange
    DecalLayer
    DepthOffset
    DrapeableNode
    DrapingCullSet
    DrapingTechnique
    DrawInstanced
    ECEF
    ElevationLayer
    ElevationLOD
    ElevationPool
    ElevationQuery
    EllipsoidIntersector
    Endian
    Export
    Extension
    FadeEffect
    FileUtils
    GDAL
    GeoCommon
    GeoData
    Geoid
    GeoMath
    GeoTransform
    GeometryClamper
    GLSLChunker
    GLUtils
    HeightFieldUtils
    Horizon
    HorizonClipPlane
    HTTPClient
    ImageLayer
    ImageMosaic
    ImageToHeightFieldConverter
    ImageUtils
    IntersectionPicker
    IOTypes
    JsonUtils
    LandCover
    LandCoverLayer
    Layer
    LayerReference
    LayerShader
    Lighting
    LineDrawable
    LineFunctor
    Locators
    LocalTangentPlane
    Map
    MapCallback
    MapInfo
    MapModelChange
    MapNode
    MapNodeObserver
    MaskLayer
    MaskSource
    Memory
    MemCache
    MetaTile
    Metrics
    MBTiles
    ModelLayer
    ModelSource
    NativeProgramAdapter
    NodeUtils
    Notify
    optional
    ObjectIndex
    OverlayDecorator
    PagedNode
    PatchLayer
    PhongLightingEffect
    Picker
    PluginLoader
    PointDrawable
    PrimitiveIntersector
    Profile
    Progress
    Random
    Registry
    ResourceReleaser
    Revisioning
    SceneGraphCallback
    ScreenSpaceLayout
    ScreenSpaceLayoutImpl
    ScreenSpaceLayoutDeclutter
    ScreenSpaceLayoutCallout
    Shaders
    ShaderFactory
    ShaderGenerator
    ShaderLoader
    ShaderMerger
    ShaderUtils
    SimplexNoise
    SpatialReference
    StateSetCache
    Status
    StringUtils
    TDTiles
    TaskService
    Terrain
    TerrainEffect
    TerrainOptions
    TerrainEngineNode
    TerrainEngineRequirements
    TerrainResources
    TerrainTileModel
    TerrainTileModelFactory
    TerrainTileNode
    Tessellator
    Text
    ThreeDTilesLayer
    TileKey
    TileLayer
    TileHandler
    TileRasterizer
    TileSource
    TileSourceElevationLayer
    TileSourceImageLayer
    TileVisitor
    TileCache
    TimeControl
    TraversalData
    ThreadingUtils
    TMS
    Units
    URI
    Utils
    Version
    VerticalDatum
    VideoLayer
    Viewpoint
    VirtualProgram
    VisibleLayer
    WMS
    XmlUtils
    XYZ

    ActivityMonitorTool
    AutoClipPlaneHandler
    AutoScaleCallback
    AtlasBuilder
    ClipSpace
    Common
    Controls
    ContourMap
    ClampCallback
    ClusterNode
    DebugImageLayer
    EarthManipulator
    Ephemeris
    ExampleResources
    Export
    FlatteningLayer
    Formatter
    FractalElevationLayer
    GARSGraticule
    GeodeticGraticule
    GeodeticLabelingEngine
    GraticuleLabelingEngine
    HTM
    LatLongFormatter
    LineOfSight
    LinearLineOfSight
    LogarithmicDepthBuffer
    MeasureTool
    MGRSFormatter
    MGRSGraticule
    MouseCoordsTool
    RadialLineOfSight
    RTTPicker
    Shaders
    Shadowing
    SimpleOceanLayer
    SimplePager
    Sky
    SkyView
    StarData
    TerrainProfile
    TileIndex
    TileIndexBuilder
    TFSPackager
    TMSBackFiller
    TopologyGraph
    UTMGraticule
    UTMLabelingEngine
    ViewFitter

    AltitudeFilter
    BufferFilter
    BuildGeometryFilter
    BuildTextFilter
    CentroidFilter
    Common
    ConvertTypeFilter
    CropFilter
    ExtrudeGeometryFilter
    Feature
    FeatureCursor
    FeatureDisplayLayout
    FeatureElevationLayer
    FeatureImageLayer
    FeatureIndex
    FeatureMaskLayer
    FeatureModelGraph
    FeatureModelLayer
    FeatureModelSource
    FeatureSource
    FeatureSourceIndexNode
    Filter
    FilterContext
    GeometryCompiler
    GeometryUtils
    ImageToFeatureLayer
    LabelSource
    MVT
    OgrUtils
    OGRFeatureSource
    PolygonizeLines
    ResampleFilter
    ScaleFilter
    Session
    ScatterFilter
    Script
    ScriptEngine
    ScriptFilter
    Shaders
    SubstituteModelFilter
    TessellateOperator
    TextSymbolizer
    TFS
    TransformFilter
    XYZFeatureSource
    WFS

    AGG.h
    AltitudeSymbol
    BBoxSymbol
    BillboardResource
    BillboardSymbol
    Common
    CoverageSymbol
    CssUtils
    Expression
    ExtrusionSymbol
    Fill
    Geometry
    GeometryFactory
    GEOS
    GeometryRasterizer
    IconResource
    IconSymbol
    InstanceResource
    InstanceSymbol
    LineSymbol
    MeshConsolidator
    MeshFlattener
    MeshSubdivider
    ModelResource
    ModelSymbol
    PointSymbol
    PolygonSymbol
    Query
    RenderSymbol
    Resource
    ResourceCache
    ResourceLibrary
    Skins
    Stroke
    Style
    StyleSelector
    StyleSheet
    Symbol
    Tags
    TextSymbol

    AnnotationSettings
    AnnotationData
    AnnotationLayer
    AnnotationNode
    AnnotationRegistry
    AnnotationUtils
	BboxDrawable
    CircleNode
    Common
    Draggers
    EllipseNode
    Export
    FeatureNode
    GeoPositionNode
    GeoPositionNodeAutoScaler
    LocalGeometryNode
    ImageOverlay
    ImageOverlayEditor
    LabelNode
    ModelNode
    PlaceNode
    RectangleNode
    TrackNode

    rtree.h

    FileGDBFeatureSource
    
    ${OSGEARTH_BUILDCONFIG_HEADER}
)


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
IF (OSGEARTH_EMBED_GIT_SHA)
	ADD_DEFINITIONS(-DOSGEARTH_EMBED_GIT_SHA)

	# auto-generate the VersionGit file to include the git SHA1 variable.
	configure_file(
		"${CMAKE_CURRENT_SOURCE_DIR}/VersionGit.cpp.in"
		"${CMAKE_CURRENT_BINARY_DIR}/VersionGit.cpp"
		@ONLY)

	set(VERSION_GIT_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/VersionGit.cpp")
ENDIF (OSGEARTH_EMBED_GIT_SHA)

set(TARGET_SRC
    ArcGISServer.cpp
    ArcGISTilePackage.cpp
    Async.cpp
    Bing.cpp
    Bounds.cpp
    Cache.cpp
    CacheBin.cpp
    CacheEstimator.cpp
    CachePolicy.cpp
    CacheSeed.cpp
    Callouts.cpp
    CameraUtils.cpp
    Capabilities.cpp
    CesiumIon.cpp
    CascadeDrapingDecorator.cpp
    Clamping.cpp
    ClampableNode.cpp
    ClampingTechnique.cpp
    Color.cpp
    ColorFilter.cpp
    Composite.cpp
    Config.cpp
    Cube.cpp
    CullingUtils.cpp
    DateTime.cpp
    DateTimeRange.cpp
    DecalLayer.cpp
    DepthOffset.cpp
    DrapeableNode.cpp
    DrapingCullSet.cpp
    DrapingTechnique.cpp
    DrawInstanced.cpp
    ECEF.cpp
    ElevationLayer.cpp
    ElevationLOD.cpp
    ElevationPool.cpp
    ElevationQuery.cpp
    EllipsoidIntersector.cpp
    Extension.cpp
    FadeEffect.cpp
    FileUtils.cpp
    GDAL.cpp
    GeoData.cpp
    Geoid.cpp
    GeoMath.cpp
    GeoTransform.cpp
    GeometryClamper.cpp
    GLSLChunker.cpp
    GLUtils.cpp
    HeightFieldUtils.cpp
    Horizon.cpp
    HorizonClipPlane.cpp
    HTTPClient.cpp
    ImageLayer.cpp
    ImageMosaic.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    IntersectionPicker.cpp
    IOTypes.cpp
    JsonUtils.cpp
    LandCover.cpp
    LandCoverLayer.cpp
    Layer.cpp
    LayerShader.cpp
    Lighting.cpp
    LineDrawable.cpp
    Locators.cpp
    LocalTangentPlane.cpp
    Map.cpp
    MapCallback.cpp
    MapInfo.cpp
    MapNode.cpp
    MaskLayer.cpp
    MaskSource.cpp
    MemCache.cpp
    Memory.cpp
    MetaTile.cpp
    Metrics.cpp
    MBTiles.cpp
    MimeTypes.cpp
    ModelLayer.cpp
    ModelSource.cpp
    NodeUtils.cpp
    Notify.cpp
    ObjectIndex.cpp
    OverlayDecorator.cpp
    PagedNode.cpp
    PatchLayer.cpp
    PhongLightingEffect.cpp
    PointDrawable.cpp
    PrimitiveIntersector.cpp
    Profile.cpp
    Progress.cpp
    Random.cpp
    Registry.cpp
    ResourceReleaser.cpp
    Revisioning.cpp
    SceneGraphCallback.cpp
    ScreenSpaceLayout.cpp
    ShaderFactory.cpp
    ShaderGenerator.cpp
    ShaderLoader.cpp
    ShaderMerger.cpp
    ShaderUtils.cpp
    SimplexNoise.cpp
    SpatialReference.cpp
    StateSetCache.cpp
    Status.cpp
    StringUtils.cpp
    TaskService.cpp
    TDTiles.cpp
    Terrain.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TerrainResources.cpp
    TerrainTileModel.cpp
    TerrainTileModelFactory.cpp
    Tessellator.cpp
    Text.cpp
    ThreeDTilesLayer.cpp
    TextureBufferSerializer.cpp
    TileKey.cpp
    TileLayer.cpp
    TileHandler.cpp
    TileRasterizer.cpp
    TileVisitor.cpp
    TileSource.cpp
    TileSourceElevationLayer.cpp
    TileSourceImageLayer.cpp
    TileCache.cpp
    TimeControl.cpp
    TraversalData.cpp
    ThreadingUtils.cpp
    TMS.cpp
    Units.cpp
    URI.cpp
    Utils.cpp
    Version.cpp
    VerticalDatum.cpp
    VideoLayer.cpp
    Viewpoint.cpp
    VirtualProgram.cpp
    VisibleLayer.cpp
    WMS.cpp
    XmlUtils.cpp
    XYZ.cpp

    ActivityMonitorTool.cpp
    AtlasBuilder.cpp
    AutoClipPlaneHandler.cpp
    ClampCallback.cpp
    ClipSpace.cpp
    ClusterNode.cpp
    Controls.cpp
    ContourMap.cpp
    DebugImageLayer.cpp
    EarthManipulator.cpp
    Ephemeris.cpp
    ExampleResources.cpp
    FlatteningLayer.cpp
    FractalElevationLayer.cpp
    GARSGraticule.cpp
    GeodeticGraticule.cpp
    GeodeticLabelingEngine.cpp
    GraticuleLabelingEngine.cpp
    HTM.cpp
    LatLongFormatter.cpp
    LinearLineOfSight.cpp
    LogarithmicDepthBuffer.cpp
    MeasureTool.cpp
    MGRSFormatter.cpp
    MGRSGraticule.cpp
    MouseCoordsTool.cpp
    RadialLineOfSight.cpp
    RTTPicker.cpp
    Shadowing.cpp
    SimpleOceanLayer.cpp
    SimplePager.cpp
    Sky.cpp
    SkyView.cpp
    TerrainProfile.cpp
    TileIndex.cpp
    TileIndexBuilder.cpp
    TFSPackager.cpp
    TMSBackFiller.cpp
    TopologyGraph.cpp
    UTMGraticule.cpp
    UTMLabelingEngine.cpp
    ViewFitter.cpp

    AltitudeFilter.cpp
    BufferFilter.cpp
    BuildGeometryFilter.cpp
    BuildTextFilter.cpp
    CentroidFilter.cpp
    ConvertTypeFilter.cpp
    CropFilter.cpp
    ExtrudeGeometryFilter.cpp
    Feature.cpp
    FeatureCursor.cpp
    FeatureDisplayLayout.cpp
    FeatureElevationLayer.cpp
    FeatureImageLayer.cpp
    FeatureMaskLayer.cpp
    FeatureModelGraph.cpp
    FeatureModelLayer.cpp
    FeatureModelSource.cpp
    FeatureSource.cpp
    FeatureSourceIndexNode.cpp
    Filter.cpp
    FilterContext.cpp
    GeometryCompiler.cpp
    GeometryUtils.cpp
    ImageToFeatureLayer.cpp
    LabelSource.cpp
    MVT.cpp
    OgrUtils.cpp
    OGRFeatureSource.cpp
    PolygonizeLines.cpp
    ResampleFilter.cpp
    ScaleFilter.cpp
    Session.cpp
    ScatterFilter.cpp
    ScriptEngine.cpp
    ScriptFilter.cpp
    SubstituteModelFilter.cpp
    TessellateOperator.cpp
    TextSymbolizer.cpp
    TFS.cpp
    TransformFilter.cpp
    XYZFeatureSource.cpp
    WFS.cpp

    AltitudeSymbol.cpp
    BBoxSymbol.cpp
    BillboardResource.cpp
    BillboardSymbol.cpp
    CoverageSymbol.cpp
    CssUtils.cpp
    Expression.cpp
    ExtrusionSymbol.cpp
    Fill.cpp
    Geometry.cpp
    GeometryFactory.cpp
    GEOS.cpp
    GeometryRasterizer.cpp
    IconResource.cpp
    IconSymbol.cpp
    InstanceResource.cpp
    InstanceSymbol.cpp
    LineSymbol.cpp
    MeshConsolidator.cpp
    MeshFlattener.cpp
    MeshSubdivider.cpp
    ModelResource.cpp
    ModelSymbol.cpp
    PointSymbol.cpp
    PolygonSymbol.cpp
    Query.cpp
    RenderSymbol.cpp
    Resource.cpp
    ResourceCache.cpp
    ResourceLibrary.cpp
    Skins.cpp
    Stroke.cpp
    Style.cpp
    StyleSelector.cpp
    StyleSheet.cpp
    Symbol.cpp
    TextSymbol.cpp

    AnnotationSettings.cpp
    AnnotationData.cpp
    AnnotationLayer.cpp
    AnnotationNode.cpp
    AnnotationRegistry.cpp
    AnnotationUtils.cpp
	BboxDrawable.cpp
    CircleNode.cpp
    Draggers.cpp
    EllipseNode.cpp
    FeatureNode.cpp
    GeoPositionNode.cpp
    GeoPositionNodeAutoScaler.cpp
    LocalGeometryNode.cpp
    ImageOverlay.cpp
    ImageOverlayEditor.cpp
    LabelNode.cpp
    RectangleNode.cpp
    ModelNode.cpp
    PlaceNode.cpp
    TrackNode.cpp

    FileGDBFeatureSource.cpp

    ${SHADERS_CPP}
)

IF(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE)
  IF(NOT OUT_OF_SOURCE_BUILD)
    INCLUDE_DIRECTORIES(${Protobuf_INCLUDE_DIR})
  ELSE()
    INCLUDE_DIRECTORIES(${Protobuf_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
  ENDIF()
  
  PROTOBUF_GENERATE_CPP(PROTO_VECTOR_TILE_CPP PROTO_VECTOR_TILE_H vector_tile.proto)
  SET(LIB_PUBLIC_HEADERS
    ${LIB_PUBLIC_HEADERS}
    ${PROTO_VECTOR_TILE_H}
  )
  SET(TARGET_SRC
    ${TARGET_SRC}
    ${PROTO_VECTOR_TILE_CPP}
  )
ENDIF()


ADD_LIBRARY(
    ${LIB_NAME} ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TINYXML_SRC}
    ${VERSION_GIT_SOURCE}
    ${TARGET_SRC}
    ${TARGET_GLSL}
    ${TARGET_IN} )

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

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

# GEOS support?
IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
    INCLUDE_DIRECTORIES(${GEOS_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} GEOS_LIBRARY)
ENDIF(GEOS_FOUND)

# Protobuf/MVT?
IF(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE)

  ADD_DEFINITIONS(-DOSGEARTH_HAVE_MVT)
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

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ENDIF(WIN32)

IF (TRACY_FOUND)
    INCLUDE_DIRECTORIES(${TRACY_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} TRACY_LIBRARY)
ENDIF(TRACY_FOUND)

OPTION(NRL_STATIC_LIBRARIES "Link osgEarth against static GDAL and cURL, including static OpenSSL, Proj4, JPEG, PNG, and TIFF." OFF)
if(NOT NRL_STATIC_LIBRARIES)
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGDB_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY OSGMANIPULATOR_LIBRARY)
else(NOT NRL_STATIC_LIBRARIES)
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGDB_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY OSGMANIPULATOR_LIBRARY SSL_EAY_RELEASE LIB_EAY_RELEASE TIFF_LIBRARY PROJ4_LIBRARY PNG_LIBRARY JPEG_LIBRARY)
endif(NOT NRL_STATIC_LIBRARIES)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})


INCLUDE(ModuleInstall OPTIONAL)
