#
# osgEarth ImGui NodeKit
#
find_package(GLEW REQUIRED)

set(OSGEARTH_HAVE_IMGUI_NODEKIT ON PARENT_SCOPE) # for BuildConfig

set(LIB_NAME osgEarthImGui)

# This forces the export of all ImGui symbols on Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# embedded ImGui source code:
set(IMGUI_HEADERS imgui.h imgui_internal.h imgui_impl_opengl3.h imconfig.h)
set(IMGUI_SOURCES imgui.cpp imgui_demo.cpp imgui_draw.cpp imgui_tables.cpp imgui_widgets.cpp imgui_impl_opengl3.cpp)

set(STOCK_PANELS
    AnnotationsGUI
    CameraGUI
    ContentBrowserGUI
    #DateTimeGUI
    EnvironmentGUI    
    LayersGUI
    NetworkMonitorGUI
    NotifyGUI
    OpenEarthFileGUI
    LiveCamerasGUI
    PickerGUI
    RenderingGUI
    SceneGraphGUI
    SearchGUI
    ShaderGUI
    SystemGUI
    TerrainGUI
    TextureInspectorGUI
    ViewpointsGUI
    
    # osgEarthProcedural
    LifeMapLayerGUI
    TerrainEditGUI
    TextureSplattingLayerGUI
    VegetationLayerGUI
    
    # osgEarthCesium
    CesiumIonGUI
)

# osgEarth source code:
set(LIB_HEADERS
    Common
    ImGuiEventHandler
    ImGuiApp
    ImGuiPanel )
    
set(LIB_SOURCES 
    ImGuiEventHandler.cpp )

add_osgearth_library(
    TARGET ${LIB_NAME}
    SOURCES ${LIB_SOURCES} ${IMGUI_SOURCES}
    PUBLIC_HEADERS ${LIB_HEADERS} ${IMGUI_HEADERS} ${STOCK_PANELS}
    LIBRARIES PUBLIC GLEW::GLEW OpenGL::GL
    FOLDER "NodeKits")
