set(TARGET_LIBRARIES_VARS OSGDB_LIBRARY OSGUTIL_LIBRARY OSGGA_LIBRARY OSG_LIBRARY OSGVIEWER_LIBRARY  OSGSIM_LIBRARY OSGSHADOW_LIBRARY OSGTEXT_LIBRARY OSGMANIPULATOR_LIBRARY OPENTHREADS_LIBRARY OPENGL_gl_LIBRARY)

set(TARGET_SRC osgearth_imgui.cpp)
SETUP_IMGUI_APPLICATION(osgearth_imgui)
