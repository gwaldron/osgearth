set(CLIPPER2_INCLUDE_DIR "H:/devel/clipper2/install/include")

if (EXISTS ${CLIPPER2_INCLUDE_DIR})

    INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${CLIPPER2_INCLUDE_DIR})

    SET(CLIPPER2_LIB "H:/devel/clipper2/install/lib/clipper2.lib")
    
    SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGGA_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OSGSIM_LIBRARY OSGSHADOW_LIBRARY OSGTEXT_LIBRARY OSGMANIPULATOR_LIBRARY OPENTHREADS_LIBRARY OPENGL_gl_LIBRARY CLIPPER2_LIB)

    SET(TARGET_SRC osgearth_roads.cpp )

    SETUP_IMGUI_APPLICATION(osgearth_roads)
    
endif()