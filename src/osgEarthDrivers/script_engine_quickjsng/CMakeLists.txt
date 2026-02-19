add_osgearth_plugin(
    TARGET
        osgdb_osgearth_scriptengine_javascript_qjs
    HEADERS
        QuickJSNGEngine
        quickjs.h
        quickjs-libc.h
    SOURCES
        QuickJSNGEngine.cpp
        Plugin.cpp
        quickjs-amalgam.c )
