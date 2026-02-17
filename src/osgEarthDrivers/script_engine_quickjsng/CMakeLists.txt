set(TARGET_H   QuickJSNGEngine)
set(TARGET_SRC QuickJSNGEngine.cpp Plugin.cpp)

add_osgearth_plugin(
    TARGET
        osgdb_osgearth_scriptengine_javascript
    HEADERS
        QuickJSNGEngine
        quickjs.h
        quickjs-libc.h
    SOURCES
        QuickJSNGEngine.cpp
        Plugin.cpp
        quickjs-amalgam.c )
