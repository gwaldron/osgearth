set(TARGET_H
    DuktapeEngine
    JSGeometry
    duktape.h
    duk_config.h )

set(TARGET_SRC
    Plugin.cpp
    DuktapeEngine.cpp
    duktape.c )

add_osgearth_plugin(
    TARGET osgdb_osgearth_scriptengine_javascript
    HEADERS ${TARGET_H}
    SOURCES ${TARGET_SRC} )
