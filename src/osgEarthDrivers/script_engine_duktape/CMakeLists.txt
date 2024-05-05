if(NOT WITH_EXTERNAL_DUKTAPE OR DUKTAPE_FOUND)

    if(WITH_EXTERNAL_DUKTAPE)
        include_directories( ${DUKTAPE_INCLUDE_DIR} )

        set(TARGET_H
            DuktapeEngine
            JSGeometry )

        set(TARGET_SRC
            Plugin.cpp
            DuktapeEngine.cpp )
    else()
        set(TARGET_H
            duktape.h
            duk_config.h
            DuktapeEngine
            JSGeometry )

        set(TARGET_SRC
            Plugin.cpp
            duktape.c
            DuktapeEngine.cpp )
    endif()
    
    add_osgearth_library_as_plugin(
        TARGET osgdb_osgearth_scriptengine_javascript
        HEADERS ${TARGET_H}
        SOURCES ${TARGET_SRC}
        LIBRARIES ${DUKTAPE_LIBRARIES} )

endif(NOT WITH_EXTERNAL_DUKTAPE OR DUKTAPE_FOUND)
