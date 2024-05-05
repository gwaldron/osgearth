set(WITH_EXTERNAL_DUKTAPE FALSE CACHE BOOL "Use external Duktape instead of inlining it")

if(WITH_EXTERNAL_DUKTAPE)
    find_package(Duktape)
endif()

if(NOT WITH_EXTERNAL_DUKTAPE OR DUKTAPE_FOUND)

    if(WITH_EXTERNAL_DUKTAPE)

        set(TARGET_H
            DuktapeEngine
            JSGeometry )

        set(TARGET_SRC
            Plugin.cpp
            DuktapeEngine.cpp )
    
        add_osgearth_library_as_plugin(
            TARGET osgdb_osgearth_scriptengine_javascript
            HEADERS ${TARGET_H}
            SOURCES ${TARGET_SRC}
            INCLUDE_DIRECTORIES ${DUKTAPE_INCLUDE_DIR}
            LIBRARIES ${DUKTAPE_LIBRARIES} )
        
    else()
        set(TARGET_H
            DuktapeEngine
            JSGeometry
            duktape.h
            duk_config.h )

        set(TARGET_SRC
            Plugin.cpp
            DuktapeEngine.cpp
            duktape.c )
    
        add_osgearth_library_as_plugin(
            TARGET osgdb_osgearth_scriptengine_javascript
            HEADERS ${TARGET_H}
            SOURCES ${TARGET_SRC} )

    endif()
    
endif()
