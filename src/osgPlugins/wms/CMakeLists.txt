

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} ${EXPAT_LIBRARY})

SET(TARGET_H
    Capabilities)

SET(TARGET_SRC
    Capabilities.cpp
    ReaderWriterWMS.cpp )

SETUP_PLUGIN(osgearth_wms)
