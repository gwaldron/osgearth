SET(TARGET_SRC ReaderWriterYahoo.cpp)
SET(TARGET_H YahooOptions)
SETUP_PLUGIN(osgearth_yahoo)

# to install public driver includes:
SET(LIB_NAME yahoo)
SET(LIB_PUBLIC_HEADERS YahooOptions)
INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

