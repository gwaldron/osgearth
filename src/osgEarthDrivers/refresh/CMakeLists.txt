SET(TARGET_SRC
  ReaderWriterRefresh.cpp
)
SET(TARGET_H
  RefreshOptions
)

SETUP_PLUGIN(osgearth_refresh)

# to install public driver includes:
SET(LIB_NAME refresh)
SET(LIB_PUBLIC_HEADERS RefreshOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
