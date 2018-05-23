#pragma once 

//This file is used to force the linking of the osg plugins
//as we our doing a static build we can't depend on the loading of the
//dynamic libs to add the plugins to the registries

USE_OSGPLUGIN(osg)
USE_OSGPLUGIN(curl)
//USE_OSGPLUGIN(tiff)
USE_OSGPLUGIN(jpeg)
USE_OSGPLUGIN(earth)
USE_OSGPLUGIN(osgearth_tms)
USE_OSGPLUGIN(osgearth_engine_mp)
USE_OSGPLUGIN(osgearth_engine_rex)

USE_DOTOSGWRAPPER_LIBRARY(osg)

