#pragma once 

//This file is used to force the linking of the osg plugins
//as we our doing a static build we can't depend on the loading of the
//dynamic libs to add the plugins to the registries

#include <osgViewer/GraphicsWindow>
#include <osgDB/Registry>

//windowing system
#ifndef ANDROID
USE_GRAPICSWINDOW_IMPLEMENTATION(IOS)
#endif


//osg plugins
USE_OSGPLUGIN(zip)
USE_OSGPLUGIN(curl)
USE_OSGPLUGIN(freetype)

USE_OSGPLUGIN(tiff)
USE_OSGPLUGIN(rgb)
#ifndef ANDROID
USE_OSGPLUGIN(imageio)
#else
USE_OSGPLUGIN(png)
USE_OSGPLUGIN(jpeg)
USE_OSGPLUGIN(bmp);
#endif

USE_OSGPLUGIN(OpenFlight)
USE_OSGPLUGIN(obj)
USE_OSGPLUGIN(shp)
//USE_OSGPLUGIN(ive)

USE_OSGPLUGIN(osg)
USE_DOTOSGWRAPPER_LIBRARY(osg)
//USE_DOTOSGWRAPPER_LIBRARY(osgAnimation)
USE_DOTOSGWRAPPER_LIBRARY(osgFX)
//USE_DOTOSGWRAPPER_LIBRARY(osgParticle)
USE_DOTOSGWRAPPER_LIBRARY(osgShadow)
USE_DOTOSGWRAPPER_LIBRARY(osgSim)
USE_DOTOSGWRAPPER_LIBRARY(osgTerrain)
USE_DOTOSGWRAPPER_LIBRARY(osgText)
USE_DOTOSGWRAPPER_LIBRARY(osgViewer)
//USE_DOTOSGWRAPPER_LIBRARY(osgVolume)

USE_OSGPLUGIN(osg2)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
//USE_SERIALIZER_WRAPPER_LIBRARY(osgAnimation)
USE_SERIALIZER_WRAPPER_LIBRARY(osgFX)
USE_SERIALIZER_WRAPPER_LIBRARY(osgManipulator)
//USE_SERIALIZER_WRAPPER_LIBRARY(osgParticle)
USE_SERIALIZER_WRAPPER_LIBRARY(osgShadow)
USE_SERIALIZER_WRAPPER_LIBRARY(osgSim)
USE_SERIALIZER_WRAPPER_LIBRARY(osgTerrain)
USE_SERIALIZER_WRAPPER_LIBRARY(osgText)
//USE_SERIALIZER_WRAPPER_LIBRARY(osgUtil)
//USE_SERIALIZER_WRAPPER_LIBRARY(osgViewer)
//USE_SERIALIZER_WRAPPER_LIBRARY(osgVolume)

USE_OSGPLUGIN(rot)
USE_OSGPLUGIN(scale)
USE_OSGPLUGIN(trans)

// osgearth plugins
USE_OSGPLUGIN(osgearth_agglite)
USE_OSGPLUGIN(osgearth_arcgis)
USE_OSGPLUGIN(osgearth_arcgis_map_cache)
USE_OSGPLUGIN(osgearth_bing)
USE_OSGPLUGIN(osgearth_bumpmap)
USE_OSGPLUGIN(osgearth_cache_filesystem)
USE_OSGPLUGIN(osgearth_colorramp)
USE_OSGPLUGIN(osgearth_debug)
USE_OSGPLUGIN(osgearth_detail)
USE_OSGPLUGIN(earth)
USE_OSGPLUGIN(osgearth_engine_byo)
USE_OSGPLUGIN(osgearth_engine_mp)
USE_OSGPLUGIN(osgearth_engine_rex)
USE_OSGPLUGIN(osgearth_feature_elevation)
USE_OSGPLUGIN(osgearth_feature_ogr)
USE_OSGPLUGIN(osgearth_feature_raster)
USE_OSGPLUGIN(osgearth_feature_tfs)
USE_OSGPLUGIN(osgearth_feature_wfs)
USE_OSGPLUGIN(osgearth_feature_xyz)
USE_OSGPLUGIN(osgearth_featurefilter_intersect)
USE_OSGPLUGIN(osgearth_featurefilter_join)
USE_OSGPLUGIN(osgearth_gdal)
USE_OSGPLUGIN(kml)
USE_OSGPLUGIN(osgearth_label_annotation)
USE_OSGPLUGIN(osgearth_mapinspector)
USE_OSGPLUGIN(osgearth_mask_feature)
USE_OSGPLUGIN(osgearth_mbtiles)
USE_OSGPLUGIN(osgearth_model_feature_geom)
USE_OSGPLUGIN(osgearth_model_simple)
USE_OSGPLUGIN(osgearth_monitor)
USE_OSGPLUGIN(osgearth_noise)
USE_OSGPLUGIN(osgearth_ocean_simple)
USE_OSGPLUGIN(osgearth_osg)
USE_OSGPLUGIN(osgearth_quadkey)
USE_OSGPLUGIN(osgearth_refresh)
USE_OSGPLUGIN(osgearth_scriptengine_javascript)
USE_OSGPLUGIN(osgearth_sky_gl)
USE_OSGPLUGIN(osgearth_sky_simple)
USE_OSGPLUGIN(osgearth_skyview)
USE_OSGPLUGIN(osgearth_splat_mask)
USE_OSGPLUGIN(template)
USE_OSGPLUGIN(osgearth_template_matclass)
USE_OSGPLUGIN(osgearth_terrainshader)
USE_OSGPLUGIN(osgearth_tilecache)
USE_OSGPLUGIN(osgearth_tileindex)
USE_OSGPLUGIN(osgearth_tileservice)
USE_OSGPLUGIN(osgearth_tms)
USE_OSGPLUGIN(osgearth_vdatum_egm84)
USE_OSGPLUGIN(osgearth_vdatum_egm96)
USE_OSGPLUGIN(osgearth_vdatum_egm2008)
USE_OSGPLUGIN(osgearth_viewpoints)
USE_OSGPLUGIN(osgearth_vpb)
USE_OSGPLUGIN(osgearth_wcs)
USE_OSGPLUGIN(osgearth_wms)
USE_OSGPLUGIN(osgearth_xyz)
USE_OSGPLUGIN(osgearth_yahoo)

