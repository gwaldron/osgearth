#pragma once 

//This file is used to force the linking of the osg plugins
//as we our doing a static build we can't depend on the loading of the
//dynamic libs to add the plugins to the registries

#include <osgViewer/GraphicsWindow>
#include <osgDB/Registry>
#include <osgEarth/ColorFilter>
#include <osgEarthFeatures/Filter>
#include <osgEarthSymbology/Symbol>
#include <osgEarthAnnotation/AnnotationRegistry>

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
USE_OSGPLUGIN(osgearth_bing)
USE_OSGPLUGIN(osgearth_bumpmap)
USE_OSGPLUGIN(osgearth_cache_filesystem)
USE_OSGPLUGIN(osgearth_colorramp)
USE_OSGPLUGIN(osgearth_debug)
USE_OSGPLUGIN(osgearth_detail)
USE_OSGPLUGIN(earth)
USE_OSGPLUGIN(osgearth_engine_mp)
USE_OSGPLUGIN(osgearth_engine_rex)
USE_OSGPLUGIN(osgearth_feature_elevation)
USE_OSGPLUGIN(osgearth_feature_ogr)
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
USE_OSGPLUGIN(osgearth_osg)
USE_OSGPLUGIN(osgearth_scriptengine_javascript)
USE_OSGPLUGIN(osgearth_sky_gl)
USE_OSGPLUGIN(osgearth_sky_simple)
USE_OSGPLUGIN(osgearth_skyview)
USE_OSGPLUGIN(template)
USE_OSGPLUGIN(osgearth_terrainshader)
USE_OSGPLUGIN(osgearth_tileindex)
USE_OSGPLUGIN(osgearth_tms)
USE_OSGPLUGIN(osgearth_vdatum_egm84)
USE_OSGPLUGIN(osgearth_vdatum_egm96)
USE_OSGPLUGIN(osgearth_vdatum_egm2008)
USE_OSGPLUGIN(osgearth_viewpoints)
USE_OSGPLUGIN(osgearth_vpb)
USE_OSGPLUGIN(osgearth_wcs)
USE_OSGPLUGIN(osgearth_wms)
USE_OSGPLUGIN(osgearth_xyz)

// extensions
USE_OSGPLUGIN(osgearth_bump_map)
USE_OSGPLUGIN(osgearth_contourmap)
USE_OSGPLUGIN(osgearth_contour_map)
USE_OSGPLUGIN(osgearth_lod_blending)
USE_OSGPLUGIN(osgearth_screen_space_layout)
USE_OSGPLUGIN(osgearth_decluttering)

// annotations
USE_OSGEARTH_ANNOTATION(circle)
USE_OSGEARTH_ANNOTATION(ellipse)
USE_OSGEARTH_ANNOTATION(feature)
USE_OSGEARTH_ANNOTATION(imageoverlay)
USE_OSGEARTH_ANNOTATION(label)
USE_OSGEARTH_ANNOTATION(local_geometry)
USE_OSGEARTH_ANNOTATION(place)
USE_OSGEARTH_ANNOTATION(rectangle)

// layers
USE_OSGEARTH_LAYER(annotations)
USE_OSGEARTH_LAYER(elevation)
USE_OSGEARTH_LAYER(feature_mask)
USE_OSGEARTH_LAYER(feature_model)
USE_OSGEARTH_LAYER(feature_source)
USE_OSGEARTH_LAYER(flattened_elevation)
USE_OSGEARTH_LAYER(fractal_elevation)
USE_OSGEARTH_LAYER(image)
USE_OSGEARTH_LAYER(land_cover_dictionary)
USE_OSGEARTH_LAYER(land_cover)
USE_OSGEARTH_LAYER(model)
USE_OSGEARTH_LAYER(multi_elevation)
USE_OSGEARTH_LAYER(simple_ocean)
USE_OSGEARTH_LAYER(video)

// simple symbols
USE_OSGEARTH_SIMPLE_SYMBOL(altitude)
USE_OSGEARTH_SIMPLE_SYMBOL(bbox)
USE_OSGEARTH_SIMPLE_SYMBOL(billboard)
USE_OSGEARTH_SIMPLE_SYMBOL(coverage)
USE_OSGEARTH_SIMPLE_SYMBOL(extrusion)
USE_OSGEARTH_SIMPLE_SYMBOL(icon)
USE_OSGEARTH_SIMPLE_SYMBOL(line)
USE_OSGEARTH_SIMPLE_SYMBOL(model)
USE_OSGEARTH_SIMPLE_SYMBOL(point)
USE_OSGEARTH_SIMPLE_SYMBOL(polygon)
USE_OSGEARTH_SIMPLE_SYMBOL(render)
USE_OSGEARTH_SIMPLE_SYMBOL(skin)
USE_OSGEARTH_SIMPLE_SYMBOL(text)

// simple feature filters
USE_OSGEARTH_SIMPLE_FEATUREFILTER(buffer)
USE_OSGEARTH_SIMPLE_FEATUREFILTER(convert)
USE_OSGEARTH_SIMPLE_FEATUREFILTER(resample)
USE_OSGEARTH_SIMPLE_FEATUREFILTER(script)

// color filters
USE_OSGEARTH_COLORFILTER(brightness_contrast)
USE_OSGEARTH_COLORFILTER(chroma_key)
USE_OSGEARTH_COLORFILTER(cmyk)
USE_OSGEARTH_COLORFILTER(gamma)
USE_OSGEARTH_COLORFILTER(glsl)
USE_OSGEARTH_COLORFILTER(hsl)
USE_OSGEARTH_COLORFILTER(night)
USE_OSGEARTH_COLORFILTER(rgb)

