/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgViewer/Viewer>
#include <osg/CullFace>
#include <osgEarth/Notify>
#include <osgEarth/ExampleResources>
#include <osgViewer/GraphicsWindow> // USE_GRAPHICSWINDOW
#include <osgDB/Registry> //USE_OSGPLUGIN
#include <osgEarth/Layer> //USE_OSGEARTH_LAYER
#include "SkyManipulator"

#ifdef  OSGEARTH_LIBRARY_STATIC

//osg plugins

USE_OSGPLUGIN(OpenFlight)
USE_OSGPLUGIN(obj)
USE_OSGPLUGIN(shp)
USE_OSGPLUGIN(ive)

//depreceated osg format
USE_OSGPLUGIN(osg)
USE_DOTOSGWRAPPER_LIBRARY(osg)


USE_OSGPLUGIN(osg2)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_SERIALIZER_WRAPPER_LIBRARY(osgAnimation)

USE_OSGPLUGIN(rot)
USE_OSGPLUGIN(scale)
USE_OSGPLUGIN(trans)


//image files
// #ifndef ANDROID
// USE_OSGPLUGIN(tiff)
// USE_OSGPLUGIN(imageio)
// #else
USE_OSGPLUGIN(png)
USE_OSGPLUGIN(tiff)
USE_OSGPLUGIN(jpeg)
// #endif

USE_OSGPLUGIN(zip) //didn't build on Android for some reason
USE_OSGPLUGIN(curl)
USE_OSGPLUGIN(freetype)
USE_OSGPLUGIN(osgearth_engine_rex)
 

USE_OSGPLUGIN(kml)
USE_OSGPLUGIN(osgearth_sky_simple)
USE_OSGPLUGIN(osgearth_sky_gl)
//USE_OSGPLUGIN(osgearth_feature_wfs)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(wfsfeatures)

//USE_OSGPLUGIN(osgearth_feature_tfs)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(TFSFeatures)


//USE_OSGPLUGIN(osgearth_tms)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(tmsimage)
USE_OSGEARTH_LAYER(tmselevation)

//USE_OSGPLUGIN(osgearth_wms)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(wmsimage)

//USE_OSGPLUGIN(osgearth_label_overlay) 

//USE_OSGPLUGIN(osgearth_xyz)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(xyzimage)
USE_OSGEARTH_LAYER(xyzelevation)
USE_OSGEARTH_LAYER(xyzfeatures)
USE_OSGEARTH_LAYER(XYZModel)

// USE_OSGPLUGIN(osgearth_label_annotation)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(annotations)

// USE_OSGPLUGIN(osgearth_mask_feature)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(featuremask)

// USE_OSGPLUGIN(osgearth_model_feature_geom)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(FeatureModel)
USE_OSGEARTH_LAYER(feature_model)

//USE_OSGPLUGIN(osgearth_agglit)  //causing link errors on android
// USE_OSGPLUGIN(osgearth_feature_ogr)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(ogrfeatures)

// USE_OSGPLUGIN(osgearth_model_feature_stencil)// raw code--commented-out-byAbner

USE_OSGPLUGIN(osgearth_vdatum_egm2008)

// USE_OSGPLUGIN(osgearth_model_simple)// raw code--commented-out-byAbner
// USE_OSGPLUGIN(osgearth_engine_mp)// raw code--commented-out-byAbner

// USE_OSGPLUGIN(osgearth_engine_byo)// raw code--commented-out-byAbner
USE_OSGPLUGIN(osgearth_vdatum_egm96)

//USE_OSGPLUGIN(osgearth_ocean_surface)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(ocean)

// USE_OSGPLUGIN(osgearth_debug)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(debug)
USE_OSGEARTH_LAYER(debugimage)

//USE_OSGPLUGIN(osgearth_mbtiles)
USE_OSGPLUGIN(osgearth_vdatum_egm84)
// USE_OSGPLUGIN(osgearth_tileservice)// raw code--commented-out-byAbner

// USE_OSGPLUGIN(osgearth_yahoo)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(bingimage)
USE_OSGEARTH_LAYER(bingelevation)

// USE_OSGPLUGIN(osgearth_arcgis_map_cache)// raw code--commented-out-byAbner

// USE_OSGPLUGIN(osgearth_tilecache)// raw code--commented-out-byAbner
USE_OSGEARTH_LAYER(tilecacheimage)
USE_OSGEARTH_LAYER(tilecacheelevation)

// USE_OSGPLUGIN(osgearth_wcs)// raw code--commented-out-byAbner

// USE_OSGPLUGIN(osgearth_gdal)// raw code--commented-out-byAbner
USE_OSGPLUGIN(gdal)
USE_OSGEARTH_LAYER(gdalimage)
USE_OSGEARTH_LAYER(gdalelevation)
USE_OSGEARTH_LAYER(GDALDEM)

USE_OSGPLUGIN(earth)
USE_OSGPLUGIN(osgearth_cache_filesystem)

//--- to use  REGISTERed X11 WINDOWINGSYSTEMINTERFACE2
USE_GRAPHICSWINDOW()
#endif//#ifdef  OSGEARTH_LIBRARY_STATIC

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

int
main(int argc, char** argv)
{
    osgEarth::initialize();
    
    osg::setNotifyLevel(osg::DEBUG_FP);
    osgEarth::setNotifyLevel(osg::DEBUG_FP);

    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // Increase the fov to provide a more immersive experience.
    float vfov = 100.0f;
    arguments.read("--vfov", vfov);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    if ( vfov > 0.0 )
    {
        double fov, ar, n, f;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fov, ar, n, f);
        viewer.getCamera()->setProjectionMatrixAsPerspective(vfov, ar, n, f);
    }

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    auto node = MapNodeHelper().load( arguments, &viewer );

    //Set our custom manipulator
    viewer.setCameraManipulator(new SkyManipulator());

    if ( node )
    {
        // Disable backface culling
       node->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);


        viewer.setSceneData( node );
        while(!viewer.done())
        {
            viewer.frame();
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
