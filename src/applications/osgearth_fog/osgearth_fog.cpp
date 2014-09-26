/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Fog>
#include <osg/Fog>


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
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    if ( arguments.read("--stencil") )
        osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    //Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );    
    if ( node )
    {
        MapNode* mapNode = MapNode::findMapNode( node );

        FogEffect* fogEffect = new FogEffect;
        fogEffect->attach( node->getOrCreateStateSet() );
        
        float maxDensity = 0.000125; 
        float fogStartHeight = 10000.0f;        

        // Setup a Fog state attribute
        osg::Vec4 fogColor(0.66f, 0.7f, 0.81f, 1.0f);
        osg::Fog* fog = new osg::Fog;        
        fog->setColor( fogColor ); //viewer.getCamera()->getClearColor() );                
        fog->setDensity( 0 );
        node->getOrCreateStateSet()->setAttributeAndModes( fog, osg::StateAttribute::ON );                

        viewer.setSceneData( node );

        // configure the near/far so we don't clip things that are up close
        viewer.getCamera()->setNearFarRatio(0.00002);
        viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

        while (!viewer.done())
        {
            // Get the height above the terrain
            osg::Vec3d eye, center, up;
            viewer.getCamera()->getViewMatrixAsLookAt(eye,center, up);
            GeoPoint map;
            map.fromWorld( mapNode->getMapSRS(), eye );            
            
            // Compute the fog density based on the camera height
            float ratio = ((fogStartHeight - map.z()) / fogStartHeight);
            ratio = osg::clampBetween(ratio, 0.0f, 1.0f);
            float density = ratio * maxDensity;
            fog->setDensity( density );            
            viewer.frame();
        }        
    }
    else
    {
        return usage(argv[0]);
    }
    return 0;
}
