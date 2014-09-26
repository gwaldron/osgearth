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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthAnnotation/PlaceNode>
#include <osgViewer/CompositeViewer>
#include <osgEarthDrivers/gdal/GDALOptions>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;
using namespace osgEarth::Drivers;


/**
 * Makes a simple projected MapNode that contains a basemap of the world
 */
MapNode* makeMiniMapNode( ) {    
    MapOptions mapOpt;
    mapOpt.coordSysType() = MapOptions::CSTYPE_PROJECTED;    
    Map* map = new Map( mapOpt );    

    GDALOptions basemapOpt;
    basemapOpt.url() = "../data/world.tif";
    map->addImageLayer( new ImageLayer( ImageLayerOptions("basemap", basemapOpt) ) );

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNodeOptions mapNodeOptions;
    mapNodeOptions.enableLighting() = false;    

    return new MapNode( map, mapNodeOptions );
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--stencil") )
        osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    //Setup a CompositeViewer
    osgViewer::CompositeViewer viewer(arguments);

    //Setup our main view that will show the loaded earth file.
    osgViewer::View* mainView = new osgViewer::View();
    mainView->getCamera()->setNearFarRatio(0.00002);
    mainView->setCameraManipulator( new EarthManipulator() );      
    mainView->setUpViewInWindow( 50, 50, 800, 800 );
    viewer.addView( mainView );
    
    //Setup a MiniMap View that will be embedded in the main view
    int miniMapWidth = 400;
    int miniMapHeight = 200;
    osgViewer::View* miniMapView = new osgViewer::View();
    miniMapView->getCamera()->setNearFarRatio(0.00002);
    miniMapView->getCamera()->setViewport( 0, 0, miniMapWidth, miniMapHeight);    
    miniMapView->setCameraManipulator( new EarthManipulator() );    
    miniMapView->getCamera()->setClearColor( osg::Vec4(0,0,0,0));
    miniMapView->getCamera()->setProjectionResizePolicy( osg::Camera::FIXED );
    miniMapView->getCamera()->setProjectionMatrixAsPerspective(30.0, double(miniMapWidth) / double(miniMapHeight), 1.0, 1000.0);
    //Share a graphics context with the main view
    miniMapView->getCamera()->setGraphicsContext( mainView->getCamera()->getGraphicsContext());        
    viewer.addView( miniMapView );
    
    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, mainView );
    if ( node )
    {
        //Set the main view's scene data to the loaded earth file
        mainView->setSceneData( node );

        //Setup a group to hold the contents of the MiniMap
        osg::Group* miniMapGroup = new osg::Group;

        MapNode* miniMapNode = makeMiniMapNode();        
        miniMapGroup->addChild( miniMapNode );
       
        //Get the main MapNode so we can do tranformations between it and our minimap
        MapNode* mainMapNode = MapNode::findMapNode( node );
                               
        //Set the scene data for the minimap
        miniMapView->setSceneData( miniMapGroup );        

        //Add a marker we can move around with the main view's eye point
        Style markerStyle;
        markerStyle.getOrCreate<IconSymbol>()->url()->setLiteral( "../data/placemark32.png" );
        PlaceNode* eyeMarker = new PlaceNode(miniMapNode, GeoPoint(miniMapNode->getMapSRS(), 0, 0), "", markerStyle);
        miniMapGroup->addChild( eyeMarker );        

        while (!viewer.done())
        {
            //Reset the viewport so that the camera's viewport is static and doesn't resize with window resizes
            miniMapView->getCamera()->setViewport( 0, 0, miniMapWidth, miniMapHeight);    

            //Get the eye point of the main view
            osg::Vec3d eye, up, center;
            mainView->getCamera()->getViewMatrixAsLookAt( eye, center, up );

            //Turn the eye into a geopoint and transform it to the minimap's SRS
            GeoPoint eyeGeo;
            eyeGeo.fromWorld( mainMapNode->getMapSRS(), eye );
            eyeGeo.transform( miniMapNode->getMapSRS());

            //We want the marker to be positioned at elevation 0, so zero out any elevation in the eye point
            eyeGeo.z() = 0;           
        
            //Set the position of the marker
            eyeMarker->setPosition( eyeGeo );

            viewer.frame();
        }        
    }
    else
    {
        OE_NOTICE 
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }
    return 0;
}
