/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/MapNode>
#include <osgEarth/Decluttering>
#include <osgEarth/ECEF>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AnnotationEvents>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/ExampleResources>

#include <osgEarthAnnotation/AnnotationEditing>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthAnnotation/ImageOverlayEditor>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/RectangleNode>
#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/PlaceNode>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgGA/EventVisitor>


using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Util;

int
usage( char** argv )
{
    OE_WARN << "Usage: " << argv[0] << " <earthfile>" << std::endl;
    return -1;
}


//------------------------------------------------------------------


int
main(int argc, char** argv)
{
    osg::Group* root = new osg::Group();

    // try to load an earth file.
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    unsigned int numObjects = 200;
    while (arguments.read("--count", numObjects)) {}

    bool declutter = false;
    if (arguments.read("--declutter")) declutter = true;
    
    // initialize the viewer:    
    viewer.setCameraManipulator( new EarthManipulator() );


    // load an earth file and parse demo arguments
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( !node )
        return usage(argv);

    // find the map node that we loaded.
    MapNode* mapNode = MapNode::findMapNode(node);
    if ( !mapNode )
        return usage(argv);

    root->addChild( node );
   
    // Make a group for 2D items, and activate the decluttering engine. Decluttering
    // will migitate overlap between elements that occupy the same screen real estate.
    osg::Group* labelGroup = new osg::Group();
    if (declutter)
    {
        Decluttering::setEnabled( labelGroup->getOrCreateStateSet(), true );
    }
    root->addChild( labelGroup );
    
    // set up a style to use for placemarks:
    Style placeStyle;
    placeStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    //--------------------------------------------------------------------

    //Create a bunch of placemarks around Mt Rainer so we can actually get some elevation
    {
        osg::Image* pin = osgDB::readImageFile( "../data/placemark32.png" );

        double centerLat =  46.840866;
        double centerLon = -121.769846;
        double height = 0.2;
        double width = 0.2;
        double minLat = centerLat - (height/2.0);
        double minLon = centerLon - (width/2.0);

        OE_NOTICE << "Placing " << numObjects << " placemarks" << std::endl;

        for (unsigned int i = 0; i < numObjects; i++)
        {
            double lat = minLat + height * (rand() * 1.0)/(RAND_MAX-1);
            double lon = minLon + width * (rand() * 1.0)/(RAND_MAX-1);        
            PlaceNode* place = new PlaceNode(mapNode, GeoPoint(geoSRS, lon, lat), pin, "Placemark", placeStyle);
            //Enable occlusion culling.  This will hide placemarks that are hidden behind terrain.
            //This makes use of the OcclusionCullingCallback in CullingUtils.
            place->setOcclusionCulling( true );
            labelGroup->addChild( place );
        }    
    }

    viewer.setSceneData( root );

    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode) );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
