/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/ECEF>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Formatters>
#include <osgEarthUtil/AnnotationEvents>
#include <osgEarthUtil/AutoClipPlaneHandler>

#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthAnnotation/ImageOverlayEditor>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/GeometryNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarthAnnotation/HighlightDecoration>
#include <osgEarthAnnotation/ScaleDecoration>

#include <osgEarthSymbology/GeometryFactory>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgGA/EventVisitor>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

int
usage( char** argv )
{
    OE_WARN << "Usage: " << argv[0] << " <earthfile>" << std::endl;
    return -1;
}

//------------------------------------------------------------------

/**
 * Event handler that processes events fired from the
 * AnnotationEventCallback
 */
struct MyAnnoEventHandler : public AnnotationEventHandler
{
    void onHoverEnter( AnnotationNode* anno, const EventArgs& args )
    {
        anno->setDecoration( "hover" );
    }

    void onHoverLeave( AnnotationNode* anno, const EventArgs& args )
    {
        anno->clearDecoration();
    }
};

//------------------------------------------------------------------

int
main(int argc, char** argv)
{
    // The HighlightDecoration requires you to allocate stencil planes,
    // and will yell at you if you don't. You have to do this prior to creating
    // your Viewer.
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 2 );

    osg::Group* root = new osg::Group();

    // try to load an earth file.
    osg::ArgumentParser arguments(&argc,argv);
    osg::Node* node = osgDB::readNodeFiles( arguments );
    if ( !node )
        return usage(argv);

    // find the map node that we loaded.
    MapNode* mapNode = MapNode::findMapNode(node);
    if ( !mapNode )
        return usage(argv);

    root->addChild( mapNode );


    // Group to hold all our annotation elements.
    osg::Group* annoGroup = new osg::Group();
    root->addChild( annoGroup );


    // Make a group for 2D items, and activate the decluttering engine. Decluttering
    // will migitate overlap between elements that occupy the same screen real estate.
    osg::Group* labelGroup = new osg::Group();
    Decluttering::setEnabled( labelGroup->getOrCreateStateSet(), true );
    annoGroup->addChild( labelGroup );
    
    // set up a style to use for labels:
    Style placeStyle;
    placeStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    Style labelStyle;
    labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    labelStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    //--------------------------------------------------------------------

    // A series of place nodes (an icon with a text label)
    {
        osg::Image* pushpin = osgDB::readImageFile( "../data/placemark32.png" );

        labelGroup->addChild( new PlaceNode(mapNode,  -74.00, 40.71, pushpin, "New York",       placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode,  -77.04, 38.85, pushpin, "Washington, DC", placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode,  -87.65, 41.90, pushpin, "Chicago",        placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, -118.40, 33.93, pushpin, "Los Angeles",    placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode,  -71.03, 42.37, pushpin, "Boston",         placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, -157.93, 21.35, pushpin, "Honolulu",       placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode,  138.75, 35.68, pushpin, "Tokyo",          placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode,  -90.25, 29.98, pushpin, "New Orleans",    placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode,  -80.28, 25.82, pushpin, "Miami",          placeStyle) );
    }

    //--------------------------------------------------------------------

    // a box that follows lines of latitude (rhumb line interpolation, the default)
    {
        Geometry* geom = new Polygon();
        geom->push_back( osg::Vec3d(0,   40, 0) );
        geom->push_back( osg::Vec3d(-60, 40, 0) );
        geom->push_back( osg::Vec3d(-60, 60, 0) );
        geom->push_back( osg::Vec3d(0,   60, 0) );
        Style geomStyle;
        geomStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Cyan;
        geomStyle.getOrCreate<LineSymbol>()->stroke()->width() = 5.0f;
        geomStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        FeatureNode* gnode = new FeatureNode(mapNode, new Feature(geom, mapNode->getMapSRS(), geomStyle));
        annoGroup->addChild( gnode );

        labelGroup->addChild( new LabelNode(mapNode, -30, 50, "Rhumb line polygon", labelStyle) );
    }

    //--------------------------------------------------------------------

    // another rhumb box that crosses the antimeridian
    {
        Geometry* geom = new Polygon();
        geom->push_back( -160., -30. );
        geom->push_back(  150., -20. );
        geom->push_back(  160., -45. );
        geom->push_back( -150., -40. );
        Style geomStyle;
        geomStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Lime;
        geomStyle.getOrCreate<LineSymbol>()->stroke()->width() = 3.0f;
        geomStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        FeatureNode* gnode = new FeatureNode(mapNode, new Feature(geom, mapNode->getMapSRS(), geomStyle));
        annoGroup->addChild( gnode );

        labelGroup->addChild( new LabelNode(mapNode, -175, -35, "Antimeridian polygon", labelStyle) );
    }

    //--------------------------------------------------------------------

    // A path using great-circle interpolation.
    {
        Geometry* path = new LineString();
        path->push_back( osg::Vec3d(-74, 40.714, 0) );    // New York
        path->push_back( osg::Vec3d(139.75, 35.685, 0) ); // Tokyo

        Style pathStyle;
        pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Red;
        pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 3.0f;
        pathStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

        Feature* pathFeature = new Feature(path, mapNode->getMapSRS(), pathStyle);
        pathFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;

        //OE_INFO << "Path extent = " << pathFeature->getExtent().toString() << std::endl;

        FeatureNode* pathNode = new FeatureNode(mapNode, pathFeature);
        annoGroup->addChild( pathNode );

        labelGroup->addChild( new LabelNode(mapNode, -170, 61.2, "Great circle path", labelStyle) );
    }

    //--------------------------------------------------------------------

    // A circle around New Orleans.
    {
        Style circleStyle;
        circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Cyan, 0.5);
        CircleNode* circle = new CircleNode(
            mapNode, 
            osg::Vec3d( -90.25, 29.98, 0 ),
            Linear(300, Units::KILOMETERS ),
            circleStyle,
            false );
        annoGroup->addChild( circle );
    }

    //--------------------------------------------------------------------

    // An draped ellipse around Miami.
    {
        Style ellipseStyle;
        ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Orange, 0.75);
        EllipseNode* ellipse = new EllipseNode(
            mapNode, 
            osg::Vec3d(-80.28,25.82,0), 
            Linear(200, Units::MILES),
            Linear(100, Units::MILES),
            Angular(45, Units::DEGREES),
            ellipseStyle,
            true );
        annoGroup->addChild( ellipse );
    }

    //--------------------------------------------------------------------

    // An extruded polygon roughly the shape of Utah. Here we demonstrate the
    // FeatureNode, where you create a geographic geometry and use it as an
    // annotation.
    {
        Geometry* utah = new Polygon();
        utah->push_back( osg::Vec3d(-114.052, 37, 0) );
        utah->push_back( osg::Vec3d(-109.054, 37, 0) );
        utah->push_back( osg::Vec3d(-109.054, 41, 0) );
        utah->push_back( osg::Vec3d(-111.04, 41, 0) );
        utah->push_back( osg::Vec3d(-111.08, 42.059, 0) );
        utah->push_back( osg::Vec3d(-114.08, 42.024, 0) );

        Style utahStyle;
        utahStyle.getOrCreate<ExtrusionSymbol>()->height() = 250000.0; // meters MSL
        utahStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::White, 0.8);

        Feature*     utahFeature = new Feature(utah, mapNode->getMapSRS(), utahStyle);
        FeatureNode* featureNode = new FeatureNode(mapNode, utahFeature);
        annoGroup->addChild( featureNode );
    }

    //--------------------------------------------------------------------

    // an image overlay
    {
        ImageOverlay* imageOverlay = 0L;
        osg::Image* image = osgDB::readImageFile( "E:/devel/osgearth/2.x/repo/data/USFLAG.TGA" );
        if ( image )
        {
            imageOverlay = new ImageOverlay(mapNode, image);
            imageOverlay->setBounds( Bounds( -100.0, 35.0, -90.0, 40.0) );
            annoGroup->addChild( imageOverlay );
        }
    }
    
    //--------------------------------------------------------------------

    // install decoration. These change the appearance of an Annotation
    // based on some user action.

    // highlight annotation upon hover by default:
    
    DecorationInstaller highlightInstaller("hover", new HighlightDecoration());
    annoGroup->accept( highlightInstaller );

    // scale labels when hovering:
    DecorationInstaller scaleInstaller("hover", new ScaleDecoration(1.1f));
    labelGroup->accept( scaleInstaller );

    // install an event handler for picking and hovering.
    AnnotationEventCallback* cb = new AnnotationEventCallback();
    cb->addHandler( new MyAnnoEventHandler() );

    annoGroup->addEventCallback( cb );

    //--------------------------------------------------------------------

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.setSceneData( root );

    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode->getMap()) );
    viewer.getDatabasePager()->setDoPreCompile( true );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
