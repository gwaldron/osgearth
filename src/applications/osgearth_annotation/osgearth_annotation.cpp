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
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/HighlightDecoration>
#include <osgEarthAnnotation/ScaleDecoration>

#include <osgEarthSymbology/GeometryFactory>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgGA/EventVisitor>
#include <osgDB/WriteFile>

#include <osgEarth/Pickers>

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
        OE_NOTICE << "Hover enter" << std::endl;
    }

    void onHoverLeave( AnnotationNode* anno, const EventArgs& args )
    {
        anno->clearDecoration();
        OE_NOTICE << "Hover leave" << std::endl;
    }

    virtual void onClick( AnnotationNode* node, const EventArgs& details )
    {        
        PlaceNode* place = dynamic_cast<PlaceNode*>(node);
        if (place == NULL)
        {
            OE_NOTICE << "Thanks for clicking this annotation" << std::endl;
        }
        else
        {
            OE_NOTICE << "Thanks for clicking the PlaceNode: " << place->getText() << std::endl;
        }
    }
};

//------------------------------------------------------------------

struct ToggleNodeHandler : public ControlEventHandler
{
    ToggleNodeHandler( osg::Node* node ) : _node(node) { }

    void onValueChanged( Control* control, bool value )
    {
        _node->setNodeMask( value ? ~0 : 0 );
    }

    osg::Node* _node;
};

//------------------------------------------------------------------


int
main(int argc, char** argv)
{
    osg::Group* root = new osg::Group();

    // try to load an earth file.
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file and parse demo arguments
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( !node )
        return usage(argv);

    root->addChild( node );

    // find the map node that we loaded.
    MapNode* mapNode = MapNode::findMapNode(node);
    if ( !mapNode )
        return usage(argv);

    // Group to hold all our annotation elements.
    osg::Group* annoGroup = new osg::Group();
    root->addChild( annoGroup );

    //A group for all the editors
    osg::Group* editorGroup = new osg::Group;
    root->addChild( editorGroup );
    editorGroup->setNodeMask( 0 );

    HBox* box = ControlCanvas::getOrCreate(&viewer)->addControl( new HBox() );
    box->setChildSpacing( 5 );
    //Add a toggle button to toggle editing
    CheckBoxControl* editCheckbox = new CheckBoxControl( false );
    editCheckbox->addEventHandler( new ToggleNodeHandler( editorGroup ) );
    box->addControl( editCheckbox );
    LabelControl* labelControl = new LabelControl( "Edit Annotations" );
    labelControl->setFontSize( 24.0f );
    box->addControl( labelControl  );


    // Make a group for 2D items, and activate the decluttering engine. Decluttering
    // will migitate overlap between elements that occupy the same screen real estate.
    osg::Group* labelGroup = new osg::Group();
    Decluttering::setEnabled( labelGroup->getOrCreateStateSet(), true );
    annoGroup->addChild( labelGroup );

    // Style our labels:
    Style labelStyle;
    labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    labelStyle.getOrCreate<TextSymbol>()->fill()->color() = Color::Yellow;

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    //--------------------------------------------------------------------

    // A series of place nodes (an icon with a text label)
    {
        Style pin;
        pin.getOrCreate<IconSymbol>()->url()->setLiteral( "../data/placemark32.png" );

        // bunch of pins:
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -74.00, 40.71), "New York"      , pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -77.04, 38.85), "Washington, DC", pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-118.40, 33.93), "Los Angeles"   , pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -71.03, 42.37), "Boston"        , pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-157.93, 21.35), "Honolulu"      , pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, 139.75, 35.68), "Tokyo"         , pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -90.25, 29.98), "New Orleans"   , pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -80.28, 25.82), "Miami"         , pin));
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-117.17, 32.72), "San Diego"     , pin));

        // test with an LOD:
        osg::LOD* lod = new osg::LOD();
        lod->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, 14.68, 50.0), "Prague", pin), 0.0, 1e6);
        labelGroup->addChild( lod );

        // absolute altitude:
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -87.65, 41.90, 1000, ALTMODE_ABSOLUTE), "Chicago"       , pin));
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
        geomStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;
        FeatureNode* gnode = new FeatureNode(mapNode, new Feature(geom, geoSRS, geomStyle));
        annoGroup->addChild( gnode );

        labelGroup->addChild( new LabelNode(mapNode, GeoPoint(geoSRS,-30, 50), "Rhumb line polygon", labelStyle) );
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
        geomStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;
        FeatureNode* gnode = new FeatureNode(mapNode, new Feature(geom, geoSRS, geomStyle));
        annoGroup->addChild( gnode );

        labelGroup->addChild( new LabelNode(mapNode, GeoPoint(geoSRS, -175, -35), "Antimeridian polygon", labelStyle) );
    }

    //--------------------------------------------------------------------

    // A path using great-circle interpolation.
    {
        Geometry* path = new LineString();
        path->push_back( osg::Vec3d(-74, 40.714, 0) );   // New York
        path->push_back( osg::Vec3d(139.75, 35.68, 0) ); // Tokyo

        Style pathStyle;
        pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Red;
        pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 3.0f;
        pathStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        pathStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;

        Feature* pathFeature = new Feature(path, geoSRS, pathStyle);
        pathFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;

        //OE_INFO << "Path extent = " << pathFeature->getExtent().toString() << std::endl;

        FeatureNode* pathNode = new FeatureNode(mapNode, pathFeature);
        annoGroup->addChild( pathNode );

        labelGroup->addChild( new LabelNode(mapNode, GeoPoint(geoSRS,-170, 61.2), "Great circle path", labelStyle) );
    }

    //--------------------------------------------------------------------

    // Two circle segments around New Orleans.
    {
        Style circleStyle;
        circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Cyan, 0.5);
        circleStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        circleStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

        CircleNode* circle = new CircleNode(
            mapNode,
            GeoPoint(geoSRS, -90.25, 29.98, 1000., ALTMODE_RELATIVE),
            Distance(300, Units::KILOMETERS),
            circleStyle, Angle(-45.0, Units::DEGREES), Angle(45.0, Units::DEGREES), true);
        annoGroup->addChild( circle );

        editorGroup->addChild( new CircleNodeEditor( circle ) );
    }

	{
		Style circleStyle;
		circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Red, 0.5);
		circleStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
		circleStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

		CircleNode* circle = new CircleNode(
			mapNode,
			GeoPoint(geoSRS, -90.25, 29.98, 1000., ALTMODE_RELATIVE),
			Distance(300, Units::KILOMETERS),
			circleStyle, Angle(45.0, Units::DEGREES), Angle(360.0 - 45.0, Units::DEGREES), true);
		annoGroup->addChild( circle );

		editorGroup->addChild( new CircleNodeEditor( circle ) );
	}

    //--------------------------------------------------------------------

    // An extruded ellipse around Miami.
    {
        Style ellipseStyle;
        ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Orange, 0.75);
        ellipseStyle.getOrCreate<ExtrusionSymbol>()->height() = 250000.0; // meters MSL
        EllipseNode* ellipse = new EllipseNode(
            mapNode, 
            GeoPoint(geoSRS, -80.28, 25.82, 0.0, ALTMODE_RELATIVE),
            Distance(250, Units::MILES),
            Distance(100, Units::MILES),
            Angle   (0, Units::DEGREES),
            ellipseStyle,
            Angle(45.0, Units::DEGREES),
            Angle(360.0 - 45.0, Units::DEGREES), 
            true);
        annoGroup->addChild( ellipse );
        editorGroup->addChild( new EllipseNodeEditor( ellipse ) );
    }
	{
		Style ellipseStyle;
		ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Blue, 0.75);
		ellipseStyle.getOrCreate<ExtrusionSymbol>()->height() = 250000.0; // meters MSL
		EllipseNode* ellipse = new EllipseNode(
			mapNode, 
			GeoPoint(geoSRS, -80.28, 25.82, 0.0, ALTMODE_RELATIVE),
			Distance(250, Units::MILES),
			Distance(100, Units::MILES),
			Angle   (0, Units::DEGREES),
			ellipseStyle, 
            Angle(-40.0, Units::DEGREES), 
            Angle(40.0, Units::DEGREES), 
            true);
		annoGroup->addChild( ellipse );
		editorGroup->addChild( new EllipseNodeEditor( ellipse ) );
	}
    
    //--------------------------------------------------------------------

    {
        // A rectangle around San Diego
        Style rectStyle;
        rectStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Green, 0.5);
        rectStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        rectStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
        RectangleNode* rect = new RectangleNode(
            mapNode, 
            GeoPoint(geoSRS, -117.172, 32.721),
            Distance(300, Units::KILOMETERS ),
            Distance(600, Units::KILOMETERS ),
            rectStyle);
        annoGroup->addChild( rect );

        editorGroup->addChild( new RectangleNodeEditor( rect ) );
    }    

    //--------------------------------------------------------------------

    // An extruded polygon roughly the shape of Utah. Here we demonstrate the
    // FeatureNode, where you create a geographic geometry and use it as an
    // annotation.
    {
        Geometry* utah = new Polygon();
        utah->push_back( -114.052, 37.0   );
        utah->push_back( -109.054, 37.0   );
        utah->push_back( -109.054, 41.0   );
        utah->push_back( -111.040, 41.0   );
        utah->push_back( -111.080, 42.059 );
        utah->push_back( -114.080, 42.024 );

        Style utahStyle;
        utahStyle.getOrCreate<ExtrusionSymbol>()->height() = 250000.0; // meters MSL
        utahStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::White, 0.8);

        Feature*     utahFeature = new Feature(utah, geoSRS, utahStyle);
        FeatureNode* featureNode = new FeatureNode(mapNode, utahFeature);
        annoGroup->addChild( featureNode );
    }

    //--------------------------------------------------------------------

    // an image overlay.
    {
        ImageOverlay* imageOverlay = 0L;
        osg::Image* image = osgDB::readImageFile( "../data/USFLAG.TGA" );
        if ( image )
        {
            imageOverlay = new ImageOverlay(mapNode, image);
            imageOverlay->setBounds( Bounds( -100.0, 35.0, -90.0, 40.0) );
            annoGroup->addChild( imageOverlay );

            editorGroup->addChild( new ImageOverlayEditor( imageOverlay ) );
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

    // initialize the viewer:    
    viewer.setSceneData( root );

    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode) );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
