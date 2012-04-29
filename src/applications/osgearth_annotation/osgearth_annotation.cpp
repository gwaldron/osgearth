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
#include <osgEarthUtil/AnnotationEvents>
#include <osgEarthUtil/AutoClipPlaneHandler>

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
#include <osgEarthAnnotation/Decluttering>
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
    }

    void onHoverLeave( AnnotationNode* anno, const EventArgs& args )
    {
        anno->clearDecoration();
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

    osgViewer::Viewer viewer(arguments);


    // Group to hold all our annotation elements.
    osg::Group* annoGroup = new osg::Group();
    root->addChild( annoGroup );

    //A group for all the editors
    osg::Group* editorGroup = new osg::Group;
    root->addChild( editorGroup );
    editorGroup->setNodeMask( 0 );

    // create a surface to house the controls
    ControlCanvas* cs = ControlCanvas::get( &viewer );
    root->addChild( cs );

    HBox* box = new HBox();    
    box->setChildSpacing( 5 );
    //Add a toggle button to toggle editing
    CheckBoxControl* editCheckbox = new CheckBoxControl( false );    
    editCheckbox->addEventHandler( new ToggleNodeHandler( editorGroup ) );
    box->addControl( editCheckbox );
    LabelControl* labelControl = new LabelControl( "Edit Annotations" );
    labelControl->setFontSize( 24.0f );
    box->addControl( labelControl  );
    cs->addControl( box );


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
    labelStyle.getOrCreate<TextSymbol>()->fill()->color() = Color::Yellow;
    labelStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    //--------------------------------------------------------------------

    // A series of place nodes (an icon with a text label)
    {
        osg::Image* pin = osgDB::readImageFile( "../data/placemark32.png" );

        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -74.00, 40.71), pin, "New York",       placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -77.04, 38.85), pin, "Washington, DC", placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -87.65, 41.90), pin, "Chicago",        placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-118.40, 33.93), pin, "Los Angeles",    placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -71.03, 42.37), pin, "Boston",         placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-157.93, 21.35), pin, "Honolulu",       placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, 139.75, 35.68), pin, "Tokyo",          placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -90.25, 29.98), pin, "New Orleans",    placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS, -80.28, 25.82), pin, "Miami",          placeStyle) );
        labelGroup->addChild( new PlaceNode(mapNode, GeoPoint(geoSRS,-117.17, 32.72), pin, "San Diego",      placeStyle) );
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

        Feature* pathFeature = new Feature(path, geoSRS, pathStyle);
        pathFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;

        //OE_INFO << "Path extent = " << pathFeature->getExtent().toString() << std::endl;

        FeatureNode* pathNode = new FeatureNode(mapNode, pathFeature);
        annoGroup->addChild( pathNode );

        labelGroup->addChild( new LabelNode(mapNode, GeoPoint(geoSRS,-170, 61.2), "Great circle path", labelStyle) );
    }

    //--------------------------------------------------------------------

    // A circle around New Orleans.
    {
        Style circleStyle;
        circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Cyan, 0.5);
        CircleNode* circle = new CircleNode(
            mapNode, 
            GeoPoint(geoSRS, -90.25, 29.98, 1000., AltitudeMode::RELATIVE_TO_TERRAIN),
            Linear(300, Units::KILOMETERS ),
            circleStyle,
            false );
        annoGroup->addChild( circle );        

        editorGroup->addChild( new CircleNodeEditor( circle ) );
    }

    //--------------------------------------------------------------------

    // An draped ellipse around Miami.
    {
        Style ellipseStyle;
        ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Orange, 0.75);
        EllipseNode* ellipse = new EllipseNode(
            mapNode, 
            GeoPoint(geoSRS, -80.28, 25.82, 0.0, AltitudeMode::RELATIVE_TO_TERRAIN),
            Linear(500, Units::MILES),
            Linear(100, Units::MILES),
            Angular(0, Units::DEGREES),
            ellipseStyle,
            true );
        annoGroup->addChild( ellipse );
        editorGroup->addChild( new EllipseNodeEditor( ellipse ) );
    }

    {
        // A rectangle around San Diego
        Style rectStyle;
        rectStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Green, 0.5);
        RectangleNode* rect = new RectangleNode(
            mapNode, 
            GeoPoint(geoSRS, -117.172, 32.721),
            Linear(300, Units::KILOMETERS ),
            Linear(600, Units::KILOMETERS ),
            rectStyle,
            true );
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

    // an image overlay
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
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.setSceneData( root );

    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode) );
    viewer.getDatabasePager()->setDoPreCompile( true );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    // testing:
    Config annoConfig = AnnotationRegistry::instance()->getConfig( annoGroup );
    mapNode->externalConfig().add(annoConfig);
    osgDB::writeNodeFile( *mapNode, "out.earth" );

    return viewer.run();
}
