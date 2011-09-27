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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Annotation>
#include <osgEarthUtil/ImageOverlay>
#include <osgEarthUtil/ImageOverlayEditor>
#include <osgEarthSymbology/GeometryFactory>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Util::Annotation;

int
usage( char** argv )
{
    OE_WARN << "Usage: " << argv[0] << " <earthfile>" << std::endl;
    return -1;
}

struct ToggleNode : public ControlEventHandler {
    ToggleNode( osg::Node* node ) : _node( node ) { }
    void onValueChanged( Control* src, bool value ) {
        if ( _node.valid() )
            _node->setNodeMask( value ? ~0 : 0 );
    }
    osg::observer_ptr<osg::Node> _node;
};

int
main(int argc, char** argv)
{
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

    // make some annotations.
    osg::Group* annoGroup = new osg::Group();
    root->addChild( annoGroup );

    // a Placemark combines a 2D icon with a text label.
    annoGroup->addChild( new PlacemarkNode(
        mapNode, 
        osg::Vec3d(-74, 40.714, 0), 
        URI("../data/placemark32.png").readImage(),
        "New York") );

    // a Placemark combines a 2D icon with a text label.
    annoGroup->addChild( new PlacemarkNode(
        mapNode, 
        osg::Vec3d(139.75, 35.685, 0), 
        URI("../data/placemark32.png").readImage(),
        "Tokyo" ) );

    // a box that follows lines of latitude (rhumb line interpolation, the default)
    Geometry* geom = new Ring();
    geom->push_back( osg::Vec3d(0,   40, 0) );
    geom->push_back( osg::Vec3d(-60, 40, 0) );
    geom->push_back( osg::Vec3d(-60, 60, 0) );
    geom->push_back( osg::Vec3d(0,   60, 0) );
    Style geomStyle;

    geomStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Yellow;
    geomStyle.getOrCreate<LineSymbol>()->stroke()->width() = 5.0f;

    FeatureNode* gnode = new FeatureNode(mapNode, new Feature(geom, geomStyle), true);
    annoGroup->addChild( gnode );

    // another line, this time using great-circle interpolation (flight path from New York to Tokyo)
    Geometry* path = new LineString();
    path->push_back( osg::Vec3d(-74, 40.714, 0) );    // New York
    path->push_back( osg::Vec3d(139.75, 35.685, 0) ); // Tokyo

    Style pathStyle;
    pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Red;
    pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 10.0f;

    Feature* pathFeature = new Feature(path, pathStyle);
    pathFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    FeatureNode* pathNode = new FeatureNode(mapNode, pathFeature, true);
    annoGroup->addChild( pathNode );

    // a circle around New Orleans
    Style circleStyle;
    circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Cyan, 0.5);
    CircleNode* circle = new CircleNode( 
        mapNode, 
        osg::Vec3d(-90.25, 29.98, 0), 
        Linear(600, Units::KILOMETERS), 
        circleStyle );
    annoGroup->addChild( circle );

    // an ellipse around Miami
    Style ellipseStyle;
    ellipseStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Orange, 0.75);
    EllipseNode* ellipse = new EllipseNode(
        mapNode, 
        osg::Vec3d(-80.28,25.82,0), 
        Linear(200, Units::MILES),
        Linear(100, Units::MILES),
        Angular(45, Units::DEGREES),
        ellipseStyle,
        true);
    annoGroup->addChild( ellipse );

    // an extruded polygon roughly the shape of Utah
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

    Feature* utahFeature = new Feature(utah, utahStyle);
    FeatureNode* utahNode = new FeatureNode(mapNode, utahFeature, false);
    annoGroup->addChild( utahNode );

    // an image overlay
    ImageOverlay* imageOverlay = 0L;
    osg::Image* image = osgDB::readImageFile( "../data/USFLAG.TGA" );
    if ( image ) {
        imageOverlay = new ImageOverlay(mapNode, image);
        imageOverlay->setBounds( Bounds( -100.0, 50.0, -90.0, 55.0) );

        //Add an editor            
        annoGroup->addChild( imageOverlay );
        
        osg::Node* editor = new ImageOverlayEditor( imageOverlay, mapNode->getMap()->getProfile()->getSRS()->getEllipsoid(), mapNode );
        root->addChild( editor );



    }




    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.setSceneData( root );

    // make a little HUD to toggle stuff:
    VBox* vbox = new VBox();
    vbox->setBackColor( Color(Color::Black, 0.5) );
    vbox->setVertAlign( Control::ALIGN_TOP );
    vbox->addControl( new LabelControl("Annotation Example", 22.0f, Color::Yellow) );
    Grid* grid = new Grid();
    vbox->addControl( grid );
    grid->setChildSpacing( 5 );
    grid->setChildHorizAlign( Control::ALIGN_LEFT );
    grid->setChildVertAlign( Control::ALIGN_CENTER );
    grid->setControl( 0, 0, new CheckBoxControl(true, new ToggleNode(gnode)) );
    grid->setControl( 1, 0, new LabelControl("Line geoemtry") );
    grid->setControl( 0, 1, new CheckBoxControl(true, new ToggleNode(pathNode)) );
    grid->setControl( 1, 1, new LabelControl("Red flight path") );
    grid->setControl( 0, 2, new CheckBoxControl(true, new ToggleNode(circle)) );
    grid->setControl( 1, 2, new LabelControl("Blue circle") );
    grid->setControl( 0, 3, new CheckBoxControl(true, new ToggleNode(ellipse)) );
    grid->setControl( 1, 3, new LabelControl("Orange ellipse") );
    grid->setControl( 0, 4, new CheckBoxControl(true, new ToggleNode(utahNode)) );
    grid->setControl( 1, 4, new LabelControl("Extruded state") );
    if ( imageOverlay ) {
        grid->setControl( 0, 5, new CheckBoxControl(true, new ToggleNode(imageOverlay)) );
        grid->setControl( 1, 5, new LabelControl("Image overlay") );
    }
    ControlCanvas::get(&viewer,true)->addControl(vbox);

    // add some stock OSG handlers:
    viewer.getDatabasePager()->setDoPreCompile( true );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
