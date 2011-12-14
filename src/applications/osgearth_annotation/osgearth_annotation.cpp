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
#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthAnnotation/ImageOverlayEditor>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/GeometryNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarthFeatures/FeatureNode>
#include <osgEarthSymbology/GeometryFactory>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgGA/EventVisitor>
#include <osgUtil/LineSegmentIntersector>

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

struct ToggleNode : public ControlEventHandler {
    ToggleNode( osg::Node* node ) : _node( node ) { }
    void onValueChanged( Control* src, bool value ) {
        if ( _node.valid() )
            _node->setNodeMask( value ? ~0 : 0 );
    }
    osg::observer_ptr<osg::Node> _node;
};



typedef osgUtil::LineSegmentIntersector::Intersections Hits;


struct AnnotationEventCallback : public osg::NodeCallback
{
    std::set<AnnotationNode*> _selected;
    float _mx, _my;

    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
        osgGA::EventVisitor::EventList& events = ev->getEvents();

        osgViewer::View* view = static_cast<osgViewer::View*>(ev->getActionAdapter());
        for( osgGA::EventVisitor::EventList::const_iterator e = events.begin(); e != events.end(); ++e )
        {
            osgGA::GUIEventAdapter* ea = e->get();

            if ( ea->getEventType() == osgGA::GUIEventAdapter::MOVE )
            {
                _mx = ea->getX();
                _my = ea->getY();
                OE_NOTICE << "x=" << _mx << ", y=" << _my << std::endl;
            }

            else if ( ea->getEventType() == osgGA::GUIEventAdapter::FRAME )
            {
                OE_NOTICE << "frame " << ev->getFrameStamp()->getFrameNumber()
                    << ", mx=" << _mx
                    << ", my=" << _my
                    << ", selected=" << _selected.size()
                    << std::endl;

                std::set<AnnotationNode*> toRevert;
                toRevert.swap( _selected );

                osg::NodePath path;
                path.push_back(node);
                Hits hits;

                if ( view->computeIntersections(_mx, _my, path, hits) )
                {
                    OE_NOTICE << "Hits = " << hits.size() << std::endl;
                    for( Hits::const_iterator h = hits.begin(); h != hits.end(); ++h )
                    {
                        const osgUtil::LineSegmentIntersector::Intersection& hit = *h;

                        const osg::NodePath& hitPath = hit.nodePath;

                        for( osg::NodePath::const_reverse_iterator n = hitPath.rbegin(); n != hitPath.rend(); ++n )
                        {
                            AnnotationNode* annode = dynamic_cast<AnnotationNode*>(*n);
                            if ( annode )
                            {
                                _selected.insert( annode );
                                annode->setHighlight( true );
                                toRevert.erase( annode );
                                OE_NOTICE << "Highlighted one" << std::endl;
                                //OE_NOTICE << "Hit an annotation node, stamp = "
                                //    << nv->getFrameStamp()->getFrameNumber() << std::endl;
                                break;
                            }
                        }
                    }
                }                

                for( std::set<AnnotationNode*>::iterator i = toRevert.begin(); i != toRevert.end(); ++i )
                {
                    (*i)->setHighlight( false );
                    OE_NOTICE << "UN-highlighted one" << std::endl;
                }
            }
        }

        traverse(node,nv);
    }

};

struct GeomAnnoNode : public LocalizedNode
{
    GeomAnnoNode( MapNode* mapNode, Geometry* geom )
        : LocalizedNode( mapNode->getMap()->getProfile()->getSRS() )
    {
        LocalGeometryNode* lgn = new LocalGeometryNode( mapNode, geom, Style(), false, getTransform() );
        this->addChild( lgn );
    }
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

    // a scaling selection state technique.
    DrawStateTechnique* scaler = new ScaleDrawStateTechnique();

    // make a group for 2D items, and apply decluttering to it.
    osg::Group* labelGroup = new osg::Group();
    Decluttering::setEnabled( labelGroup->getOrCreateStateSet(), true );
    annoGroup->addChild( labelGroup );

#if 0
    osg::Image* pushpin = osgDB::readImageFile( "../data/placemark32.png" );

    // a Placemark combines a 2D icon with a text label.

    PlaceNode* p1 = new PlaceNode(
        mapNode, 
        osg::Vec3d(-74, 40.714, 0), 
        pushpin,
        "New York" );
    p1->setAltDrawStateTechnique( scaler );
    labelGroup->addChild( p1 );

    labelGroup->addChild( new PlaceNode(
        mapNode, 
        osg::Vec3d(-77.04, 38.85, 0),
        pushpin,
        "Washington, DC") );

    labelGroup->addChild( new LabelNode(
        mapNode, 
        osg::Vec3d(-71.03, 42.37, 0),
        "Boston") );

    labelGroup->addChild( new LabelNode(
        mapNode, 
        osg::Vec3d(139.75, 35.685, 0), 
        "Tokyo" ) );

    // a box that follows lines of latitude (rhumb line interpolation, the default)
    Geometry* geom = new Polygon();
    geom->push_back( osg::Vec3d(0,   40, 0) );
    geom->push_back( osg::Vec3d(-60, 40, 0) );
    geom->push_back( osg::Vec3d(-60, 60, 0) );
    geom->push_back( osg::Vec3d(0,   60, 0) );
    Style geomStyle;

    //geomStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color::Yellow;
    //geomStyle.getOrCreate<LineSymbol>()->stroke()->width() = 5.0f;

    geomStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color::White;

    FeatureNode* gnode = new FeatureNode(mapNode, new Feature(geom, geomStyle), true );

    AnnotationNode* gannode = new AnnotationNode();
    gannode->addChild( gnode );
    gannode->setAltDrawStateTechnique( scaler );
    annoGroup->addChild( gannode );
    //annoGroup->addChild( gnode );
#endif

#if 0

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
    AnnotationNode* pathAnno = new AnnotationNode();
    pathAnno->setAltDrawStateTechnique( scaler );
    pathAnno->addChild( pathNode );
    annoGroup->addChild( pathAnno );

#endif

#if 1
    Geometry* ss = new Polygon();
    ss->push_back( osg::Vec3d(-100000, -100000, 0) );
    ss->push_back( osg::Vec3d( 100000, -100000, 0) );
    ss->push_back( osg::Vec3d( 100000,  100000, 0) );
    ss->push_back( osg::Vec3d(-100000,  100000, 0) );

    GeomAnnoNode* gan = new GeomAnnoNode( mapNode, ss );
    gan->setPosition( osg::Vec3d(0,0,0) );
    AnnotationNode* gan2 = new AnnotationNode();
    gan2->addChild( gan );
    gan2->setAltDrawStateTechnique( scaler );
    annoGroup->addChild( gan2 );
#endif

#if 1
    // a circle around New Orleans
    Style circleStyle;
    circleStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Cyan, 0.5);
    CircleNode* circle = new CircleNode(
        mapNode, 
        osg::Vec3d( -90.25, 29.98, 0 ),
        Linear(300, Units::KILOMETERS ),
        circleStyle,
        false ); //true );
    circle->setAltDrawStateTechnique( scaler );
    annoGroup->addChild( circle );
#endif

#if 0
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
        false );
    ellipse->setAltDrawStateTechnique( scaler );
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
    AnnotationNode* utahAnnoNode = new AnnotationNode();
    utahAnnoNode->addChild( utahNode );
    utahAnnoNode->setAltDrawStateTechnique( scaler );
    annoGroup->addChild( utahAnnoNode );
    //annoGroup->addChild( utahNode );
#endif

#if 0
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
#endif

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.setSceneData( root );

    // install an event handler for interacting with the annotation group.
    annoGroup->addEventCallback( new AnnotationEventCallback() );

#if 0
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
#endif

    // add some stock OSG handlers:
    viewer.getDatabasePager()->setDoPreCompile( true );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
