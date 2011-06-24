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

#include <osg/Notify>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/AutoTransform>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarth/FindNode>
#include <osgEarth/ElevationQuery>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ObjectLocator>
#include <osg/Depth>
#include <sstream>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Util;

static osg::Node*     s_flagNode;
static osgText::Text* s_flagText;

static
osg::Node* createFlag()
{
    osg::Geode* g = new osg::Geode();
    osgText::Text* text = new osgText::Text();
    text->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );
    text->setCharacterSize( 24.f );
    text->setFont( osgText::readFontFile("arial.ttf") );
    text->setBackdropType( osgText::Text::OUTLINE );
    text->setText( "00000000000000" );
    text->setAutoRotateToScreen( true );
    text->setPosition( osg::Vec3d( 0, 0, 0 ) );
    text->setDataVariance( osg::Object::DYNAMIC );
    g->addDrawable( text );
    g->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
    g->getStateSet()->setAttribute( new osg::Depth(osg::Depth::ALWAYS), 1 );
    g->getStateSet()->setRenderBinDetails( 99, "RenderBin" );
    g->setDataVariance( osg::Object::DYNAMIC );
    g->setCullingActive( false );
    g->setNodeMask( 0 );
    s_flagText = text;
    s_flagNode = g;
    return g;
}

// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler(const Map* map, ObjectLocator* flagLocator ) 
        : _map(map),
          _mouseDown( false ), 
          _flagLocator(flagLocator), 
          _query(map)
    {
        _query.setMaxTilesToCache(10);
    }

    void update( float x, float y, osgViewer::View* view )
    {
        osgUtil::LineSegmentIntersector::Intersections results;
        if ( view->computeIntersections( x, y, results, 0x01 ) )
        {
            // find the first hit under the mouse:
            osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
            osg::Vec3d point = first.getWorldIntersectPoint();
            
            // transform it to map coordinates:
            _map->worldPointToMapPoint(point, point);

            // find the elevation at that map point:
            osg::Matrixd out_mat;
            double query_resolution = 0.1; // 1/10th of a degree
            double out_elevation = 0.0;
            double out_resolution = 0.0;

            bool ok = _query.getElevation( 
                point,
                _map->getProfile()->getSRS(),
                out_elevation, 
                query_resolution, 
                &out_resolution );

            if ( ok )
            {
                _flagLocator->setPosition( osg::Vec3d(point.x(), point.y(), out_elevation) );
                s_flagNode->setNodeMask( ~0 );

                std::stringstream buf;
                buf << std::fixed << std::setprecision(2) 
                    << "Pos: " << point.x() << ", " << point.y() << std::endl
                    << "Elv: " << out_elevation << "m";
                s_flagText->setText( buf.str() );

                OE_NOTICE << buf.str() << std::endl;
            }
            else
            {
                OE_NOTICE
                    << "getElevation FAILED! at (" << point.x() << ", " << point.y() << ")" << std::endl;
            }

        }
        else
        {
            s_flagNode->setNodeMask(0);
        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == osgGA::GUIEventAdapter::MOVE )
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
        }

        return false;
    }

    const Map*       _map;
    bool             _mouseDown;
    ElevationQuery   _query;
    ObjectLocator*   _flagLocator;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

	osgEarth::MapNode* mapNode = NULL;

	osg::Node* loadedNode = osgDB::readNodeFiles( arguments );
    if (loadedNode)
    {
		mapNode = findTopMostNodeOfType<osgEarth::MapNode>( loadedNode );
    }

    if ( !mapNode )
    {
        OE_WARN << "Unable to load earth file." << std::endl;
        return -1;
    }

    osg::Group* root = new osg::Group();

    // The MapNode will render the Map object in the scene graph.
    mapNode->setNodeMask( 0x01 );
    root->addChild( mapNode );

    // A flag so we can see where we clicked
    ObjectLocatorNode* flag = new ObjectLocatorNode( mapNode->getMap() );
    flag->addChild( createFlag() );
    flag->setNodeMask( 0x02 );
    root->addChild( flag );

    viewer.setSceneData( root );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new QueryElevationHandler( mapNode->getMap(), flag->getLocator() ) );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    // install the programmable manipulator.
    if ( mapNode->getMap()->isGeocentric() )
        viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    return viewer.run();
}
