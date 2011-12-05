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

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarth/ElevationQuery>
#include <osgEarth/StringUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ObjectLocator>
#include <osgEarthAnnotation/LabelNode>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Util;

static MapNode*       s_mapNode    = 0L;
static LabelNode*     s_labelNode  = 0L;


// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler()
        : _mouseDown( false ),
          _query( s_mapNode->getMap() )
    {
        _map = s_mapNode->getMap();
        _query.setMaxTilesToCache(10);
        _path.push_back( s_mapNode->getTerrainEngine() );
    }

    void update( float x, float y, osgViewer::View* view )
    {
        osgUtil::LineSegmentIntersector::Intersections results;

        if ( view->computeIntersections( x, y, _path, results ) )
        {
            // find the first hit under the mouse:
            osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
            osg::Vec3d point = first.getWorldIntersectPoint();
            osg::Vec3d lla;
            
            // transform it to map coordinates:
            _map->worldPointToMapPoint(point, lla);

            // find the elevation at that map point:
            osg::Matrixd out_mat;
            double query_resolution = 0.1; // 1/10th of a degree
            double out_elevation = 0.0;
            double out_resolution = 0.0;

            bool ok = _query.getElevation( 
                lla,
                _map->getProfile()->getSRS(),
                out_elevation, 
                query_resolution, 
                &out_resolution );

            if ( ok )
            {
                s_labelNode->setPosition( osg::Vec3d(lla.x(), lla.y(), out_elevation) );

                s_labelNode->setText( Stringify()
                    << std::fixed << std::setprecision(2) 
                    << "Pos: " << lla.x() << ", " << lla.y() << "\n"
                    << "Elv: " << out_elevation << "m" << "\n"
                    << "X:   " << point.x() << "\n"
                    << "Y:   " << point.y() << "\n"
                    << "Z:   " << point.z() );

                s_labelNode->setNodeMask( ~0 );
            }
            else
            {
                OE_NOTICE
                    << "getElevation FAILED! at (" << point.x() << ", " << point.y() << ")" << std::endl;
            }

        }
        else
        {
            s_labelNode->setNodeMask( 0 );
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
    osg::NodePath    _path;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    s_mapNode = MapNode::load(arguments);
    if ( !s_mapNode )
    {
        OE_WARN << "Unable to load earth file." << std::endl;
        return -1;
    }

    osg::Group* root = new osg::Group();

    // The MapNode will render the Map object in the scene graph.
    root->addChild( s_mapNode );

    // style the text
    TextSymbol* sym = new TextSymbol();
    sym->halo()->color().set(0,0,0,1);

    s_labelNode = new LabelNode( s_mapNode, osg::Vec3d(0,0,0), "", sym );
    s_labelNode->setDynamic( true );

    root->addChild( s_labelNode );

    viewer.setSceneData( root );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new QueryElevationHandler() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    // install the programmable manipulator.
    if ( s_mapNode->getMap()->isGeocentric() )
        viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    return viewer.run();
}
