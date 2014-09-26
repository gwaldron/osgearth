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

#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ElevationQuery>
#include <osgEarth/StringUtils>
#include <osgEarth/Terrain>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/LatLongFormatter>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static MapNode*       s_mapNode     = 0L;
static LabelControl*  s_posLabel    = 0L;
static LabelControl*  s_vdaLabel    = 0L;
static LabelControl*  s_mslLabel    = 0L;
static LabelControl*  s_haeLabel    = 0L;
static LabelControl*  s_mapLabel    = 0L;
static LabelControl*  s_resLabel    = 0L;


// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler()
        : _mouseDown( false ),
          _terrain  ( s_mapNode->getTerrain() ),
          _query    ( s_mapNode->getMap() )
    {
        _map = s_mapNode->getMap();
        _query.setMaxTilesToCache(10);
        _path.push_back( s_mapNode->getTerrainEngine() );
    }

    void update( float x, float y, osgViewer::View* view )
    {
        bool yes = false;

        // look under the mouse:
        osg::Vec3d world;
        osgUtil::LineSegmentIntersector::Intersections hits;
        if ( view->computeIntersections(x, y, hits) )
        {
            world = hits.begin()->getWorldIntersectPoint();

            // convert to map coords:
            GeoPoint mapPoint;
            mapPoint.fromWorld( _terrain->getSRS(), world );

            // do an elevation query:
            double query_resolution = 0; // 1/10th of a degree
            double out_hamsl        = 0.0;
            double out_resolution   = 0.0;

            bool ok = _query.getElevation( 
                mapPoint,
                out_hamsl,
                query_resolution, 
                &out_resolution );

            if ( ok )
            {
                // convert to geodetic to get the HAE:
                mapPoint.z() = out_hamsl;
                GeoPoint mapPointGeodetic( s_mapNode->getMapSRS()->getGeodeticSRS(), mapPoint );

                static LatLongFormatter s_f;

                s_posLabel->setText( Stringify()
                    << std::fixed << std::setprecision(2) 
                    << s_f.format(mapPointGeodetic.y())
                    << ", " 
                    << s_f.format(mapPointGeodetic.x()) );

                s_mslLabel->setText( Stringify() << out_hamsl );
                s_haeLabel->setText( Stringify() << mapPointGeodetic.z() );
                s_resLabel->setText( Stringify() << out_resolution );

                yes = true;
            }

            // finally, get a normal ISECT HAE point.
            GeoPoint isectPoint;
            isectPoint.fromWorld( _terrain->getSRS()->getGeodeticSRS(), world );
            s_mapLabel->setText( Stringify() << isectPoint.alt() );
        }

        if (!yes)
        {
            s_posLabel->setText( "-" );
            s_mslLabel->setText( "-" );
            s_haeLabel->setText( "-" );
            s_resLabel->setText( "-" );
        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE &&
            aa.asView()->getFrameStamp()->getFrameNumber() % 10 == 0)
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            update( ea.getX(), ea.getY(), view );
        }

        return false;
    }

    const Map*       _map;
    const Terrain*   _terrain;
    bool             _mouseDown;
    ElevationQuery   _query;
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
    viewer.setSceneData( root );
    
    // install the programmable manipulator.
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // The MapNode will render the Map object in the scene graph.
    root->addChild( s_mapNode );

    // Make the readout:
    Grid* grid = new Grid();
    grid->setControl(0,0,new LabelControl("Coords (Lat, Long):"));
    grid->setControl(0,1,new LabelControl("Vertical Datum:"));
    grid->setControl(0,2,new LabelControl("Height (MSL):"));
    grid->setControl(0,3,new LabelControl("Height (HAE):"));
    grid->setControl(0,4,new LabelControl("Isect  (HAE):"));
    grid->setControl(0,5,new LabelControl("Resolution:"));

    s_posLabel = grid->setControl(1,0,new LabelControl(""));
    s_vdaLabel = grid->setControl(1,1,new LabelControl(""));
    s_mslLabel = grid->setControl(1,2,new LabelControl(""));
    s_haeLabel = grid->setControl(1,3,new LabelControl(""));
    s_mapLabel = grid->setControl(1,4,new LabelControl(""));
    s_resLabel = grid->setControl(1,5,new LabelControl(""));

    const SpatialReference* mapSRS = s_mapNode->getMapSRS();
    s_vdaLabel->setText( mapSRS->getVerticalDatum() ? 
        mapSRS->getVerticalDatum()->getName() : 
        Stringify() << "geodetic (" << mapSRS->getEllipsoid()->getName() << ")" );

    ControlCanvas* canvas = new ControlCanvas();    
    root->addChild(canvas);
    canvas->addControl( grid );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new QueryElevationHandler() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
