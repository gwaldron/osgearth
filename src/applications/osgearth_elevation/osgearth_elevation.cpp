/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ElevationManager>


// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler( osgEarthUtil::ElevationManager* elevMan ) : _elevMan(elevMan) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        // ctrl-click:
        if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH && 0 != (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) )
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
            osgUtil::LineSegmentIntersector::Intersections results;
            if ( view->computeIntersections( ea.getX(), ea.getY(), results ) )
            {
                // find the first hit under the mouse:
                osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
                osg::Vec3d point = first.getWorldIntersectPoint();
                
                // transform it to map coordinates:
                const SpatialReference* srs = _elevMan->getMap()->getProfile()->getSRS();
                double lat_rad, lon_rad, height;
                srs->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, height );
                
                // query the elevation at the map point:
                double lat_deg = osg::RadiansToDegrees( lat_rad );
                double lon_deg = osg::RadiansToDegrees( lon_rad );
                double out_elevation = 0.0;
                double out_resolution = 0.0;

                if ( _elevMan->getElevation( lon_deg, lat_deg, 0, NULL, out_elevation, out_resolution ) )
                {
                    osg::notify(osg::NOTICE) 
                        << "Elevation at (" << lat_deg << ", " << lon_deg << ") = " << out_elevation
                        << ", Resolution = " << out_resolution << " " 
                        << std::endl;
                }
                else
                {
                    osg::notify(osg::NOTICE)
                        << "getElevation FAILED! at (" << lat_deg << ", " << lon_deg << ")" << std::endl;
                }

            }
            return true;
        }
        else
            return false;
    }

    osg::ref_ptr<osgEarthUtil::ElevationManager> _elevMan;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    osg::notify(osg::NOTICE)
        << "*******************************************" << std::endl
        << "CTRL-Click on the Globe to query elevation." << std::endl
        << std::endl;

    // install the programmable manipulator.
    osgEarthUtil::EarthManipulator* manip = new osgEarthUtil::EarthManipulator();
    viewer.setCameraManipulator( manip );

    // load up a map with an elevation layer:
    osgEarth::Map* map = new osgEarth::Map();

    // Add some imagery
    {
        osgEarth::Properties conf;
        conf["url"] = "http://demo.pelicanmapping.com/rmweb/data/bluemarble-tms/tms.xml";
        map->addMapLayer( new osgEarth::MapLayer(
            "BLUEMARBLE", osgEarth::MapLayer::TYPE_IMAGE, "tms", conf ) );
    }

    // Add some elevation
    {
        osgEarth::Properties conf;
        conf["url"] = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
        map->addMapLayer( new osgEarth::MapLayer(
            "SRTM", osgEarth::MapLayer::TYPE_HEIGHTFIELD, "tms", conf ) );
    }

    // The MapNode will render the Map object in the scene graph.
    osgEarth::MapNode* mapNode = new osgEarth::MapNode( map );
    viewer.setSceneData( mapNode );

    // AN elevation manager that is tied to the map node:
    osgEarthUtil::ElevationManager* elevMan = new osgEarthUtil::ElevationManager( mapNode );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new QueryElevationHandler( elevMan ) );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
