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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ElevationManager>
#include <osgEarthDrivers/tms/TMSOptions>
#include <sstream>

using namespace osgEarth::Drivers;

static
osg::MatrixTransform* createFlag()
{
    osg::Cylinder* c = new osg::Cylinder( osg::Vec3d(0,0,0), 2.0f, 250.f );
    osg::Geode* g = new osg::Geode();
    g->addDrawable( new osg::ShapeDrawable( c ) );
    osgText::Text* text = new osgText::Text();
    text->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );
    text->setCharacterSize( 72.f );
    text->setBackdropType( osgText::Text::OUTLINE );
    text->setText( "00000000000000" );
    text->setAutoRotateToScreen( true );
    text->setPosition( osg::Vec3d( 0, 0, 125 ) );
    g->addDrawable( text );
    osg::AutoTransform* at = new osg::AutoTransform();
    at->setAutoScaleToScreen( true );
    at->addChild( g );
    at->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
    osg::MatrixTransform* xf = new osg::MatrixTransform();
    xf->addChild( at );
    return xf;
}

static void
updateFlag( osg::MatrixTransform* xf, const osg::Matrix& mat, double elev )
{
    osg::Geode* g = static_cast<osg::Geode*>( xf->getChild(0)->asGroup()->getChild(0) );
    std::stringstream buf;
    buf << elev;
	std::string bufStr;
	bufStr = buf.str();
    static_cast<osgText::Text*>( g->getDrawable(1) )->setText( bufStr );
    xf->setMatrix( mat );
}

// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler( osgEarthUtil::ElevationManager* elevMan, osg::MatrixTransform* flag )
        :_elevMan(elevMan), _flag(flag), _mouseDown(false) { }

    void update( float x, float y, osgViewer::View* view )
    {
        osgUtil::LineSegmentIntersector::Intersections results;
        if ( view->computeIntersections( x, y, results, 0x01 ) )
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
            osg::Matrixd out_mat;
            double out_elevation = 0.0;
            double out_resolution = 0.0;

            if ( _elevMan->getPlacementMatrix( lon_deg, lat_deg, 0, 0, NULL, out_mat, out_elevation, out_resolution ) )
            {
                updateFlag( _flag.get(), out_mat, out_elevation );
            }
            else
            {
                osg::notify(osg::NOTICE)
                    << "getElevation FAILED! at (" << lat_deg << ", " << lon_deg << ")" << std::endl;
            }

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

    bool _mouseDown;
    osg::ref_ptr<osgEarthUtil::ElevationManager> _elevMan;
    osg::ref_ptr<osg::MatrixTransform> _flag;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    osgEarthUtil::EarthManipulator* manip = new osgEarthUtil::EarthManipulator();
    viewer.setCameraManipulator( manip );

    // load up a map with an elevation layer:
    osgEarth::Map* map = new osgEarth::Map();

    // Add some imagery
    {
        TMSOptions* tms = new TMSOptions();
        tms->url() = "http://demo.pelicanmapping.com/rmweb/data/bluemarble-tms/tms.xml";
        map->addMapLayer( new osgEarth::ImageMapLayer( "BLUEMARBLE", tms ) );
    }

    // Add some elevation
    {
        TMSOptions* tms = new TMSOptions();
        tms->url() = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
        map->addMapLayer( new osgEarth::HeightFieldMapLayer( "SRTM", tms ) );
    }

    osg::Group* root = new osg::Group();

    // The MapNode will render the Map object in the scene graph.
    osgEarth::MapNode* mapNode = new osgEarth::MapNode( map );
    mapNode->setNodeMask( 0x01 );
    root->addChild( mapNode );

    // A flag so we can see where we clicked
    osg::MatrixTransform* flag = createFlag();
    flag->setNodeMask( 0x02 );
    root->addChild( flag );

    viewer.setSceneData( root );

    // AN elevation manager that is tied to the map node:
    osgEarthUtil::ElevationManager* elevMan = new osgEarthUtil::ElevationManager( mapNode->getMap() );
    elevMan->setMaxTilesToCache( 10 );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new QueryElevationHandler( elevMan, flag ) );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
