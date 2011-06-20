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
#include <sstream>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Util;

static osgText::Text* s_flagText;

static
osg::Node* createFlag()
{
    osg::Cylinder* c = new osg::Cylinder( osg::Vec3d(0,0,0), 2.0f, 250.f );
    osg::Geode* g = new osg::Geode();
    g->addDrawable( new osg::ShapeDrawable( c ) );
    osgText::Text* text = new osgText::Text();
    text->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );
    text->setCharacterSize( 24.f );
    text->setFont( osgText::readFontFile("arial.ttf") );
    text->setBackdropType( osgText::Text::OUTLINE );
    text->setText( "00000000000000" );
    text->setAutoRotateToScreen( true );
    text->setPosition( osg::Vec3d( 0, 0, 125 ) );
    text->setDataVariance( osg::Object::DYNAMIC );
    g->addDrawable( text );
    osg::AutoTransform* at = new osg::AutoTransform();
    at->setAutoScaleToScreen( true );
    at->addChild( g );
    at->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
    at->setDataVariance( osg::Object::DYNAMIC );
    //osg::MatrixTransform* xf = new osg::MatrixTransform();
    //xf->addChild( at );
    //xf->setDataVariance( osg::Object::DYNAMIC );
    s_flagText = text;
    return at;
}

// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler(const Map* map, ObjectLocator* flagLocator ) 
        : _mouseDown( false ), 
          _flagLocator(flagLocator), 
          _query(map),
          _mapSRS(map->getProfile()->getSRS())
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
            double lat_rad, lon_rad, height;
            _mapSRS->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, height );
            
            // query the elevation at the map point:
            double lat_deg = osg::RadiansToDegrees( lat_rad );
            double lon_deg = osg::RadiansToDegrees( lon_rad );
            osg::Matrixd out_mat;
            double query_resolution = 0.1; // 1/10th of a degree
            double out_elevation = 0.0;
            double out_resolution = 0.0;

            bool ok = _query.getElevation( 
                osg::Vec3d(lon_deg, lat_deg, 0), 
                _mapSRS.get(), 
                out_elevation, 
                query_resolution, 
                &out_resolution );

            if ( ok )
            {
                _flagLocator->setPosition( osg::Vec3d(lon_deg, lat_deg, out_elevation) );

                std::stringstream buf;
                buf << std::fixed << std::setprecision(2) 
                    << "Pos: " << lat_deg << ", " << lon_deg << std::endl
                    << "Elv: " << out_elevation << "m";
                s_flagText->setText( buf.str() );
            }
            else
            {
                OE_NOTICE
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

    bool                                 _mouseDown;
    ElevationQuery                       _query;
    ObjectLocator*                       _flagLocator;
    osg::ref_ptr<const SpatialReference> _mapSRS;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    osgEarth::Util::EarthManipulator* manip = new osgEarth::Util::EarthManipulator();
    viewer.setCameraManipulator( manip );

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

    // the SRS of the map
    const SpatialReference* mapSRS = mapNode->getMap()->getProfile()->getSRS();

    // A flag so we can see where we clicked
    ObjectLocatorNode* flag = new ObjectLocatorNode( mapSRS );
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

    return viewer.run();
}
