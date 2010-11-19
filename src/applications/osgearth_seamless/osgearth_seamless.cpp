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
#include <osg/PagedLOD>
#include <osg/PolygonMode>
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
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/engine_seamless/PatchInfo>
#include <osgEarthDrivers/engine_seamless/SeamlessOptions>
#include <sstream>

// Print info about terrain rendered using the seamless engine. Copied
// from osgearth_elevation.

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
    text->setDataVariance( osg::Object::DYNAMIC );
    g->addDrawable( text );
    osg::StateSet* ss = g->getOrCreateStateSet();
    ss->setAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,
                                          osg::PolygonMode::FILL),
                     osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
    osg::AutoTransform* at = new osg::AutoTransform();
    at->setAutoScaleToScreen( true );
    at->addChild( g );
    at->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
    osg::MatrixTransform* xf = new osg::MatrixTransform();
    xf->addChild( at );
    xf->setDataVariance( osg::Object::DYNAMIC );
    return xf;
}

static void
updateFlag(osg::MatrixTransform* xf, const osg::Matrix& mat,
           const osgEarth::TileKey& key)
{
    osg::Geode* g = static_cast<osg::Geode*>( xf->getChild(0)->asGroup()->getChild(0) );
    std::stringstream buf;
    buf << key.getLevelOfDetail() << " " << key.getTileX() << " "
        << key.getTileY();
    std::string bufStr;
    bufStr = buf.str();
    static_cast<osgText::Text*>( g->getDrawable(1) )->setText( bufStr );
    xf->setMatrix( mat );
}

// An event handler that will print out the elevation at the clicked point
struct QueryTileHandler : public osgGA::GUIEventHandler 
{
    QueryTileHandler(osg::MatrixTransform* flag,
                     const osgEarth::SpatialReference* srs)
        : _flag(flag), _mouseDown(false), _srs(srs) { }

    void update( float x, float y, osgViewer::View* view )
    {
        using namespace osg;
        using namespace osgUtil;
        using namespace osgEarth;
        LineSegmentIntersector::Intersections results;
        TileKey key(TileKey::INVALID);
        if ( view->computeIntersections( x, y, results, 0x01 ) )
        {
            // find the first hit under the mouse:
            LineSegmentIntersector::Intersection first
                = *(results.begin());
            for (NodePath::const_reverse_iterator itr = first.nodePath.rbegin(),
                     end = first.nodePath.rend();
                 itr != end;
                 ++itr)
            {
                const PagedLOD* plod = dynamic_cast<const PagedLOD*>(*itr);
                if (!plod)
                    continue;
                const seamless::PatchInfo* popt
                    = dynamic_cast<const seamless::PatchInfo*>(
                        plod->getDatabaseOptions());
                if (!popt)
                    continue;
                key = popt->getTileKey();
                break;
            }

            if (!key.valid())
            {
                OE_WARN << "Couldn't get tile key.\n";
                return;
            }
            osg::Vec3d point = first.getWorldIntersectPoint();
            
            // transform it to map coordinates:
            Matrixd coordFrame;
            _srs->getEllipsoid()->computeLocalToWorldTransformFromXYZ(
                point.x(), point.y(), point.z(), coordFrame);
            updateFlag(_flag.get(), coordFrame, key);
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
    osg::ref_ptr<osg::MatrixTransform> _flag;
    osg::ref_ptr<const osgEarth::SpatialReference> _srs;
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
    if (!loadedNode)
    {
        // load up a map with an elevation layer:
        osgEarth::Map *map = new osgEarth::Map();

        // Add some imagery
        {
            TMSOptions tms;
            tms.url() = "http://demo.pelicanmapping.com/rmweb/data/bluemarble-tms/tms.xml";
            map->addImageLayer( new osgEarth::ImageLayer( "BLUEMARBLE", tms ) );
        }

        // Add some elevation
        {
            TMSOptions tms;
            tms.url() = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
            map->addElevationLayer( new osgEarth::ElevationLayer( "SRTM", tms ) );
        }

        MapNodeOptions nodeOptions;
        nodeOptions.setTerrainOptions( osgEarth::Drivers::SeamlessOptions() );

        mapNode = new osgEarth::MapNode( map, nodeOptions );
    }
    else
    {
        mapNode = findTopMostNodeOfType<osgEarth::MapNode>( loadedNode );
    }

    osg::Group* root = new osg::Group();

    // The MapNode will render the Map object in the scene graph.
    mapNode->setNodeMask( 0x01 );
    root->addChild( mapNode );

    manip->setNode(mapNode->getTerrainEngine());
    if ( mapNode->getMap()->isGeocentric() )
    {
        manip->setHomeViewpoint( 
            osgEarth::Util::Viewpoint( osg::Vec3d( -90, 0, 0 ), 0.0, -90.0, 5e7 ) );

        // add a handler that will automatically calculate good clipping planes
        viewer.addEventHandler( new osgEarth::Util::AutoClipPlaneHandler() );
    }
    // A flag so we can see where we clicked
    osg::MatrixTransform* flag = createFlag();
    flag->setNodeMask( 0x02 );
    root->addChild( flag );

    viewer.setSceneData( root );

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler(
        new QueryTileHandler(flag, mapNode->getMap()->getProfile()->getSRS()));

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
