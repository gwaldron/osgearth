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
#include <osg/Image>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GeometryRasterizer>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;

/**
 * This sample demonstrates how to create a custom TileSource.
 */

static osg::Vec4 colors[4] = {
    osg::Vec4(1,0,0,1),
    osg::Vec4(0,1,0,1),
    osg::Vec4(0,0,1,1),
    osg::Vec4(0,0,0,1)
};

/**
 * Our homemade TileSource.
 */
class CustomTileSource : public TileSource
{
public:
    // Constructor that takes the user-provided options.
    CustomTileSource( const TileSourceOptions& options =TileSourceOptions() ) : TileSource(options)
    {
        _geom = new Ring();
        _geom->push_back( osg::Vec3(5, 5, 0) );
        _geom->push_back( osg::Vec3(250, 5, 0) );
        _geom->push_back( osg::Vec3(250, 250, 0) );
        _geom->push_back( osg::Vec3(5, 250, 0) );
    }

    // Called by the terrain engine when a layer using this driver is first added.
    Status initialize(const osgDB::Options* dbOptions)
    {
        if ( !getProfile() )
        {
            setProfile( Registry::instance()->getGlobalGeodeticProfile() );
        }
        return STATUS_OK;
    }

    // Define this method to return an image corresponding to the given TileKey.
    osg::Image* createImage( const TileKey& key, ProgressCallback* progress )
    {
        GeometryRasterizer rasterizer( 256, 256 );
        rasterizer.draw( _geom.get(), colors[key.getLevelOfDetail() % 4] );
        return rasterizer.finalize();
    }

    osg::ref_ptr<Ring> _geom;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // Start by creating the map:
    MapOptions mapOptions;
    mapOptions.cachePolicy() = CachePolicy::NO_CACHE;
    Map* map = new Map( mapOptions );

    // Create out image layer with a custom tile source.
    ImageLayerOptions options( "custom" );
    map->addImageLayer( new ImageLayer(options, new CustomTileSource()) );

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNode* mapNode = new MapNode( map );

    viewer.setSceneData( mapNode );
    viewer.setCameraManipulator( new EarthManipulator() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
