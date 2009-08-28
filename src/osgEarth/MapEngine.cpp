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

#include <osgEarth/MapEngine>
#include <osgEarth/DirectReadTileSource>
#include <osgEarth/Caching>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Compositing>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/TileSourceFactory>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/ElevationManager>

#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osgFX/MultiTextureControl>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <OpenThreads/ReentrantMutex>
#include <sstream>
#include <stdlib.h>

using namespace OpenThreads;
using namespace osgEarth;


MapEngine::MapEngine( const MapEngineProperties& props ) :
_engineProps( props )
{
    //nop
}

MapEngine::~MapEngine()
{
    //nop
}

const MapEngineProperties& 
MapEngine::getEngineProperties() const
{
    return _engineProps;
}

std::string
MapEngine::createURI( unsigned int id, const TileKey* key )
{
    std::stringstream ss;
    ss << key->str() << "." <<id<<".earth_tile";
    return ss.str();
}

// Make a MatrixTransform suitable for use with a Locator object based on the given extents.
// Calling Locator::setTransformAsExtents doesn't work with OSG 2.6 due to the fact that the
// _inverse member isn't updated properly.  Calling Locator::setTransform works correctly.
osg::Matrixd
MapEngine::getTransformFromExtents(double minX, double minY, double maxX, double maxY) const
{
    osg::Matrixd transform;
    transform.set(
        maxX-minX, 0.0,       0.0, 0.0,
        0.0,       maxY-minY, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        minX,      minY,      0.0, 1.0); 
    return transform;
}

osg::Node*
MapEngine::createNode( Map* map, osgTerrain::Terrain* terrain, const TileKey* key, bool populateLayers )
{
    osg::ref_ptr<osg::Group> parent = new osg::Group;
    if ( !addChildren( map, terrain, parent.get(), key, populateLayers ))
    {
        parent = 0;
    }
    return parent.release();
}

GeoImage*
MapEngine::createValidGeoImage(TileSource* tileSource, const TileKey* key)
{
    //Try to create the image with the given key
    osg::ref_ptr<const TileKey> image_key = key;

    osg::ref_ptr<GeoImage> geo_image;

    while (image_key.valid())
    {
        if ( tileSource->isKeyValid(image_key.get()) )
        {
            geo_image = createGeoImage( image_key.get(), tileSource );
            if (geo_image.valid()) return geo_image.release();
        }
        image_key = image_key->createParentKey();
    }
    return 0;
}

bool
MapEngine::hasMoreLevels( Map* map, const TileKey* key )
{
    OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    bool more_levels = false;
    int max_level = 0;

    for ( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
//        if ( i->get()->maxLevel().isSet() && key->getLevelOfDetail() < i->get()->maxLevel().get() )
        if ( !i->get()->maxLevel().isSet() || key->getLevelOfDetail() < i->get()->maxLevel().get() )
        {
            more_levels = true;
            break;
        }
    }
    if ( !more_levels )
    {
        for( MapLayerList::const_iterator j = map->getHeightFieldMapLayers().begin(); j != map->getHeightFieldMapLayers().end(); j++ )
        {
//            if ( j->get()->maxLevel().isSet() && key->getLevelOfDetail() < j->get()->maxLevel().get() )
            if ( !j->get()->maxLevel().isSet() || key->getLevelOfDetail() < j->get()->maxLevel().get() )
            {
                more_levels = true;
                break;
            }
        }
    }

    return more_levels;
}

bool
MapEngine::addChildren(Map* map,
                       osgTerrain::Terrain* terrain,
                       osg::Group* tile_parent,
                       const TileKey* key,
                       bool populateLayers )
{
    bool all_quadrants_created = false;

    osg::ref_ptr<osg::Node> q0, q1, q2, q3;

    osg::ref_ptr<TileKey> k0 = key->getSubkey(0);
    osg::ref_ptr<TileKey> k1 = key->getSubkey(1);
    osg::ref_ptr<TileKey> k2 = key->getSubkey(2);
    osg::ref_ptr<TileKey> k3 = key->getSubkey(3);

    q0 = createQuadrant( map, terrain, k0.get(), populateLayers );
    q1 = createQuadrant( map, terrain, k1.get(), populateLayers );
    q2 = createQuadrant( map, terrain, k2.get(), populateLayers );
    q3 = createQuadrant( map, terrain, k3.get(), populateLayers );

    all_quadrants_created = (q0.valid() && q1.valid() && q2.valid() && q3.valid());

    if (all_quadrants_created)
    {
        if (q0.valid()) tile_parent->addChild(q0.get());
        if (q1.valid()) tile_parent->addChild(q1.get());
        if (q2.valid()) tile_parent->addChild(q2.get());
        if (q3.valid()) tile_parent->addChild(q3.get());
    }
    else
    {
        osg::notify(osg::INFO) << "[osgEarth::MapEngine] Couldn't create all quadrants for " << key->str() << " time to stop subdividing!" << std::endl;
    }
    return all_quadrants_created;
}


GeoImage*
MapEngine::createGeoImage(const TileKey* mapKey, TileSource* source)
{
    GeoImage* result = NULL;
    const Profile* mapProfile = mapKey->getProfile();

    //If the key profile and the source profile exactly match, simply request the image from the source
    if ( mapProfile->isEquivalentTo( source->getProfile() ) )
    {
        osg::Image* image = source->createImage( mapKey );
        if ( image )
        {
            result = new GeoImage( image, mapKey->getGeoExtent() );
        }
    }

    // Otherwise, we need to process the tiles.
    else
    {
        Compositor comp;
        osg::ref_ptr<GeoImage> mosaic = comp.mosaicImages( mapKey, source );

        if ( mosaic.valid() )
        {
            // whether to use the fast-path mercator locator. If so, DO NOT reproject the imagery here.
            bool useMercatorFastPath =
                mosaic->getSRS()->isMercator() &&
                mapKey->getProfile()->getSRS()->isGeographic() &&
                _engineProps.getUseMercatorLocator();

            if ( !mosaic->getSRS()->isEquivalentTo( mapKey->getProfile()->getSRS()) && !useMercatorFastPath )
            {
                //We actually need to reproject the image.  Note:  The GeoImage::reprojection function will automatically
                //crop the image to the correct extents, so there is no need to crop after reprojection.
                result = mosaic->reproject( mapKey->getProfile()->getSRS(), &mapKey->getGeoExtent() );
            }
            else
            {
                // crop to fit the map key extents
                GeoExtent clampedMapExt = source->getProfile()->clampAndTransformExtent( mapKey->getGeoExtent() );
                if ( clampedMapExt.width() * clampedMapExt.height() > 0 )
                    result = mosaic->crop(clampedMapExt);
                else
                    result = NULL;
            }
        }
    }

    return result;
}

bool
MapEngine::isCached(Map* map, const osgEarth::TileKey *key)
{
    OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    const Profile* mapProfile = key->getProfile();

    //Check the imagery layers
    for( MapLayerList::const_iterator i = map->getImageMapLayers().begin(); i != map->getImageMapLayers().end(); i++ )
    {
        MapLayer* layer = i->get();
        std::vector< osg::ref_ptr< const TileKey > > keys;

        if ( map->getProfile()->isEquivalentTo( layer->getTileSource()->getProfile() ) )
        {
            keys.push_back( key );
        }
        else
        {
            layer->getTileSource()->getProfile()->getIntersectingTiles( key, keys );
        }

        for (unsigned int j = 0; j < keys.size(); ++j)
        {
            if ( layer->getTileSource()->isKeyValid( keys[j].get() ) )
            {
                if ( !layer->getTileSource()->isCached( keys[j].get() ) )
                {
                    return false;
                }
            }
        }
    }

    //Check the elevation layers
    for( MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
        MapLayer* layer = i->get();
        std::vector< osg::ref_ptr< const TileKey > > keys;

        if ( map->getProfile()->isEquivalentTo( layer->getTileSource()->getProfile() ) )
        {
            keys.push_back( key );
        }
        else
        {
            layer->getTileSource()->getProfile()->getIntersectingTiles( key, keys );
        }

        for (unsigned int j = 0; j < keys.size(); ++j)
        {
            if ( layer->getTileSource()->isKeyValid( keys[j].get() ) )
            {
                if ( !layer->getTileSource()->isCached( keys[j].get() ) )
                {
                    return false;
                }
            }
        }
    }

    return true;
}



osg::HeightField*
MapEngine::createHeightField( Map* map, const TileKey* key, bool fallback )
{   
    // dont' need this here??
    //OpenThreads::ScopedReadLock lock( map->getMapDataMutex() );

    osg::ref_ptr< ElevationManager > em = new ElevationManager;

    for( MapLayerList::const_iterator i = map->getHeightFieldMapLayers().begin(); i != map->getHeightFieldMapLayers().end(); i++ )
    {
        em->getElevationSources().push_back( i->get()->getTileSource() );
    }
    return em->createHeightField( key, 0, 0, fallback );
}

osg::HeightField*
MapEngine::createEmptyHeightField( const TileKey* key )
{
    //Get the bounds of the key
    double minx, miny, maxx, maxy;
    key->getGeoExtent().getBounds(minx, miny, maxx, maxy);

    osg::HeightField *hf = new osg::HeightField();
    hf->allocate( 16, 16 );
    for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
        hf->getHeightList()[i] = 0.0;

    hf->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
    hf->setXInterval( (maxx - minx)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (maxy - miny)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );
    return hf;
}

