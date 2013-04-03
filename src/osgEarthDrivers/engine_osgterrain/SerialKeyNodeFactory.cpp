/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include "SerialKeyNodeFactory"
#include "DynamicLODScaleCallback"
#include "FileLocationCallback"
#include "LODFactorCallback"
#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osg/PagedLOD>
#include <osg/CullStack>
#include <osg/Uniform>

#include <osgEarth/MapNode>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SerialKeyNodeFactory] "


SerialKeyNodeFactory::SerialKeyNodeFactory(TileBuilder*             builder,
                                           const OSGTerrainOptions& options,
                                           const MapInfo&           mapInfo,
                                           TerrainNode*             terrain,
                                           UID                      engineUID ) :
_builder( builder ),
_options( options ),
_mapInfo( mapInfo ),
_terrain( terrain ),
_engineUID( engineUID )
{
    //NOP
}

void
SerialKeyNodeFactory::addTile(Tile* tile, bool tileHasRealData, bool tileHasLodBlending, osg::Group* parent )
{
    // associate this tile with the terrain:
    tile->setTerrainTechnique( _terrain->cloneTechnique() );
    tile->attachToTerrain( _terrain );

    // assemble a URI for this tile's child group:
    std::stringstream buf;
    buf << tile->getKey().str() << "." << _engineUID << ".osgearth_osgterrain_tile";
    std::string uri; uri = buf.str();

    osg::Node* result = 0L;

    // Only add the next tile if all the following are true:
    // 1. Either there's real tile data, or a maxLOD is explicity set in the options;
    // 2. The tile isn't blacklisted; and
    // 3. We are still below the max LOD.
    bool wrapInPagedLOD =
        (tileHasRealData || (_options.minLOD().isSet() && tile->getKey().getLOD() < *_options.minLOD())) &&
        !osgEarth::Registry::instance()->isBlacklisted( uri ) &&
        tile->getKey().getLOD() < *_options.maxLOD();
        //(!_options.minLOD().isSet() || tile->getKey().getLevelOfDetail() < *_options.maxLOD());

    if ( wrapInPagedLOD )
    {
        osg::BoundingSphere bs = tile->getBound();
        float maxRange = FLT_MAX;
        
#if 0
        //Compute the min range based on the actual bounds of the tile.  This can break down if you have very high resolution
        //data with elevation variations and you can run out of memory b/c the elevation change is greater than the actual size of the tile so you end up
        //inifinitely subdividing (or at least until you run out of data or memory)
        float minRange = (float)(bs.radius() * _options.minTileRangeFactor().value());
#else        
        //double origMinRange = bs.radius() * _options.minTileRangeFactor().value();        
        //Compute the min range based on the 2D size of the tile
        GeoExtent extent = tile->getKey().getExtent();        
        GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
        GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
        osg::Vec3d ll, ur;
        lowerLeft.toWorld( ll );
        upperRight.toWorld( ur );
        double radius = (ur - ll).length() / 2.0;
        float minRange = (float)(radius * _options.minTileRangeFactor().value());
#endif

        // create a PLOD so we can keep subdividing:
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( bs.center() );
        plod->addChild( tile, minRange, maxRange );

        plod->setFileName( 1, uri );
        plod->setRange   ( 1, 0, minRange );

        plod->setUserData( new MapNode::TileRangeData(minRange, maxRange) );

#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setFileLocationCallback( new FileLocationCallback() );
        plod->setDatabaseOptions( options );

#endif
        result = plod;
        
        if ( tileHasLodBlending )
        {
            // Make the LOD transition distance, and a measure of how
            // close the tile is to an LOD change, to shaders.
            result->addCullCallback(new LODFactorCallback);
        }
    }
    else
    {
        result = tile;
    }

    // this cull callback dynamically adjusts the LOD scale based on distance-to-camera:
    if ( _options.lodFallOff().isSet() && *_options.lodFallOff() > 0.0 )
    {
        result->addCullCallback( new DynamicLODScaleCallback(*_options.lodFallOff()) );
    }

    // this one rejects back-facing tiles:
    if ( _mapInfo.isGeocentric() && _options.clusterCulling() == true )
    {
        result->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
            static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer())->getHeightField(),
            tile->getLocator()->getEllipsoidModel(),
            tile->getVerticalScale() ) );
    }

    parent->addChild( result );
}

osg::Node*
SerialKeyNodeFactory::createRootNode( const TileKey& key )
{
    osg::ref_ptr<Tile> tile;
    bool               real;
    bool               lodBlending;

    _builder->createTile(key, false, tile, real, lodBlending);

    osg::Group* root = new osg::Group();
    addTile( tile, real, lodBlending, root );
    
    return root;
}

osg::Node*
SerialKeyNodeFactory::createNode( const TileKey& parentKey )
{
    osg::ref_ptr<Tile> tiles[4];
    bool               realData[4];
    bool               lodBlending[4];
    bool               tileHasAnyRealData = false;

    for( unsigned i = 0; i < 4; ++i )
    {
        TileKey child = parentKey.createChildKey( i );
        _builder->createTile( child, false, tiles[i], realData[i], lodBlending[i] );
        if ( tiles[i].valid() && realData[i] )
            tileHasAnyRealData = true;
    }

    osg::Group* root = 0L;

    // assemble the tile.
    if ( tileHasAnyRealData || _options.minLOD().isSet() || parentKey.getLevelOfDetail() == 0 )
    {
        // Now postprocess them and assemble into a tile group.
        root = new osg::Group();

        for( unsigned i = 0; i < 4; ++i )
        {
            if ( tiles[i].valid() )
            {
                addTile( tiles[i].get(), realData[i], lodBlending[i], root );
            }
        }
    }

    return root;
}
