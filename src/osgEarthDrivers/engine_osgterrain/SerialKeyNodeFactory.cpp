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
#include "SerialKeyNodeFactory"
#include "DynamicLODScaleCallback"
#include "FileLocationCallback"
#include "LODFactorCallback"
#include <osgEarth/Registry>
#include <osg/PagedLOD>
#include <osg/CullStack>
#include <osg/Uniform>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SerialKeyNodeFactory] "

SerialKeyNodeFactory::SerialKeyNodeFactory(TileBuilder*             builder,
                                           const OSGTerrainOptions& options,
                                           const MapInfo&           mapInfo,
                                           CustomTerrain*           terrain,
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
SerialKeyNodeFactory::addTile(CustomTile* tile, bool tileHasRealData, bool tileHasLodBlending, osg::Group* parent )
{
    // associate this tile with the terrain:
    tile->setTerrainTechnique( osg::clone(_terrain->getTerrainTechniquePrototype(), osg::CopyOp::DEEP_COPY_ALL) );
    tile->setTerrain( _terrain );
    tile->setTerrainRevision( _terrain->getRevision() );
    _terrain->registerTile( tile );

    // check to see if this tile has children of its own:
    if ( tileHasRealData )
    {
        osg::BoundingSphere bs = tile->getBound();
        double maxRange = 1e10;
        double minRange = bs.radius() * _options.minTileRangeFactor().value();

        // create a PLOD so we can keep subdividing:
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( bs.center() );
        plod->addChild( tile, minRange, maxRange );

        if ( tileHasLodBlending )
        {
            // Make the LOD transition distance, and a measure of how
            // close the tile is to an LOD change, to shaders.
            plod->addCullCallback(new Drivers::LODFactorCallback);
        }

        // this cull callback dynamically adjusts the LOD scale based on distance-to-camera:
        if ( _options.lodFallOff().isSet() && *_options.lodFallOff() > 0.0 )
        {
            plod->addCullCallback( new DynamicLODScaleCallback(*_options.lodFallOff()) );
        }

        // this one rejects back-facing tiles:
        if ( _mapInfo.isGeocentric() )
        {
            plod->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
                static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer())->getHeightField(),
                tile->getLocator()->getEllipsoidModel(),
                tile->getVerticalScale() ) );
        }

        // assemble a URI for this tile's child group:
        std::stringstream buf;
        buf << tile->getKey().str() << "." << _engineUID << ".osgearth_osgterrain_tile";
        std::string uri; uri = buf.str();

        //Only add the next tile if it hasn't been blacklisted
        bool isBlacklisted = osgEarth::Registry::instance()->isBlacklisted( uri );
        if ( !isBlacklisted && tile->getKey().getLevelOfDetail() < (unsigned)*_options.maxLOD() ) //&& validData )
        {
            plod->setFileName( 1, uri );
            plod->setRange( 1, 0.0, minRange );
        }
        else
        {
            plod->setRange( 0, 0, FLT_MAX );
            OE_DEBUG << LC << "Ignored a blacklisted tile at key " << tile->getKey().str() << std::endl;
        }

#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = new osgDB::Options;
        options->setFileLocationCallback( new FileLocationCallback() );
        plod->setDatabaseOptions( options );

#endif

        parent->addChild( plod );
    }
    else
    {
        parent->addChild( tile );
    }
}

osg::Node*
SerialKeyNodeFactory::createNode( const TileKey& key )
{
    osg::ref_ptr<CustomTile> tiles[4];
    bool                     realData[4];
    bool                     lodBlending[4];

    for( unsigned i = 0; i < 4; ++i )
    {
        TileKey child = key.createChildKey( i );
        bool hasBlending = false;
        _builder->createTile( child, false, tiles[i], realData[i], lodBlending[i] );
    }

    // Now postprocess them and assemble into a tile group.
    osg::Group* root = new osg::Group();

    for( unsigned i = 0; i < 4; ++i )
    {
        if ( tiles[i].valid() )
        {
            addTile( tiles[i].get(), realData[i], lodBlending[i], root );
        }
    }

    //TODO: need to check to see if the group is empty, and do something different.
    return root;
}
