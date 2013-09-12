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
#include "Tile"
#include "TerrainNode"
#include "CustomTerrainTechnique"
#include "TransparentLayer"

#include <osgEarth/Registry>
#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/NodeUtils>

#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Texture2D>
#include <osgGA/EventVisitor>

#include <OpenThreads/ScopedLock>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[Tile] "


//----------------------------------------------------------------------------

Tile::Tile( const TileKey& key, GeoLocator* keyLocator, bool quickReleaseGLObjects ) :
_key( key ),
_locator( keyLocator ),
_quickReleaseGLObjects( quickReleaseGLObjects ),
_hasBeenTraversed( false ),
_verticalScale( 1.0f ),
_parentTileSet( false ),
_tileId( key.getTileId() ),
_dirty( true )
{
    this->setThreadSafeRefUnref( true );
    this->setName( key.str() );

    // initially bump the update requirement so that this tile will receive an update
    // traversal the first time through. It is on the first update traversal that we
    // know the tile is in the scene graph and that it can be registered with the terrain.
    ADJUST_UPDATE_TRAV_COUNT( this, 1 );

}

Tile::~Tile()
{
    //nop
}

void
Tile::init()
{
    if ( _tech.valid() )
    {
        _tech->init();
        _dirty = false;
    }
}

void
Tile::setTerrainTechnique( TerrainTechnique* tech )
{
    if (tech)
    {
        tech->_tile = this;
        _tech = tech;
        _dirty = true;
    }
}

void
Tile::attachToTerrain( TerrainNode* terrain )
{
    _terrain = terrain;
    if ( terrain )
        terrain->registerTile( this );
}

void
Tile::setVerticalScale (float verticalScale )
{
    if (_verticalScale != verticalScale)
    {
        _verticalScale = verticalScale;
        dirtyBound();
    }
}

void
Tile::setCustomColorLayer( const CustomColorLayer& layer, bool writeLock )
{
    if ( writeLock )
    {
        Threading::ScopedWriteLock exclusiveTileLock( _tileLayersMutex );
        setCustomColorLayer( layer, false );
    }
    else
    {
        int delta = 0;
        ColorLayersByUID::const_iterator i = _colorLayers.find( layer.getUID() );
        if ( i != _colorLayers.end() && i->second.getMapLayer()->isDynamic() )
            --delta;
        
       _colorLayers[layer.getUID()] = layer;
       
        if ( layer.getMapLayer()->isDynamic() )
            ++delta;

        if ( delta != 0 )
            ADJUST_UPDATE_TRAV_COUNT( this, delta );
    }
}

void
Tile::removeCustomColorLayer( UID layerUID, bool writeLock )
{
    if ( writeLock )
    {
        Threading::ScopedWriteLock exclusiveTileLock( _tileLayersMutex );
        removeCustomColorLayer( layerUID, false );
    }
    else
    {
        ColorLayersByUID::iterator i = _colorLayers.find(layerUID);
        if ( i != _colorLayers.end() )
        {
            if ( i->second.getMapLayer()->isDynamic() )
                ADJUST_UPDATE_TRAV_COUNT( this, -1 );

            _colorLayers.erase( i );
        }
    }
}

bool
Tile::getCustomColorLayer( UID layerUID, CustomColorLayer& out, bool readLock ) const
{
    if ( readLock )
    {
        Threading::ScopedReadLock sharedTileLock( const_cast<Tile*>(this)->_tileLayersMutex );
        return getCustomColorLayer( layerUID, out, false );
    }
    else
    {
        ColorLayersByUID::const_iterator i = _colorLayers.find( layerUID );
        if ( i != _colorLayers.end() )
        {
            out = i->second;
            return true;
        }
    }
    return false;
}

void
Tile::getCustomColorLayers( ColorLayersByUID& out, bool readLock ) const
{
    if ( readLock )
    {
        Threading::ScopedReadLock sharedTileLock( const_cast<Tile*>(this)->_tileLayersMutex );
        return getCustomColorLayers( out, false );
    }
    else
        out = _colorLayers;
}

void
Tile::setCustomColorLayers( const ColorLayersByUID& in, bool writeLock )
{
    if ( writeLock )
    {
        Threading::ScopedWriteLock exclusiveLock( _tileLayersMutex );
        setCustomColorLayers( in, false );
    }
    else
    {
        int delta = 0;
        for( ColorLayersByUID::const_iterator i = _colorLayers.begin(); i != _colorLayers.end(); ++i )
            if ( i->second.getMapLayer()->isDynamic() )
                --delta;

        _colorLayers = in;

        for( ColorLayersByUID::const_iterator i = _colorLayers.begin(); i != _colorLayers.end(); ++i )
            if ( i->second.getMapLayer()->isDynamic() )
                ++delta;

        if ( delta != 0 )
            ADJUST_UPDATE_TRAV_COUNT( this, delta );
    }
}

void Tile::clear()
{    
    //Clear out any imagery & elevation data being held by this Tile
    Threading::ScopedWriteLock exclusiveLock( _tileLayersMutex );
    _colorLayers.clear();
    _elevationLayer = 0;

}

osg::BoundingSphere
Tile::computeBound() const
{
    //Overriden computeBound that takes into account the vertical scale.
    //OE_NOTICE << "Tile::computeBound verticalScale = " << _verticalScale << std::endl;

    osg::BoundingSphere bs;

    if (_elevationLayer.valid())
    {        
        if (!_elevationLayer->getLocator()) return bs;

        osg::BoundingBox bb;
        unsigned int numColumns = _elevationLayer->getNumColumns();
        unsigned int numRows = _elevationLayer->getNumRows();
        for(unsigned int r=0;r<numRows;++r)
        {
            for(unsigned int c=0;c<numColumns;++c)
            {
                float value = 0.0f;
                bool validValue = _elevationLayer->getValidValue(c,r, value);
                if (validValue) 
                {
                    //Multiply by the vertical scale.
                    value *= _verticalScale;
                    osg::Vec3d ndc, v;
                    ndc.x() = ((double)c)/(double)(numColumns-1), 
                        ndc.y() = ((double)r)/(double)(numRows-1);
                    ndc.z() = value;

                    if (_elevationLayer->getLocator()->convertLocalToModel(ndc, v))
                    {
                        bb.expandBy(v);
                    }
                }
            }
        }
        bs.expandBy(bb);

    }
    else
    {
        for(ColorLayersByUID::const_iterator i = _colorLayers.begin(); i != _colorLayers.end(); ++i )
        {
            bs.expandBy( i->second.computeBound() ); 
        }
    }

    return bs;    
}

void
Tile::queueTileUpdate( TileUpdate::Action action, int value )
{
    _dirty = true;
}

void
Tile::applyImmediateTileUpdate( TileUpdate::Action action, int value )
{
    CustomTerrainTechnique* tech = dynamic_cast<CustomTerrainTechnique*>( _tech.get() );
    if ( tech )
    {
        tech->compile( TileUpdate(action, value), 0L );
        tech->applyTileUpdates();
    }
    else
    {
        queueTileUpdate( action, value );
    }
}

void
Tile::traverse( osg::NodeVisitor& nv )
{    
    // set the parent tile in the technique:
    if ( !_parentTileSet && _terrain.valid() )
    {
        osg::ref_ptr<Tile> parentTile;
        //Take a reference
        osg::ref_ptr< TerrainNode > terrain = _terrain.get();
        if (terrain.valid())
        {
            terrain->getTile( _key.createParentKey().getTileId(), parentTile );
            CustomTerrainTechnique* tech = dynamic_cast<CustomTerrainTechnique*>( _tech.get() );
            if ( tech )
                tech->setParentTile( parentTile.get() );
            _parentTileSet = true;
        }
    }

    // this block runs the first time the tile is traversed while in the scene graph.
    if ( !_hasBeenTraversed )
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
        {
            Threading::ScopedWriteLock lock( this->_tileLayersMutex );
            {
                if ( !_hasBeenTraversed && _terrain.valid() )
                {
                    _hasBeenTraversed = true;

                    // we constructed this tile with an update traversal count of 1 so it would get
                    // here and we could register the tile. Now we can decrement it back to normal.
                    // this MUST be called from the UPDATE traversal.
                    ADJUST_UPDATE_TRAV_COUNT( this, -1 );
                }
            }
        }
    }

    // code copied from osgTerrain::TerrainTile... TODO: evaluate this... -gw
    if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
        osg::ClusterCullingCallback* ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
        if (ccc)
        {
            if (ccc->cull(&nv,0,static_cast<osg::State *>(0))) return;
        }
    }

    if ( _dirty )
    {
        init();
    }

    if ( _tech.valid() )
    {
        _tech->traverse( nv );
    }
}

void
Tile::releaseGLObjects(osg::State* state) const
{
    osg::Node::releaseGLObjects( state );
    
    if ( _tech.valid() )
    {
        //NOTE: crashes sometimes if OSG_RELEASE_DELAY is set -gw
        _tech->releaseGLObjects( state );
    }
}

//------------------------------------------------------------------------

TileFrame::TileFrame( Tile* tile ) :
_tileKey(tile->getKey())
{
    Threading::ScopedReadLock sharedLock( tile->_tileLayersMutex );
    _colorLayers    = tile->_colorLayers;
    _elevationLayer = tile->getElevationLayer();
    _locator        = tile->getLocator();
    osg::ref_ptr< TerrainNode > terrain = tile->getTerrain();
    if (terrain.valid())
    {
        _sampleRatio  = terrain->getSampleRatio();
    }
    _masks          = MaskLayerVector(tile->getTerrainMasks());
}
