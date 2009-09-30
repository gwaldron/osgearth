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
#include <osgEarth/VersionedTerrain>
#include <osgEarth/Registry>
#include <osgEarth/Locators>
#include <OpenThreads/ScopedLock>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>

using namespace osgEarth;
using namespace OpenThreads;


/*****************************************************************************/

struct TileLayerRequest : public TaskRequest
{
    TileLayerRequest( const TileKey* key, TileDataFactory* factory )
        : _key(key), _factory(factory) { }
    virtual bool isColorLayerRequest() const { return false; }
    virtual bool isElevLayerRequest() const { return false; }
    osg::ref_ptr<const TileKey> _key;
    osg::ref_ptr<TileDataFactory> _factory;
};

struct TileColorLayerRequest : public TileLayerRequest
{
    TileColorLayerRequest( int layerIndex, const TileKey* key, TileDataFactory* factory )
        : TileLayerRequest( key, factory ), _layerIndex( layerIndex ) { }
    bool isColorLayerRequest() const { return true; }
    void operator()( TaskProgress* p )
    {
        osg::ref_ptr<GeoImage> image = _factory->createImage( _key.get(), _layerIndex );
        if ( image.valid() )
            _result = _factory->createImageLayer( _key.get(), image.get() );
    }
    int _layerIndex;
};

struct TileElevationLayerRequest : public TileLayerRequest
{
    TileElevationLayerRequest( const TileKey* key, TileDataFactory* factory )
        : TileLayerRequest( key, factory ) { }
    bool isElevLayerRequest() const { return true; }
    void operator()( TaskProgress* p )
    {
        osg::ref_ptr<osg::HeightField> hf = _factory->createHeightField( _key.get() );
        if ( hf.valid() )
            _result = _factory->createHeightFieldLayer( _key.get(), hf.get() );
    }
};

/*****************************************************************************/


VersionedTile::VersionedTile( const TileKey* key ) :
_key( key ),
_useLayerRequests( false ),
_terrainRevision( -1 ),
_tileRevision( 0 ),
_requestsInstalled( false )
{
    setTileID( key->getTileId() );
    this->setNumChildrenRequiringUpdateTraversal(1);
}

VersionedTerrain*
VersionedTile::getVersionedTerrain() {
    return static_cast<VersionedTerrain*>(getTerrain());
}
const VersionedTerrain*
VersionedTile::getVersionedTerrain() const {
    return static_cast<const VersionedTerrain*>(getTerrain());
}

void
VersionedTile::setUseLayerRequests( bool value )
{
    _useLayerRequests = value;
}

int
VersionedTile::getTerrainRevision() const
{
    return _terrainRevision;
}

void
VersionedTile::setTerrainRevision( int revision )
{
    _terrainRevision = revision;
}

bool
VersionedTile::isInSyncWithTerrain() const
{
    return _terrainRevision == getVersionedTerrain()->getRevision();
}

int
VersionedTile::getTileRevision() const
{
    return _tileRevision;
}

void
VersionedTile::incrementTileRevision()
{
    _tileRevision++;
}

void
VersionedTile::servicePendingRequests( int stamp )
{
    if ( !_requestsInstalled )
    {
        TileDataFactory* factory = getVersionedTerrain()->getTileDataFactory();
        if ( factory )
        {
            if ( this->getElevationLayer() )
            {
                TileLayerRequest* r = new TileElevationLayerRequest( _key.get(), factory );
                r->setPriority( (float)_key->getLevelOfDetail() );
                r->setStamp( stamp );
                _requests.push_back( r );
            }
            for( int k=0; k<getNumColorLayers(); k++ )
            {
                TaskRequest* r = new TileColorLayerRequest( k, _key.get(), factory );
                r->setPriority( (float)_key->getLevelOfDetail() );
                r->setStamp( stamp );
                _requests.push_back( r );
            }
        }
        _requestsInstalled = true;
    }

    if ( _requestsInstalled )
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            TileLayerRequest* r = static_cast<TileLayerRequest*>( i->get() );

            if ( r->isIdle() )
            {
                r->setStamp( stamp );
                osgEarth::Registry::instance()->getOrCreateTaskService()->add( r );
            }
            else if ( !r->isCompleted() )
            {
                r->setStamp( stamp );
            }
        }
    }
}

void VersionedTile::serviceCompletedRequests()
{
    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); )
    {
        TileLayerRequest* r = static_cast<TileLayerRequest*>( i->get() );

        if ( r->isCompleted() )
        {
            // TODO: merge new tile data into tile.
            if ( r->isElevLayerRequest() )
            {
                TileElevationLayerRequest* er = static_cast<TileElevationLayerRequest*>( r );
                osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>( er->getResult() );
                if ( hfLayer )
                {
                    this->setElevationLayer( hfLayer );
                    this->setDirty( true );
                }

                //osg::HeightField* hf = static_cast<osg::HeightField*>( er->getResult() );
                //if ( hf )
                //{
                //    //osg::notify(osg::NOTICE) << "Tile " << _key->str() << ": Merging heightfield data..." << std::endl;
                //    ((osgTerrain::HeightFieldLayer*)this->getElevationLayer())->setHeightField( hf );
                //    this->setDirty( true );
                //}
            }
            else // if ( r->isColorLayerRequest() )
            {
                TileColorLayerRequest* cr = static_cast<TileColorLayerRequest*>( r );
                osgTerrain::ImageLayer* imgLayer = static_cast<osgTerrain::ImageLayer*>( cr->getResult() );
                if ( imgLayer )
                {
                    this->setColorLayer( cr->_layerIndex, imgLayer );
                    this->setDirty( true );
                }

                //GeoImage* img = static_cast<GeoImage*>( cr->getResult() );
                //if ( img )
                //{
                //    //osg::notify(osg::NOTICE) << "Tile " << _key->str() << ": Merging image data..." << std::endl;
                //    osgTerrain::ImageLayer* imgLayer = (osgTerrain::ImageLayer*)this->getColorLayer( cr->_layerIndex );
                //    imgLayer->setImage( img->getImage() );
                //    const GeoExtent& e = img->getExtent();
                //    bool isPlateCarre = false;
                //    GeoLocator* locator = _key->getProfile()->getSRS()->createLocator(
                //        e.xMin(), e.yMin(), e.xMax(), e.yMax(), isPlateCarre );
                //    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
                //    imgLayer->setLocator( locator );
                //    this->setDirty( true );
                //}
            }

            // remove from the list
            i = _requests.erase( i );
        }
        else if ( r->isCanceled() )
        {
            i = _requests.erase( i );
        }
        else
        {
            i++;
        }
    }
}

void
VersionedTile::traverse( osg::NodeVisitor& nv )
{
    if ( _useLayerRequests )
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            servicePendingRequests( nv.getFrameStamp()->getFrameNumber() );
        }
        else if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
        {
            serviceCompletedRequests();
        }
    }
    osgTerrain::TerrainTile::traverse( nv );
}

/****************************************************************************/


VersionedTerrain::VersionedTerrain( TileDataFactory* factory ) :
_tileDataFactory( factory ),
_revision(0)
{
    //nop
}

void
VersionedTerrain::incrementRevision()
{
    // no need to lock; if we miss it, we'll get it the next time around
    _revision++;
}

int
VersionedTerrain::getRevision() const
{
    // no need to lock; if we miss it, we'll get it the next time around
    return _revision;
}

VersionedTile*
VersionedTerrain::getVersionedTile(const osgTerrain::TileID& tileID)
{
    ScopedLock<Mutex> lock(_mutex);

    TerrainTileMap::iterator itr = _terrainTileMap.find(tileID);
    if (itr == _terrainTileMap.end()) return 0;

    return static_cast<VersionedTile*>(itr->second);
}

void 
VersionedTerrain::getTerrainTiles( TerrainTileList& out_tiles )
{
    ScopedLock<Mutex> lock(_mutex);

    out_tiles.reserve( _terrainTileMap.size() );

    for(TerrainTileSet::iterator itr = _terrainTileSet.begin();
        itr != _terrainTileSet.end();
        ++itr)
    {
        out_tiles.push_back( (*itr) );
    }
}

TileDataFactory*
VersionedTerrain::getTileDataFactory() const {
    return _tileDataFactory.get();
}
