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

#include <osgEarth/MapNode>
#include <osgEarth/Mercator>
#include <osgEarth/GeocentricMap>
#include <osgEarth/ProjectedMap>
#include <osgEarth/FindNode>
#include <osgEarth/EarthTerrainTechnique>
#include <osgEarth/TileSourceFactory>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/Notify>

using namespace osgEarth;

//static
OpenThreads::ReentrantMutex MapNode::s_mapNodeCacheMutex;
static unsigned int s_mapNodeID = 0;
//Caches the MapNodes that have been created
typedef std::map<unsigned int, osg::observer_ptr<MapNode> > MapNodeCache;



static
MapNodeCache& getMapNodeCache()
{
    static MapNodeCache s_cache;
    return s_cache;
}

struct MapNodeMapCallbackProxy : public MapCallback
{
    MapNodeMapCallbackProxy(MapNode* node) : _node(node) { }
    osg::observer_ptr<MapNode> _node;

    void onMapProfileEstablished( const Profile* profile ) {
        _node->onMapProfileEstablished(profile);
    }
    void onMapLayerAdded( MapLayer* layer, unsigned int index ) {
        _node->onMapLayerAdded(layer, index);
    }
    void onMapLayerRemoved( MapLayer* layer, unsigned int index ) {
        _node->onMapLayerRemoved(layer, index);
    }
    void onMapLayerMoved( MapLayer* layer, unsigned int oldIndex, unsigned int newIndex ) {
        _node->onMapLayerMoved(layer,oldIndex,newIndex);
    }
};


void
MapNode::registerMapNode(MapNode* mapNode)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    getMapNodeCache()[mapNode->_id] = mapNode;
    osg::notify(osg::INFO) << "[osgEarth::MapNode] Registered " << mapNode->_id << std::endl;
}

void
MapNode::unregisterMapNode(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    MapNodeCache::iterator k = getMapNodeCache().find( id);
    if (k != getMapNodeCache().end())
    {
        getMapNodeCache().erase(k);
        osg::notify(osg::INFO) << "[osgEarth::MapNode] Unregistered " << id << std::endl;
    }
}

MapNode*
MapNode::getMapNodeById(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_mapNodeCacheMutex);
    MapNodeCache::const_iterator k = getMapNodeCache().find( id);
    if (k != getMapNodeCache().end()) return k->second.get();
    return 0;
}

unsigned int
MapNode::getId() const
{
    return _id;
}


MapNode::MapNode() :
_map( new Map() )
{
    init();
}

MapNode::MapNode( Map* map ) :
_map( map )
{
    init();
}

MapNode::MapNode( const MapEngineProperties& engineProps ) :
_map( new Map() ),
_engineProps( engineProps )
{
    init();
}

MapNode::MapNode( Map* map, const MapEngineProperties& engineProps ) :
_map( map? map : new Map() ),
_engineProps( engineProps )
{
    init();
}

void
MapNode::init()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( s_mapNodeCacheMutex );
    _id = s_mapNodeID++;

    _map->setId( _id );

    const osgDB::ReaderWriter::Options* global_options = _map->getGlobalOptions();
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ? 
        new osgDB::ReaderWriter::Options( *global_options ) :
        NULL;

    // transcribe proxy settings:
    if ( !_engineProps.getProxyHost().empty() )
    {
        if ( !local_options.valid() )
            local_options = new osgDB::ReaderWriter::Options();

        std::stringstream buf;
        buf << local_options->getOptionString() << " "
            << "OSG_CURL_PROXY=" << _engineProps.getProxyHost() << " "
            << "OSG_CURL_PROXYPORT=" << _engineProps.getProxyPort();
        local_options->setOptionString( buf.str() );
    }

    osg::notify(osg::INFO) 
        << "[osgEarth] Map: options string = " 
        << (local_options.valid()? local_options->getOptionString() : "<empty>")
        << std::endl;

    _map->setGlobalOptions( local_options.get() );

    // create the map engine that wil geneate tiles for this node:
    _engine = _map->createMapEngine( _engineProps );

    // handle an already-established map profile:
    if ( _map->getProfile() )
    {
        onMapProfileEstablished( _map->getProfile() );
    }

    // go through the map and process any already-installed layers:
    unsigned int index = 0;
    for( MapLayerList::const_iterator i = _map->getHeightFieldMapLayers().begin(); i != _map->getHeightFieldMapLayers().end(); i++ )
    {
        onMapLayerAdded( i->get(), index++ );
    }
    index = 0;
    for( MapLayerList::const_iterator j = _map->getImageMapLayers().begin(); j != _map->getImageMapLayers().end(); j++ )
    {
        onMapLayerAdded( j->get(), index++ );
    }

    updateStateSet();

    // install a layer callback for processing further map actions:
    _map->addMapCallback( new MapNodeMapCallbackProxy(this) );

    registerMapNode(this);
}

MapNode::~MapNode()
{
    unregisterMapNode(_id);
}

Map*
MapNode::getMap()
{
    return _map.get();
}

MapEngine*
MapNode::getEngine() const
{
    return _engine.get();
}

osg::CoordinateSystemNode*
MapNode::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = _map->getProfile()->getSRS()->createCoordinateSystemNode();

    if ( _map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED )
    {
        // Setting the ellipsoid to NULL indicates that the CS should be interpreted 
        // as PROJECTED instead of GEOGRAPHIC.
        csn->setEllipsoidModel( NULL );
    }
    return csn;
}


MapNode*
MapNode::findMapNode( osg::Node* graph )
{
    return findTopMostNodeOfType<MapNode>( graph );
}

osg::CoordinateSystemNode*
MapNode::findCoordinateSystemNode( osg::Node* graph )
{
    return findTopMostNodeOfType<osg::CoordinateSystemNode>( graph );
}

bool
MapNode::isGeocentric() const
{
    return dynamic_cast<GeocentricMapEngine*>( _engine.get() ) != NULL;
}

unsigned int
MapNode::getNumTerrains() const
{
    return _terrains.size();
}

osgEarth::EarthTerrain*
MapNode::getTerrain( unsigned int i ) const
{
    return _terrains[i].get();
}

void
MapNode::onMapProfileEstablished( const Profile* mapProfile )
{
    // Note: CSN must always be at the top
    osg::CoordinateSystemNode* csn = createCoordinateSystemNode();

    // go through and build the root nodesets.
    int faces_ok = 0;
    for( int face = 0; face < _map->getProfile()->getNumFaces(); face++ )
    {
        EarthTerrain* terrain = new EarthTerrain;
        terrain->setVerticalScale( _engineProps.getVerticalScale() );
        terrain->setSampleRatio( _engineProps.getSampleRatio() );
        csn->addChild( terrain );
        _terrains.push_back( terrain );

        std::vector< osg::ref_ptr<TileKey> > keys;
        _map->getProfile()->getFaceProfile( face )->getRootKeys( keys, face );

        int numAdded = 0;
        for (unsigned int i = 0; i < keys.size(); ++i)
        {
            osg::Node* node = _engine->createNode( _map.get(), terrain, keys[i].get() );
            if (node)
            {
                terrain->addChild(node);
                numAdded++;
            }
            else
            {
                osg::notify(osg::NOTICE) << "[osgEarth::MapNode] Couldn't get tile for " << keys[i]->str() << std::endl;
            }
        }
        if ( numAdded == keys.size() )
        {
            faces_ok++;
        }
    }

    addChild( csn );
}

void
MapNode::onMapLayerAdded( MapLayer* layer, unsigned int index )
{
    if ( layer && layer->getTileSource() )
    {        
        if ( layer->getType() == MapLayer::TYPE_IMAGE )
        {
            addImageTileSource( layer->getTileSource() );
        }
        else if ( layer->getType() == MapLayer::TYPE_HEIGHTFIELD )
        {
            addHeightFieldTileSource( layer->getTileSource() );
        }
    }
}

void
MapNode::addImageTileSource( TileSource* source )
{
    OpenThreads::ScopedWriteLock lock( _map->getMapDataMutex() );

    for( unsigned int i=0; i<_terrains.size(); i++ )
    {            
        EarthTerrain* terrain = _terrains[i].get();
        EarthTerrain::TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());

            //Create a TileKey from the TileID
            osgTerrain::TileID tileId = itr->get()->getTileID();
		    osg::ref_ptr< TileKey > key = new TileKey( i, TileKey::getLOD(tileId), tileId.x, tileId.y, _map->getProfile()->getFaceProfile( i ) );

            osg::ref_ptr< GeoImage > geoImage = _engine->createValidGeoImage( source, key.get() );

            if (geoImage.valid())
            {
                double img_min_lon, img_min_lat, img_max_lon, img_max_lat;

                //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
                osg::ref_ptr<osgTerrain::Locator> img_locator; // = key->getProfile()->getSRS()->createLocator();

                
                // Use a special locator for mercator images (instead of reprojecting).
                // We do this under 2 conditions when we have mercator tiles:
                // a) The map is geocentric; or
                // b) The map is projected but is also "geographic" (i.e., plate carre)
                bool isGeocentric = _map->getCoordinateSystemType() != Map::CSTYPE_PROJECTED;
                bool isGeographic = _map->getProfile()->getSRS()->isGeographic();
                bool canUseMercatorLocator = geoImage->getSRS()->isMercator() && (isGeocentric || isGeographic);

                if ( canUseMercatorLocator && _engineProps.getUseMercatorLocator() )
                {
                    GeoExtent geog_ext = geoImage->getExtent().transform(geoImage->getExtent().getSRS()->getGeographicSRS());
                    geog_ext.getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                    img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
                    img_locator = new MercatorLocator( *img_locator.get(), geoImage->getExtent() );
                }
                else
                {
                    geoImage->getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                    img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
                }

                //Set the CS to geocentric is we are dealing with a geocentric map
                if ( _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC || _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC_CUBE)
                {
                    img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
                }

                osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( geoImage->getImage() );
                img_layer->setLocator( img_locator.get());

                unsigned int newLayer = _map->getImageMapLayers().size() - 1;
                osg::notify(osg::INFO) << "Inserting layer at position " << newLayer << std::endl;
                itr->get()->setColorLayer( newLayer, img_layer );
            }
            else
            {
                osg::notify(osg::NOTICE) << "Could not create geoimage for " << source->getName() << " " << key->str() << std::endl;
            }
            itr->get()->setDirty(true);
        }
    }

    updateStateSet();       
}


void
MapNode::addHeightFieldTileSource( TileSource* source )
{
    osg::notify(osg::INFO) << "[osgEarth::MapEngine::addHeightFieldSource] Begin " << std::endl;

    OpenThreads::ScopedWriteLock lock( _map->getMapDataMutex() ); // _mapConfig.getSourceMutex());

    for (unsigned int i = 0; i < _terrains.size(); ++i)
    {            
        EarthTerrain* terrain = _terrains[i].get();
        EarthTerrain::TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());

            //Create a TileKey from the TileID
            osgTerrain::TileID tileId = itr->get()->getTileID();
			osg::ref_ptr< TileKey > key = new TileKey( i, TileKey::getLOD(tileId), tileId.x, tileId.y, _map->getProfile()->getFaceProfile( i ) );

            osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
            if (heightFieldLayer)
            {
                osg::HeightField* hf = _engine->createHeightField( _map.get(), key.get(), true );
                if (!hf) hf = MapEngine::createEmptyHeightField( key.get() );
                heightFieldLayer->setHeightField( hf );
                hf->setSkirtHeight( itr->get()->getBound().radius() * _engineProps.getSkirtRatio() );
            }
            itr->get()->setDirty(true);
        }
    }
}

void
MapNode::onMapLayerRemoved( MapLayer* layer, unsigned int index )
{
    if ( layer )
    {
        if ( layer->getType() == MapLayer::TYPE_IMAGE )
        {
            removeImageTileSource( index );
        }
        else if ( layer->getType() == MapLayer::TYPE_HEIGHTFIELD )
        {
            removeHeightFieldTileSource( index );
        }
    }
}

void
MapNode::removeImageTileSource( unsigned int index )
{
    OpenThreads::ScopedWriteLock lock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrains.size(); ++i)
    {            
        EarthTerrain* terrain = _terrains[i].get();
        EarthTerrain::TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );

        for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());
            //An image layer was removed, so reorganize the color layers in the tiles to account for it's removal
            std::vector< osg::ref_ptr< osgTerrain::Layer > > layers;
            for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
            {   
                //Skip the layer that is being removed
                if (i != index)
                {
                    osgTerrain::Layer* imageLayer = itr->get()->getColorLayer(i);
                    if (imageLayer)
                    {
                        layers.push_back(imageLayer);
                    }
                }
                //Set the current value to NULL
                itr->get()->setColorLayer( i, NULL);
            }

            //Reset the color layers to the correct order
            for (unsigned int i = 0; i < layers.size(); ++i)
            {
                itr->get()->setColorLayer( i, layers[i].get() );
            }
            itr->get()->setDirty(true);
        }
    }

    updateStateSet();

    osg::notify(osg::INFO) << "[osgEarth::Map::removeImageSource] end " << std::endl;  
}

void
MapNode::removeHeightFieldTileSource( unsigned int index )
{
    OpenThreads::ScopedWriteLock lock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrains.size(); ++i)
    {            
        EarthTerrain* terrain = _terrains[i].get();
        EarthTerrain::TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        //osg::notify(osg::NOTICE) << "Found " << tiles.size() << std::endl;

        for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());
            osgTerrain::TileID tileId = itr->get()->getTileID();
			osg::ref_ptr< TileKey > key = new TileKey( i, TileKey::getLOD(tileId), tileId.x, tileId.y, _map->getProfile()->getFaceProfile( i ) );
            osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
            if (heightFieldLayer)
            {
                osg::HeightField* hf = _engine->createHeightField( _map.get(), key.get(), true );
                if (!hf) hf = MapEngine::createEmptyHeightField( key.get() );
                heightFieldLayer->setHeightField( hf );
                hf->setSkirtHeight( itr->get()->getBound().radius() * _engineProps.getSkirtRatio() );
            }
            itr->get()->setDirty(true);
        }
    }
}

void
MapNode::onMapLayerMoved( MapLayer* layer, unsigned int oldIndex, unsigned int newIndex )
{
    if ( layer )
    {
        if ( layer->getType() == MapLayer::TYPE_IMAGE )
        {
            moveImageTileSource( oldIndex, newIndex );
        }
        else if ( layer->getType() == MapLayer::TYPE_HEIGHTFIELD )
        {
            moveHeightFieldTileSource( oldIndex, newIndex );
        }
    }
}

void
MapNode::moveImageTileSource( unsigned int oldIndex, unsigned int newIndex )
{
    OpenThreads::ScopedWriteLock lock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrains.size(); ++i)
    {            
        EarthTerrain* terrain = _terrains[i].get();
        EarthTerrain::TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());
            //Collect the current color layers
            std::vector< osg::ref_ptr< osgTerrain::Layer > > layers;

            for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
            {              
                layers.push_back(itr->get()->getColorLayer(i));
            }

            //Swap the original position
            osg::ref_ptr< osgTerrain::Layer > layer = layers[oldIndex];
            layers.erase(layers.begin() + oldIndex);
            layers.insert(layers.begin() + newIndex, layer.get());

            for (unsigned int i = 0; i < layers.size(); ++i)
            {
                itr->get()->setColorLayer( i, layers[i].get() );
            }
            itr->get()->setDirty(true);
        }
    } 

    updateStateSet();
}

void
MapNode::moveHeightFieldTileSource( unsigned int oldIndex, unsigned int newIndex )
{
    OpenThreads::ScopedWriteLock lock( _map->getMapDataMutex() );

    for (unsigned int i = 0; i < _terrains.size(); ++i)
    {            
        EarthTerrain* terrain = _terrains[i].get();
        EarthTerrain::TerrainTileList tiles;
        terrain->getTerrainTiles( tiles );
        osg::notify(osg::INFO) << "Found " << tiles.size() << std::endl;

        for (EarthTerrain::TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
        {
            OpenThreads::ScopedLock< OpenThreads::Mutex > tileLock(((EarthTerrainTechnique*)itr->get()->getTerrainTechnique())->getMutex());

            osgTerrain::TileID tileId = itr->get()->getTileID();
			osg::ref_ptr< TileKey > key = new TileKey( i, TileKey::getLOD(tileId), tileId.x, tileId.y, _map->getProfile()->getFaceProfile( i ) );
            osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
            if (heightFieldLayer)
            {
                osg::HeightField* hf = _engine->createHeightField( _map.get(), key.get(), true );
                if (!hf) hf = MapEngine::createEmptyHeightField( key.get() );
                heightFieldLayer->setHeightField( hf );
                hf->setSkirtHeight( itr->get()->getBound().radius() * _engineProps.getSkirtRatio() );
            }                
            itr->get()->setDirty(true);
        }
    }
}

void MapNode::updateStateSet()
{
    if ( _engineProps.getCombineLayers() )
    {
        int numLayers = _map->getImageMapLayers().size(); //getNumImageSources();

        osg::StateSet* stateset = getOrCreateStateSet();

        if (numLayers == 1)
        {
            osg::TexEnv* texenv = new osg::TexEnv(osg::TexEnv::MODULATE);
            stateset->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
        }
        else if (numLayers >= 2)
        {
            //Blend together the colors and accumulate the alpha values of textures 0 and 1 on unit 0
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateset->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
            }


            //For textures 2 and beyond, blend them together with the previous
            //Add the alpha values of this unit and the previous unit
            for (int unit = 1; unit < numLayers-1; ++unit)
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateset->setTextureAttributeAndModes(unit, texenv, osg::StateAttribute::ON);
            }

            //Modulate the colors to get proper lighting on the last unit
            //Keep the alpha results from the previous stage
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::MODULATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::REPLACE);

                texenv->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                stateset->setTextureAttributeAndModes(numLayers-1, texenv, osg::StateAttribute::ON);
            }
        }
    }
}


