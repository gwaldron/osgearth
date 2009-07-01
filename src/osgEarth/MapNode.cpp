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

MapNode::MapNode(MapConfig& mapConfig ):
_mapConfig(mapConfig)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( s_mapNodeCacheMutex );
    _id = s_mapNodeID++;

    _mapConfig.setId( _id );
    _mapConfig.initialize();


    const osgDB::ReaderWriter::Options* global_options = mapConfig.getGlobalOptions();
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ? 
        new osgDB::ReaderWriter::Options( *global_options ) :
        NULL;

    // transcribe proxy settings:
    if ( !mapConfig.getProxyHost().empty() )
    {
        if ( !local_options.valid() )
            local_options = new osgDB::ReaderWriter::Options();

        std::stringstream buf;
        buf << local_options->getOptionString() << " "
            << "OSG_CURL_PROXY=" << mapConfig.getProxyHost() << " "
            << "OSG_CURL_PROXYPORT=" << mapConfig.getProxyPort();
        local_options->setOptionString( buf.str() );
    }

    osg::notify(osg::INFO) 
        << "[osgEarth] Map: options string = " 
        << (local_options.valid()? local_options->getOptionString() : "<empty>")
        << std::endl;

    _mapConfig.setGlobalOptions( local_options.get() );

    if (mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC || 
        mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC_CUBE )
    {     
        _engine = new GeocentricMap();
    }
    else
    {
        _engine = new ProjectedMap();
    }

    updateStateSet();


    // Note: CSN must always be at the top
    osg::CoordinateSystemNode* csn = createCoordinateSystemNode();

    // go through and build the root nodesets.
    int faces_ok = 0;
    for( int face = 0; face < getProfile()->getNumFaces(); face++ )
    {
        EarthTerrain* terrain = new EarthTerrain;
        terrain->setVerticalScale( _mapConfig.getVerticalScale() );
        terrain->setSampleRatio( _mapConfig.getSampleRatio() );
        csn->addChild( terrain );
        _terrains.push_back( terrain );

        std::vector< osg::ref_ptr<TileKey> > keys;
        getProfile()->getFaceProfile( face )->getRootKeys( keys, face );

        int numAdded = 0;
        for (unsigned int i = 0; i < keys.size(); ++i)
        {
            osg::Node* node = _engine->createNode( _mapConfig, terrain, keys[i].get() );
            if (node)
            {
                terrain->addChild(node);
                numAdded++;
            }
            else
            {
                osg::notify(osg::NOTICE) << "[osgEarth::MapEngine] Couldn't get tile for " << keys[i]->str() << std::endl;
            }
        }
        if ( numAdded == keys.size() )
            faces_ok++;
    }

    addChild( csn );

    registerMapNode(this);
}

MapNode::~MapNode()
{
    unregisterMapNode(_id);
}


MapEngine*
MapNode::getEngine() const
{
    return _engine.get();
}

const Profile*
MapNode::getProfile() const 
{
    return _mapConfig.getProfile();
}

bool
MapNode::isOK() const
{
    return _mapConfig.isOK();
}

osg::CoordinateSystemNode*
MapNode::createCoordinateSystemNode() const
{
    osg::CoordinateSystemNode* csn = getProfile()->getSRS()->createCoordinateSystemNode();

    if (_mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_PROJECTED)
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
    return dynamic_cast<GeocentricMap*>( _engine.get() ) != NULL;
}

unsigned int
MapNode::getImageSourceIndex( TileSource* source ) const
{
    for (unsigned int i = 0; i < _mapConfig.getImageSources().size(); ++i)
    {
        if (_mapConfig.getImageSources()[i].get() == source) return i;
    }
    return _mapConfig.getImageSources().size();
}

unsigned int
MapNode::getHeightFieldSourceIndex( TileSource* source ) const
{
    for (unsigned int i = 0; i < _mapConfig.getHeightFieldSources().size(); ++i)
    {
        if (_mapConfig.getHeightFieldSources()[i].get() == source) return i;
    }
    return _mapConfig.getHeightFieldSources().size();
}

TileSource*
MapNode::getImageSource( unsigned int index ) const
{
    return index < _mapConfig.getImageSources().size() ? _mapConfig.getImageSources()[index].get() : 0;
}

TileSource*
MapNode::getHeightFieldSource( unsigned int index) const
{    
    return index < _mapConfig.getHeightFieldSources().size() ? _mapConfig.getHeightFieldSources()[index].get() : 0;
}

unsigned int
MapNode::getNumImageSources() const
{
    return _mapConfig.getImageSources().size();
}

unsigned int
MapNode::getNumHeightFieldSources() const
{
    return _mapConfig.getHeightFieldSources().size();
}

TileSource*
MapNode::createTileSource(const osgEarth::SourceConfig& sourceConfig)
{
    //Create the TileSource
    TileSourceFactory factory;
    osg::ref_ptr<TileSource> tileSource = factory.createMapTileSource(sourceConfig, _mapConfig );
    if (tileSource.valid())
    {
        const Profile* profile = tileSource->initProfile( getProfile(), _mapConfig.getFilename() );
        if (!profile)
        {
            osg::notify(osg::NOTICE) << "[osgEarth::MapEngine] Could not initialize profile " << std::endl;
            tileSource = NULL;
        }
    }
    return tileSource.release();
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
MapNode::addImageSource( const SourceConfig& sourceConfig)
{
    TileSource* source = createTileSource( sourceConfig );
    osg::notify(osg::INFO) << "[osgEarth::Map::addImageSource] Begin " << std::endl;
    if (source)
    {        
        osg::notify(osg::INFO) << "[osgEarth::Map::addImageSource] Waiting for lock..." << std::endl;
        OpenThreads::ScopedWriteLock lock( _mapConfig.getSourceMutex());
        osg::notify(osg::INFO) << "[osgEarth::Map::addImageSource] Obtained lock " << std::endl;

        //Add the source to the list
        _mapConfig.getImageSources().push_back( source );

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
                osg::ref_ptr< TileKey > key = new TileKey( i, tileId.level, tileId.x, tileId.y, getProfile()->getFaceProfile( i ) );

                osg::ref_ptr< GeoImage > geoImage = _engine->createValidGeoImage( source, key.get() );

                if (geoImage.valid())
                {
                    double img_min_lon, img_min_lat, img_max_lon, img_max_lat;

                    //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
                    osg::ref_ptr<osgTerrain::Locator> img_locator; // = key->getProfile()->getSRS()->createLocator();

                    // Use a special locator for mercator images (instead of reprojecting)
                    if ( geoImage->getSRS()->isMercator() )
                    {
                        GeoExtent geog_ext = geoImage->getExtent().transform(geoImage->getExtent().getSRS()->getGeographicSRS());
                        geog_ext.getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                        img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
                        img_locator = new MercatorLocator( *img_locator.get(), geoImage->getExtent() );
                        //Transform the mercator extents to geographic
                    }
                    else
                    {
                        geoImage->getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);
                        img_locator = key->getProfile()->getSRS()->createLocator( img_min_lon, img_min_lat, img_max_lon, img_max_lat );
                    }

                    //Set the CS to geocentric is we are dealing with a geocentric map
                    if (_mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC || _mapConfig.getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC_CUBE)
                    {
                        img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
                    }

                    osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( geoImage->getImage() );
                    img_layer->setLocator( img_locator.get());

                    unsigned int newLayer = _mapConfig.getImageSources().size()-1;
                    osg::notify(osg::INFO) << "Inserting layer at position " << newLayer << std::endl;
                    itr->get()->setColorLayer(newLayer, img_layer );
                }
                else
                {
                    osg::notify(osg::NOTICE) << "Could not create geoimage for " << source->getName() << " " << key->str() << std::endl;
                }
                itr->get()->setDirty(true);
            }
        }

        updateStateSet();       
       if (_sourceCallback.valid()) _sourceCallback->imageSourceAdded( source, _mapConfig.getImageSources().size()-1 );
    }
}

void
MapNode::addHeightFieldSource( const SourceConfig& sourceConfig )
{
    osg::notify(osg::INFO) << "[osgEarth::MapEngine::addHeightFieldSource] Begin " << std::endl;

    TileSource* source = createTileSource( sourceConfig );
    if (source)
    {        
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::addHeightFieldSource] Waiting for lock..." << std::endl;
        OpenThreads::ScopedWriteLock lock( _mapConfig.getSourceMutex());
        osg::notify(osg::INFO) << "[osgEarth::MapEngine::addHeightFieldSource] Obtained lock " << std::endl;

        //Add the layer to the list        
        _mapConfig.getHeightFieldSources().push_back( source );

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
                osg::ref_ptr< TileKey > key = new TileKey( i, tileId.level, tileId.x, tileId.y, getProfile()->getFaceProfile( i ) );

                osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
                if (heightFieldLayer)
                {
                    osg::HeightField* hf = _engine->createHeightField( _mapConfig, key, true );
                    if (!hf) hf = MapEngine::createEmptyHeightField( key.get() );
                    heightFieldLayer->setHeightField( hf );
                    hf->setSkirtHeight( itr->get()->getBound().radius() * _mapConfig.getSkirtRatio() );
                }
                itr->get()->setDirty(true);
            }
        }

        if (_sourceCallback.valid()) _sourceCallback->heightfieldSourceAdded( source, _mapConfig.getHeightFieldSources().size()-1 );
    }
}

void
MapNode::removeImageSource( unsigned int index )
{
    osg::notify(osg::INFO) << "[osgEarth::Map::removeImageSource] Begin " << std::endl;
    osg::notify(osg::INFO) << "[osgEarth::Map::removeImageSource] Waiting for lock" << std::endl;
    OpenThreads::ScopedWriteLock lock( _mapConfig.getSourceMutex());
    osg::notify(osg::INFO) << "[osgEarth::Map::removeImageSource] Obtained for lock" << std::endl;

    if (index >= _mapConfig.getImageSources().size())
    {
        osg::notify(osg::NOTICE) << "[osgEarth::Map::removeImageSource] Could not find layer " << std::endl;
        return;
    }

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

    osg::ref_ptr<TileSource> source = _mapConfig.getImageSources()[index];

    //Erase the layer from the list
    _mapConfig.getImageSources().erase( _mapConfig.getImageSources().begin() + index );

    if (_sourceCallback.valid()) _sourceCallback->imageSourceRemoved( source.get(), index);
    updateStateSet();
    osg::notify(osg::INFO) << "[osgEarth::Map::removeImageSource] end " << std::endl;   
}

void
MapNode::removeHeightFieldSource( unsigned int index )
{
    osg::notify(osg::INFO) << "[osgEarth::Map::removeHeightFieldSource] Begin " << std::endl;
    osg::notify(osg::INFO) << "[osgEarth::Map::removeHeightFieldSource] Waiting for lock" << std::endl;
    OpenThreads::ScopedWriteLock lock( _mapConfig.getSourceMutex());
    osg::notify(osg::INFO) << "[osgEarth::Map::removeHeightFieldSource] Obtained for lock" << std::endl;

    if (index >= _mapConfig.getHeightFieldSources().size())
    {
        osg::notify(osg::NOTICE) << "[osgEarth::Map::removeHeightFieldSource] Could not find layer " << std::endl;
        return;
    }


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
            osg::ref_ptr< TileKey > key = new TileKey( i, tileId.level, tileId.x, tileId.y, getProfile()->getFaceProfile( i ) );
            osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
            if (heightFieldLayer)
            {
                osg::HeightField* hf = _engine->createHeightField( _mapConfig, key, true );
                if (!hf) hf = MapEngine::createEmptyHeightField( key.get() );
                heightFieldLayer->setHeightField( hf );
                hf->setSkirtHeight( itr->get()->getBound().radius() * _mapConfig.getSkirtRatio() );
            }
            itr->get()->setDirty(true);
        }
    }

    osg::ref_ptr<TileSource> source = _mapConfig.getHeightFieldSources()[index];
    _mapConfig.getHeightFieldSources().erase( _mapConfig.getHeightFieldSources().begin() + index );
    if (_sourceCallback.valid()) _sourceCallback->heightfieldSourceRemoved( source.get(), index);

    osg::notify(osg::INFO) << "[osgEarth::Map::removeHeightFieldSource] end " << std::endl;
}

void
MapNode::moveImageSource( unsigned int index, unsigned int position )
{
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveImageSource] Begin" << std::endl;
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveImageSource] Waiting for lock..." << std::endl;
    OpenThreads::ScopedWriteLock lock(_mapConfig.getSourceMutex());
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveImageSource] Obtained lock" << std::endl;

    if (index >= _mapConfig.getImageSources().size())
    {
        osg::notify(osg::NOTICE) << "[osgEarth::GeocentricMapEngine::moveImageSource] Could not find layer" << std::endl;
        return;
    }

    //Take a reference to the incoming source
    osg::ref_ptr<TileSource> s = _mapConfig.getImageSources()[index];
    //Insert the source into it's new position
    _mapConfig.getImageSources().erase(_mapConfig.getImageSources().begin() + index);
    _mapConfig.getImageSources().insert(_mapConfig.getImageSources().begin() + position, s.get());

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
            osg::ref_ptr< osgTerrain::Layer > layer = layers[index];
            layers.erase(layers.begin() + index);
            layers.insert(layers.begin() + position, layer.get());

            for (unsigned int i = 0; i < layers.size(); ++i)
            {
                itr->get()->setColorLayer( i, layers[i].get() );
            }
            itr->get()->setDirty(true);
        }
    } 
    if (_sourceCallback.valid()) _sourceCallback->imageSourceMoved( s.get(), index, position);
    updateStateSet();
    osg::notify(osg::INFO) << "[osgEarth::MapEngine::moveImageSource] end " << std::endl;
}

void
MapNode::moveHeightFieldSource( unsigned int index, unsigned int position )
{
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveHeightFieldSource] Begin" << std::endl;
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveHeightFieldSource] Waiting for lock..." << std::endl;
    OpenThreads::ScopedWriteLock lock( _mapConfig.getSourceMutex());
    osg::notify(osg::INFO) << "[osgEarth::GeocentricMapEngine::moveHeightFieldSource] Obtained lock" << std::endl;

    if (index >= _mapConfig.getHeightFieldSources().size())
    {
        osg::notify(osg::NOTICE) << "[osgEarth::GeocentricMapEngine::moveHeightFieldSource] Could not find layer" << std::endl;
        return;
    }

    //Take a reference to the TileSource
    osg::ref_ptr<TileSource> s = _mapConfig.getHeightFieldSources()[index];
    //Insert the source into it's new position
    _mapConfig.getHeightFieldSources().erase(_mapConfig.getHeightFieldSources().begin() + index);
    _mapConfig.getHeightFieldSources().insert(_mapConfig.getHeightFieldSources().begin() + position, s.get());

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
            osg::ref_ptr< TileKey > key = new TileKey( i, tileId.level, tileId.x, tileId.y, getProfile()->getFaceProfile( i ) );
            osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(itr->get()->getElevationLayer() );
            if (heightFieldLayer)
            {
                osg::HeightField* hf = _engine->createHeightField( _mapConfig, key, true );
                if (!hf) hf = MapEngine::createEmptyHeightField( key.get() );
                heightFieldLayer->setHeightField( hf );
                hf->setSkirtHeight( itr->get()->getBound().radius() * _mapConfig.getSkirtRatio() );
            }                
            itr->get()->setDirty(true);
        }
    }
    if (_sourceCallback.valid()) _sourceCallback->heightfieldSourceMoved( s.get(), index, position);

    osg::notify(osg::INFO) << "[osgEarth::MapEngine::moveHeightFieldSource] end " << std::endl;
}

void MapNode::updateStateSet()
{
    if (_mapConfig.getCombineLayers())
    {
        int numLayers = getNumImageSources();

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
