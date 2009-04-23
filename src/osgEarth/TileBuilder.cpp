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

#include <osgEarth/TileBuilder>
#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/ProjectedTileBuilder>
#include <osgEarth/Caching>
#include <osgEarth/Mercator>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Compositing>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>

#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/TexEnvCombine>
#include <osgFX/MultiTextureControl>
#include <osgDB/ReadFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <OpenThreads/ReentrantMutex>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;

static OpenThreads::ReentrantMutex s_tileBuilderCacheMutex;
static unsigned int s_tileBuilderID = 0;

//Struture that pairs a TileBuilder with an osg::Node
typedef std::pair<osg::ref_ptr<TileBuilder>,osg::Node*> TileBuilderNodePair;
typedef std::map<unsigned int, TileBuilderNodePair> TileBuilderCache;

/**
*The UnregisterTileBuilderObserver whole purpose in life is to listen for when the root node of a TileBuilder
*has been deleted and unregister the associated TileBuilder
*/
class UnregisterTileBuilderObserver : public osg::Observer
{
    virtual void objectDeleted(void* object)
    {
        osg::Node* node = (osg::Node*)object;
        //Find the TileBuilder for the Node that was deleted
        TileBuilder* tile_builder = TileBuilder::getTileBuilderByNode(node);
        if (tile_builder)
        {
            //osg::notify(osg::NOTICE) << "Node associated with TileBuilder " << tile_builder->getId() << " deleted" << std::endl;
            //Unregsiter the TileBuilder
            TileBuilder::unregisterTileBuilder(tile_builder->getId());
        }
    }
};

static UnregisterTileBuilderObserver s_unregisterTileBuilderObserver;

static
TileBuilderCache& getCache()
{
    static TileBuilderCache s_cache;
    return s_cache;
}

TileBuilder*
TileBuilder::create( MapConfig* map, const osgDB::ReaderWriter::Options* options )
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_tileBuilderCacheMutex);
    TileBuilder* result = NULL;
    if ( map )
    {
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = options ? 
            new osgDB::ReaderWriter::Options( *local_options ) :
            NULL;

        // transcribe proxy settings:
        if ( !map->getProxyHost().empty() )
        {
            if ( !local_options.valid() )
                local_options = new osgDB::ReaderWriter::Options();

            std::stringstream buf;
            buf << local_options->getOptionString() << " "
                << "OSG_CURL_PROXY=" << map->getProxyHost() << " "
                << "OSG_CURL_PROXYPORT=" << map->getProxyPort();
            local_options->setOptionString( buf.str() );
        }

        osg::notify(osg::INFO) 
            << "[osgEarth] TileBuilder: options string = " 
            << (local_options.valid()? local_options->getOptionString() : "<empty>")
            << std::endl;

        if ( map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
        {
            result = new GeocentricTileBuilder( map, local_options.get() );
        }
        else
        {
            result = new ProjectedTileBuilder( map, local_options.get() );
        }

        result->id = s_tileBuilderID++;
        osg::notify(osg::INFO) << "TileBuilder::create assigning id " << result->id << " to Tilebuilder " << std::endl;
    }
    return result;
}

void
TileBuilder::registerTileBuilder(TileBuilder* tileBuilder, osg::Node *node)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_tileBuilderCacheMutex);
    getCache()[tileBuilder->id] = TileBuilderNodePair(tileBuilder, node);
    node->addObserver(&s_unregisterTileBuilderObserver);
    osg::notify(osg::INFO) << "Registered " << tileBuilder->id << std::endl;
}

void
TileBuilder::unregisterTileBuilder(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_tileBuilderCacheMutex);
    TileBuilderCache::iterator k = getCache().find( id);
    if (k != getCache().end())
    {
        getCache().erase(k);
        osg::notify(osg::INFO) << "Unregistered " << id << std::endl;
    }
}

TileBuilder*
TileBuilder::getTileBuilderById(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_tileBuilderCacheMutex);
    TileBuilderCache::const_iterator k = getCache().find( id);
    if (k != getCache().end()) return k->second.first.get();
    return 0;
}

TileBuilder*
TileBuilder::getTileBuilderByNode(osg::Node* node)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_tileBuilderCacheMutex);
    for (TileBuilderCache::const_iterator k = getCache().begin(); k != getCache().end(); ++k)
    {
        if (k->second.second == node)
        {
            return k->second.first.get();
        }
    }
    return 0;
}

osg::Node*
TileBuilder::readNode( MapConfig* map )
{
    //Create a fake filename for this MapConfig by converting the pointer to a string and appending ".earth" to it.
    std::stringstream filename;
    filename << &map << ".earth";
    map->setFilename(filename.str());

    //osg::notify(osg::NOTICE) << "MapFilename is " << map->getFilename() << std::endl;

    //Create the TileBuilder
    //TODO: is this a memory leak??
    TileBuilder* tileBuilder = TileBuilder::create(map);
    if ( !tileBuilder || !tileBuilder->isOK() )
        return 0;

    return tileBuilder->createRootNode();
}

unsigned int
TileBuilder::getId() const
{
    return id;
}

const Profile*
TileBuilder::getMapProfile() const
{
    return _mapProfile.get();
}

// locates a tile source by its name
TileSource*
TileBuilder::findTileSource( const std::string& name ) const
{
    for( TileSourceList::const_iterator i = _image_sources.begin(); i != _image_sources.end(); i++ )
        if ( (*i)->getName() == name )
            return i->get();
    for( TileSourceList::const_iterator i = _heightfield_sources.begin(); i != _heightfield_sources.end(); i++ )
        if ( (*i)->getName() == name )
            return i->get();
    return 0L;
}

static const Profile*
getSuitableMapProfileFor( const Profile* candidate )
{
    if ( candidate->getProfileType() == Profile::TYPE_GEODETIC )
        return osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    else if ( candidate->getProfileType() == Profile::TYPE_MERCATOR )
        return osgEarth::Registry::instance()->getGlobalMercatorProfile();
    else
        return candidate;
}

// figures out what the map profile should be. there are multiple ways of setting it.
// In order of priority:
//
//   1. Use an explicit "named" profile (e.g., "global-geodetic")
//   2. Use the profile of one of the TileSources
//   3. Use an explicitly defined profile
//   4. Scan the TileSources and use the first profile found
//
// Once we locate the profile to use, set the MAP profile accordingly. If the map profile
// is not LOCAL/PROJECTED, it must be one of the NAMED profiles (global-geodetic/mercator).
// This is done so that caches are stored consistently.
//
void
TileBuilder::initializeTileSources()
{
    TileSource* ref_source = NULL;

    //Geocentric maps are always going to be rendered in the global-geodetic profile
    if (_map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC)
    {
        _mapProfile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        osg::notify(osg::INFO) << "[osgEarth] Setting Profile to global-geodetic for geocentric scene" << std::endl;
    }

    // First check for an explicit profile declaration:
    if ( !_mapProfile.valid() && _map->getProfileConfig() )
    {
        // Check for a "well known named" profile:
        std::string namedProfile = _map->getProfileConfig()->getNamedProfile();
        if ( !namedProfile.empty() )
        {
            _mapProfile = osgEarth::Registry::instance()->getNamedProfile( namedProfile );
            if ( _mapProfile.valid() )
            {
                osg::notify(osg::INFO) << "[osgEarth] Set map profile to " << namedProfile << std::endl;
            }
            else
            {
                osg::notify(osg::WARN) << "[osgEarth] " << namedProfile << " is not a known profile name" << std::endl;
                //TODO: continue on? or fail here?
            }
        }

        // Check for a TileSource reference (i.e. get the map profile from a particular TileSource)
        if ( !_mapProfile.valid() )
        {
            std::string refLayer = _map->getProfileConfig()->getRefLayer();
            if ( !refLayer.empty() )
            {
                ref_source = findTileSource( refLayer );
                if ( ref_source )
                {
                    const Profile* ref_profile = ref_source->initProfile( NULL, _map->getFilename() );
                    if ( ref_profile )
                    {
                        _mapProfile = getSuitableMapProfileFor( ref_profile );
                        osg::notify(osg::INFO) << "[osgEarth] Setting profile from \"" << refLayer << "\"" << std::endl;
                    }
                }
                else
                {
                    osg::notify(osg::WARN) << "[osgEarth] Source \"" << refLayer << "\" does not have a valid profile" << std::endl;
                }
            }
        }

        // Try to create a profile from an explicit definition (the SRS and extents)
        if ( !_mapProfile.valid() )
        {
            if ( _map->getProfileConfig()->areExtentsValid() )
            {
                double minx, miny, maxx, maxy;
                _map->getProfileConfig()->getExtents( minx, miny, maxx, maxy );

                // TODO: should we restrict this? This is fine for LOCAL/PROJECTED, but since we are not
                // constraining non-local map profiles to the "well known" types, should we let the user
                // override that? probably...
                _mapProfile = Profile::create( _map->getProfileConfig()->getSRS(), minx, miny, maxx, maxy );

                if ( _mapProfile.valid() )
                {
                    osg::notify( osg::INFO ) << "[osgEarth::TileBuilder] Set map profile from SRS: " 
                        << _mapProfile->getSRS()->getName() << std::endl;
                }
            }
        }
    }

    // At this point we MIGHT have a profile.

    // Finally, try scanning the loaded sources and taking the first one we get. At the
    // same time, remove any incompatible sources.

    for( TileSourceList::iterator i = _image_sources.begin(); i != _image_sources.end(); )
    {
        // skip the reference source since we already initialized it
        if ( i->get() != ref_source )
        {
            osg::ref_ptr<const Profile> sourceProfile = (*i)->initProfile( _mapProfile.get(), _map->getFilename() );

            if ( !_mapProfile.valid() && sourceProfile.valid() )
            {
                _mapProfile = getSuitableMapProfileFor( sourceProfile.get() );
            }
            else if ( !sourceProfile.valid() || !_mapProfile->isCompatibleWith( sourceProfile ) )
            {
                osg::notify(osg::WARN) << "[osgEarth] Removing incompatible TileSource " << i->get()->getName() << std::endl;
                i =_image_sources.erase(i);
                continue;
            }
        }

        if ( osg::getNotifyLevel() >= osg::INFO )
        {
            std::string prof_str = i->get()->getProfile()? i->get()->getProfile()->toString() : "none";
            osg::notify(osg::INFO)
                << "Tile source \"" << i->get()->getName() << "\" : profile = " << prof_str << std::endl;
        }
        
        i++;
    }

    //Create the elevation manager
    if (_heightfield_sources.size() > 0)
    {
        _elevationManager = new ElevationManager;
    }

    for (TileSourceList::iterator i = _heightfield_sources.begin(); i != _heightfield_sources.end(); )
    {        
        if ( i->get() != ref_source )
        {
            osg::ref_ptr<const Profile> sourceProfile = (*i)->initProfile( _mapProfile.get(), _map->getFilename() );

            if ( !_mapProfile.valid() && sourceProfile.valid() )
            {
                _mapProfile = getSuitableMapProfileFor( sourceProfile );
            }
            else if ( !sourceProfile.valid() || !_mapProfile->isCompatibleWith( sourceProfile ) )
            {
                osg::notify(osg::WARN) << "[osgEarth] Removing incompatible TileSource " << i->get()->getName() << std::endl;
                i = _heightfield_sources.erase(i);
                continue;
            }
        }

        _elevationManager->getElevationSources().push_back(i->get());

        if ( osg::getNotifyLevel() >= osg::INFO )
        {
            std::string prof_str = i->get()->getProfile()? i->get()->getProfile()->toString() : "none";
            osg::notify(osg::INFO)
                << "Tile source \"" << i->get()->getName() << "\" : profile = " << prof_str << std::endl;
        }
        
        i++;
    }
}

TileSource*
TileBuilder::loadSource(const MapConfig* mapConfig,
                        const SourceConfig* source,
                        const osgDB::ReaderWriter::Options* global_options)
{
    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ?
        new osgDB::ReaderWriter::Options( *global_options ) : 
        new osgDB::ReaderWriter::Options();

    //Setup the plugin options for the source
    for( SourceProperties::const_iterator p = source->getProperties().begin(); p != source->getProperties().end(); p++ )
    {
        local_options->setPluginData( p->first, (void*)p->second.c_str() );
    }

    bool foundValidSource = false;
    osg::ref_ptr<TileSource> tile_source;

    //Only load the source if we are not running offline
    if (!mapConfig->getCacheOnly())
    {
        //Add the source to the list.  The "." prefix causes OSG to select the correct plugin.
        //For instance, the WMS plugin can be loaded by using ".osgearth_wms" as the filename
        tile_source = dynamic_cast<TileSource*>(osgDB::readObjectFile(".osgearth_" + source->getDriver(), local_options.get()));
        if (!tile_source.valid())
        {
          osg::notify(osg::NOTICE) << "Warning:  Could not load TileSource from "  << source->getDriver() << std::endl;
        }
    }

    if (tile_source.valid())
    {           
        //Initialize the source and set its name
        //tile_source->init(local_options.get());
        tile_source->setName( source->getName() );
        osg::notify(osg::INFO) << "Loaded " << source->getDriver() << " TileSource" << std::endl;
    }

    //Configure the cache if necessary
    osg::ref_ptr<const CacheConfig> cacheConfig = source->getCacheConfig();

    osg::ref_ptr<TileSource> topSource = tile_source.get();

    //If the cache config is valid, wrap the TileSource with a caching TileSource.
    if (cacheConfig.valid())
    {
        osg::ref_ptr<CachedTileSource> cache = CachedTileSourceFactory::create(
            tile_source.get(),
            cacheConfig->getType(),
            cacheConfig->getProperties(),
            local_options.get() );

        if (cache.valid())
        {
            //cache->init(local_options.get());
            cache->setName(source->getName());
            cache->setMapConfigFilename( mapConfig->getFilename() );
            //cache->initTileMap(); //gw: moved to the TileSource::createProfile() method...
            topSource = cache.get();
        }
    }

    MemCachedTileSource* memCachedSource = new MemCachedTileSource(topSource.get(), local_options.get());
    memCachedSource->setName( topSource->getName() );

    // Finally, install an override profile if the caller requested one. This will override the profile
    // that the TileSource reports.
    if ( source->getProfileConfig() )
    {
        osg::ref_ptr<const Profile> override_profile =
            osgEarth::Registry::instance()->getNamedProfile( source->getProfileConfig()->getSRS() );

        if ( !override_profile.valid() )
        {
            double xmin, ymin, xmax, ymax;
            source->getProfileConfig()->getExtents( xmin, ymin, xmax, ymax );
            override_profile = Profile::create( source->getProfileConfig()->getSRS(), xmin, ymin, xmax, ymax );
        }

        if ( override_profile.valid() )
        {
            memCachedSource->setOverrideProfile( override_profile.get() );
        }
    }

    return memCachedSource;
}


void
TileBuilder::addSources(const MapConfig* mapConfig, const SourceConfigList& from, 
                        std::vector< osg::ref_ptr<TileSource> >& to,
                        const osgDB::ReaderWriter::Options* global_options)
{        
    for( SourceConfigList::const_iterator i = from.begin(); i != from.end(); i++ )
    {
        SourceConfig* source = i->get();

        osg::ref_ptr<TileSource> tileSource = loadSource(mapConfig, source, global_options);

        if ( tileSource.valid() )
        {
            to.push_back( tileSource.get() );
        }
    }
}

TileBuilder::TileBuilder(MapConfig* map, 
                         const osgDB::ReaderWriter::Options* options ) :
_map( map )
{
    if ( !map )
        return;

    addSources( _map.get(), _map->getImageSources(), _image_sources, options );
    addSources( _map.get(), _map->getHeightFieldSources(), _heightfield_sources, options );

    initializeTileSources();

    //Set the MapConfig's Profile to the computed profile so that the TileSource's can query it when they are loaded
    _map->setProfile( _mapProfile.get() );
}

std::string
TileBuilder::createURI( const TileKey* key )
{
    std::stringstream ss;
    ss << key->str() << "." <<id<<".earth_tile";
    return ss.str();
}

MapConfig*
TileBuilder::getMapConfig() const
{
    return _map.get();
}


// Make a MatrixTransform suitable for use with a Locator object based on the given extents.
// Calling Locator::setTransformAsExtents doesn't work with OSG 2.6 due to the fact that the
// _inverse member isn't updated properly.  Calling Locator::setTransform works correctly.
osg::Matrixd
TileBuilder::getTransformFromExtents(double minX, double minY, double maxX, double maxY) const
{
    osg::Matrixd transform;
    transform.set(
        maxX-minX, 0.0,       0.0, 0.0,
        0.0,       maxY-minY, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        minX,      minY,      0.0, 1.0); 
    return transform;
}


bool
TileBuilder::isOK() const
{
    if ( !_mapProfile.valid() )
    {
        osg::notify(osg::NOTICE) << "Error: Unable to determine a map profile." << std::endl;
        return false;
    }

    if (getImageSources().size() == 0 && getHeightFieldSources().size() == 0)
    {
        osg::notify(osg::NOTICE) << "Error: TileBuilder does not contain any image or heightfield sources." << std::endl;
        return false;
    }

    //Check to see if we are trying to do a Geocentric database with a Projected profile.
    if ( _mapProfile->getProfileType() == Profile::TYPE_LOCAL && 
         _map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC)
    {
        osg::notify(osg::NOTICE) << "Error: Cannot create a geocentric scene using projected datasources.  Please specify type=\"flat\" on the map element in the .earth file." << std::endl;
        return false;
    }

    //TODO: Other cases?
    return true;
}

osg::Node*
TileBuilder::createRootNode()
{
    // Note: CSN must always be at the top
    osg::ref_ptr<osg::CoordinateSystemNode> csn = createCoordinateSystemNode();

    //If there is more than one image source, use TexEnvCombine to blend them together
    if ( _map->getImageSources().size() > 1 )
    {
#if 1
        osg::StateSet* stateset = csn->getOrCreateStateSet();
        for (unsigned int i = 0; i < _map->getImageSources().size(); ++i)
        {    
            //Blend the textures together from the bottom up
            stateset->setTextureMode(i, GL_TEXTURE_2D, osg::StateAttribute::ON);

            //Interpolate the current texture with the previous combiner result using the textures SRC_ALPHA
            osg::TexEnvCombine * tec = new osg::TexEnvCombine;
            tec->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);

            tec->setSource0_RGB(osg::TexEnvCombine::TEXTURE);
            tec->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);

            tec->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
            tec->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);

            tec->setSource2_RGB(osg::TexEnvCombine::TEXTURE);
            tec->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

            stateset->setTextureAttribute(i, tec, osg::StateAttribute::ON);
        }

        //Modulate the result with the primary color to get proper lighting
        osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
        texenv->setCombine_RGB(osg::TexEnvCombine::MODULATE);
        texenv->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
        texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
        texenv->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
        texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
        stateset->setTextureAttribute(_map->getImageSources().size(), texenv, osg::StateAttribute::ON);
        stateset->setTextureMode(_map->getImageSources().size(), GL_TEXTURE_2D, osg::StateAttribute::ON);
#else
        //Decorate the scene with a multi-texture control to control blending between textures
        osgFX::MultiTextureControl *mt = new osgFX::MultiTextureControl;
        parent->addChild( mt );

        float r = 1.0f/ map->getImageSources().size();
        for (unsigned int i = 0; i < map->getImageSources().size(); ++i)
        {
            mt->setTextureWeight(i, r);
        }
        parent = mt;
#endif
    }

    _terrain = new osgEarth::EarthTerrain;//new osgTerrain::Terrain();
    _terrain->setVerticalScale( _map->getVerticalScale() );
    _terrain->setSampleRatio( _map->getSampleRatio() );
    csn->addChild( _terrain.get() );


    std::vector< osg::ref_ptr<TileKey> > keys;
    getMapProfile()->getRootKeys(keys);

    int numAdded = 0;
    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        osg::Node* node = createNode( keys[i].get() );
        if (node)
        {
            _terrain->addChild(node);
            numAdded++;
        }
        else
        {
            osg::notify(osg::NOTICE) << "Couldn't get tile for " << keys[i]->str() << std::endl;
        }
    }

    return (numAdded == keys.size()) ? csn.release() : NULL;
}



osg::Node*
TileBuilder::createNode( const TileKey* key )
{
    osg::ref_ptr<osg::Group> parent = new osg::Group;
    if (!addChildren( parent.get(), key ))
    {
        parent = 0;
    }
    return parent.release();
}

/*osg::HeightField*
TileBuilder::createValidHeightField(osgEarth::TileSource* tileSource, const osgEarth::TileKey *key)
{
    //Try to create the heightfield with the given key
    osg::ref_ptr<osg::HeightField> hf;
    osg::ref_ptr<const TileKey> hf_key = key;

    if (!hf.valid())
    {
        while (hf_key.valid())
        {
            if (tileSource->isKeyValid(hf_key.get()))
            {
              hf = createHeightField(hf_key.get(), tileSource);
            }
            if (hf.valid()) break;
            hf_key = hf_key->createParentKey();
        }

        if (hf_key.valid() && hf.valid())
        {
          double minx, miny, maxx, maxy;
          hf_key->getGeoExtent().getBounds(minx, miny, maxx, maxy);

          //Need to init this before extracting the heightfield
          hf->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
          hf->setXInterval( (maxx - minx)/(double)(hf->getNumColumns()-1) );
          hf->setYInterval( (maxy - miny)/(double)(hf->getNumRows()-1) );

          double key_minx, key_miny, key_maxx, key_maxy;
          key->getGeoExtent().getBounds(key_minx, key_miny, key_maxx, key_maxy);
          hf = HeightFieldUtils::extractHeightField(hf.get(), key_minx, key_miny, key_maxx, key_maxy, hf->getNumColumns(), hf->getNumRows());
        }
    }

    return hf.release();
}*/

bool
TileBuilder::createValidImage(osgEarth::TileSource* tileSource,
                              const osgEarth::TileKey *key,
                              osgEarth::TileBuilder::ImageTileKeyPair &imageTile)
{
    //Try to create the image with the given key
    osg::ref_ptr<const TileKey> image_key = key;

    osg::ref_ptr<GeoImage> geo_image;

    while (image_key.valid())
    {
        if ( tileSource->isKeyValid(image_key.get()) )
        {
            geo_image = createGeoImage( image_key.get(), tileSource );
            //image = createImage(image_key.get(), tileSource);
        }
        if (geo_image.valid()) break;
        image_key = image_key->createParentKey();
    }

    if (image_key.valid() && geo_image.valid())
    {
        imageTile.first = geo_image.get();
        imageTile.second = image_key.get();
        return true;
    }
    return false;
}

TileSourceList&
TileBuilder::getImageSources()
{
    return _image_sources;
}

const TileSourceList&
TileBuilder::getImageSources() const
{
    return _image_sources;
}

TileSourceList&
TileBuilder::getHeightFieldSources()
{
    return _heightfield_sources;
}

const TileSourceList&
TileBuilder::getHeightFieldSources() const
{
    return _heightfield_sources;
}

bool
TileBuilder::hasMoreLevels( const TileKey* key ) const
{
    bool more_levels = false;
    int max_level = 0;

    for( TileSourceList::const_iterator i = getImageSources().begin(); i != getImageSources().end(); i++ )
    {
        if ( key->getLevelOfDetail() < i->get()->getMaxLevel() )
        {
            more_levels = true;
            break;
        }
    }
    if ( !more_levels )
    {
        for( TileSourceList::const_iterator j = getHeightFieldSources().begin(); j != getHeightFieldSources().end(); j++ )
        {
            if ( key->getLevelOfDetail() < j->get()->getMaxLevel() )
            {
                more_levels = true;
                break;
            }
        }
    }

    return more_levels;
}

bool
TileBuilder::addChildren( osg::Group* tile_parent, const TileKey* key )
{
    bool all_quadrants_created = false;

    osg::ref_ptr<osg::Node> q0, q1, q2, q3;

    q0 = createQuadrant( key->getSubkey(0) );
    q1 = createQuadrant( key->getSubkey(1) );
    q2 = createQuadrant( key->getSubkey(2) );
    q3 = createQuadrant( key->getSubkey(3) );

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
        osg::notify(osg::INFO) << "Couldn't create all quadrants for " << key->str() << " time to stop subdividing!" << std::endl;
    }
    return all_quadrants_created;
}


GeoImage*
TileBuilder::createGeoImage(const TileKey* mapKey, TileSource* source)
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
            if ( ! mosaic->getSRS()->isEquivalentTo( mapKey->getProfile()->getSRS() ) )
            {
                // TODO: reproject the mosaic (unless we're doing the special mercator-locator thing,
                // which we'll need a setting to enable/disable).
            }

            // crop to fit the map key extents. (NOTE: if we reproject the geo_image, there will be no need to
            // actually reproject the extents below)

            GeoExtent clampedMapExt =
                source->getProfile()->clampAndTransformExtent( mapKey->getGeoExtent() );

            result = mosaic->crop( clampedMapExt.xMin(), clampedMapExt.yMin(), clampedMapExt.xMax(), clampedMapExt.yMax() );

            //result = mosaic.release();
        }
    }

    return result;
}
