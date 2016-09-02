/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "MPTerrainEngineNode"
#include "SingleKeyNodeFactory"
#include "TerrainNode"
#include "TileModelFactory"
#include "TileModelCompiler"
#include "TilePagedLOD"
#include "MPShaders"

#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderFactory>
#include <osgEarth/MapModelChange>
#include <osgEarth/Progress>
#include <osgEarth/ShaderLoader>
#include <osgEarth/Utils>
#include <osgEarth/ObjectIndex>

#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/PagedLOD>
#include <osg/Timer>
#include <osg/Depth>
#include <osg/BlendFunc>
#include <osgDB/DatabasePager>
#include <osgUtil/RenderBin>
#include <osgUtil/RenderLeaf>

#define LC "[MPTerrainEngineNode] "

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;


// TODO: bins don't work with SSDK. No idea why. Disable until further notice.
//#define USE_RENDER_BINS 1

//------------------------------------------------------------------------

namespace
{
    // adapter that lets MPTerrainEngineNode listen to Map events
    struct MPTerrainEngineNodeMapCallbackProxy : public MapCallback
    {
        MPTerrainEngineNodeMapCallbackProxy(MPTerrainEngineNode* node) : _node(node) { }
        osg::observer_ptr<MPTerrainEngineNode> _node;

        void onMapInfoEstablished( const MapInfo& mapInfo ) {
            osg::ref_ptr<MPTerrainEngineNode> node;
            if ( _node.lock(node) )
                node->onMapInfoEstablished( mapInfo );
        }

        void onMapModelChanged( const MapModelChange& change ) {
            osg::ref_ptr<MPTerrainEngineNode> node;
            if ( _node.lock(node) )
                node->onMapModelChanged( change );
        }
    };


    // Render bin for terrain surface geometry
    class TerrainBin : public osgUtil::RenderBin
    {
    public:
        TerrainBin()
        {
            this->setStateSet( new osg::StateSet() );
            this->setSortMode(SORT_FRONT_TO_BACK);
        }

        osg::Object* clone(const osg::CopyOp& copyop) const
        {
            return new TerrainBin(*this, copyop);
        }

        TerrainBin(const TerrainBin& rhs, const osg::CopyOp& copy) :
            osgUtil::RenderBin(rhs, copy)
        {
        }
    };


    // Render bin for terrain payload geometry
    class PayloadBin : public osgUtil::RenderBin
    {
    public:
        PayloadBin()
        {
            this->setStateSet( new osg::StateSet() );
        }

        osg::Object* clone(const osg::CopyOp& copyop) const
        {
            return new PayloadBin(*this, copyop);
        }

        PayloadBin(const PayloadBin& rhs, const osg::CopyOp& copy) :
            osgUtil::RenderBin(rhs, copy)
        {
        }
    };

#if 0
    class NormalTexInstaller : public TerrainEngine::NodeCallback
    {
    public:
        NormalTexInstaller(int unit) : _unit(unit) { }
        
    public: // TileNodeCallback
        void operator()(const TileKey& key, osg::Node* node)
        {
            TileNode* tile = osgEarth::findTopMostNodeOfType<TileNode>(node);
            if ( !tile )
            {
                OE_WARN << LC << "No tile " << key.str() << "\n";
                return;
            }

            if ( !tile->getTileModel() )
            {
                OE_WARN << LC << "No tile model available for " << key.str() << "\n";
                return;
            }
            
            osg::StateSet* ss = node->getOrCreateStateSet();
            osg::Texture* tex = tile->getTileModel()->getNormalTexture();
            if ( tex )
            {
                ss->setTextureAttribute(_unit, tex);
            }

            osg::RefMatrixf* mat = tile->getModel()->getNormalTextureMatrix();
            osg::Matrixf fmat;
            if ( mat )
            {
                fmat = osg::Matrixf(*mat);
            }
            else
            {
                // special marker indicating that there's no valid normal texture.
                fmat(0,0) = 0.0f;
            }

            ss->addUniform(new osg::Uniform("oe_tile_normalTexMatrix", fmat) );
        }

    private:
        int _unit;
    };
#endif
}

//---------------------------------------------------------------------------

static Threading::ReadWriteMutex s_engineNodeCacheMutex;
//Caches the MapNodes that have been created
typedef std::map<UID, osg::observer_ptr<MPTerrainEngineNode> > EngineNodeCache;

static
EngineNodeCache& getEngineNodeCache()
{
    static EngineNodeCache s_cache;
    return s_cache;
}

void
MPTerrainEngineNode::registerEngine(MPTerrainEngineNode* engineNode)
{
    Threading::ScopedWriteLock exclusiveLock( s_engineNodeCacheMutex );
    getEngineNodeCache()[engineNode->_uid] = engineNode;
    OE_DEBUG << LC << "Registered engine " << engineNode->_uid << std::endl;
}

// since this method is called in a database pager thread, we use a ref_ptr output
// parameter to avoid the engine node being destructed between the time we 
// return it and the time it's accessed; this could happen if the user removed the
// MapNode from the scene during paging.
void
MPTerrainEngineNode::getEngineByUID( UID uid, osg::ref_ptr<MPTerrainEngineNode>& output )
{
    Threading::ScopedReadLock sharedLock( s_engineNodeCacheMutex );
    EngineNodeCache::const_iterator k = getEngineNodeCache().find( uid );
    if (k != getEngineNodeCache().end())
        output = k->second.get();
}

UID
MPTerrainEngineNode::getUID() const
{
    return _uid;
}

//------------------------------------------------------------------------

MPTerrainEngineNode::ElevationChangedCallback::ElevationChangedCallback( MPTerrainEngineNode* terrain ):
_terrain( terrain )
{
    //nop
}

void
MPTerrainEngineNode::ElevationChangedCallback::onVisibleChanged( TerrainLayer* layer )
{
    _terrain->refresh(true); // true => force a dirty
}

//------------------------------------------------------------------------

MPTerrainEngineNode::MPTerrainEngineNode() :
TerrainEngineNode     ( ),
_terrain              ( 0L ),
_update_mapf          ( 0L ),
_tileCount            ( 0 ),
_tileCreationTime     ( 0.0 ),
_primaryUnit          ( -1 ),
_secondaryUnit        ( -1 ),
_elevationTextureUnit ( -1 ),
_batchUpdateInProgress( false ),
_refreshRequired      ( false ),
_stateUpdateRequired  ( false )
{
    // unique ID for this engine:
    _uid = Registry::instance()->createUID();

#ifdef USE_RENDER_BINS
    // Register our render bins protos.
    {
        // Mutex because addRenderBinPrototype isn't thread-safe.
        Threading::ScopedMutexLock lock(_renderBinMutex);

        // generate uniquely named render bin prototypes for this engine:
        _terrainRenderBinPrototype = new TerrainBin();
        _terrainRenderBinPrototype->setName( Stringify() << "oe.TerrainBin." << _uid );
        osgUtil::RenderBin::addRenderBinPrototype( _terrainRenderBinPrototype->getName(), _terrainRenderBinPrototype.get() );

        _payloadRenderBinPrototype = new PayloadBin();
        _payloadRenderBinPrototype->setName( Stringify() << "oe.PayloadBin." << _uid );
        osgUtil::RenderBin::addRenderBinPrototype( _payloadRenderBinPrototype->getName(), _payloadRenderBinPrototype.get() );
    }
#endif

    // install an elevation callback so we can update elevation data
    _elevationCallback = new ElevationChangedCallback( this );

    // install the static shader components now.
    if ( osgEarth::Registry::capabilities().supportsGLSL() )
    {
        osg::StateSet* stateset = getOrCreateStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        includeShaderLibrary( vp );
    }
}

MPTerrainEngineNode::~MPTerrainEngineNode()
{
#ifdef USE_RENDER_BINS
    osgUtil::RenderBin::removeRenderBinPrototype( _terrainRenderBinPrototype.get() );
    osgUtil::RenderBin::removeRenderBinPrototype( _payloadRenderBinPrototype.get() );
#endif

    if ( _update_mapf )
    {
        delete _update_mapf;
    }
}

bool
MPTerrainEngineNode::includeShaderLibrary(VirtualProgram* vp)
{
    static const char* libVS =
        "#version 330\n"
        "#pragma vp_name MP Terrain SDK (VS)\n"

        "in vec4 oe_terrain_attr; \n"
        "uniform vec4 oe_tile_key; \n"
        "vec3 vp_Normal; \n"

        "float oe_terrain_getElevation(in vec2 uv) \n"
        "{ \n"
        "    return oe_terrain_attr[3]; \n"
        "} \n"

        "float oe_terrain_getElevation() \n"
        "{ \n"
        "    return oe_terrain_attr[3]; \n"
        "} \n"

        "vec4 oe_terrain_getNormalAndCurvature(in vec2 uv) \n"
        "{ \n"
        "    return vec4(vp_Normal, 0.0); \n"
        "} \n"

        "vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 uv, in float refLOD) \n"
        "{ \n"
        "    float dL = oe_tile_key.z - refLOD; \n"
        "    float factor = exp2(dL); \n"
        "    float invFactor = 1.0/factor; \n"
        "    vec2 scale = vec2(invFactor); \n"
        "    vec2 result = uv * scale; \n"
        "    if ( factor >= 1.0 ) \n"
        "    { \n"
        "        vec2 a = floor(oe_tile_key.xy * invFactor); \n"
        "        vec2 b = a * factor; \n"
        "        vec2 c = (a+1.0) * factor; \n"
        "        vec2 offset = (oe_tile_key.xy-b)/(c-b); \n"
        "        result += offset; \n"
        "    } \n"
        "    return result; \n"
        "} \n";

    static const char* libFS =
        "#version 330\n"
        "#pragma vp_name MP Terrain SDK (FS)\n"

        "uniform vec4 oe_tile_key; \n"
        "vec3 vp_Normal; \n"
        "in float oe_mp_terrainElev; // internal variable \n"

        "float oe_terrain_getElevation(in vec2 uv) \n"
        "{ \n"
        "    return oe_mp_terrainElev; \n"
        "} \n"

        "float oe_terrain_getElevation() \n"
        "{ \n"
        "    return oe_mp_terrainElev; \n"
        "} \n"

        "vec4 oe_terrain_getNormalAndCurvature(in vec2 uv) \n"
        "{ \n"
        "    return vec4(vp_Normal, 0.0); \n"
        "} \n"

        "vec4 oe_terrain_getNormalAndCurvature() \n"
        "{ \n"
        "    return vec4(vp_Normal, 0.0); \n"
        "} \n"

        "vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 uv, in float refLOD) \n"
        "{ \n"
        "    float dL = oe_tile_key.z - refLOD; \n"
        "    float factor = exp2(dL); \n"
        "    float invFactor = 1.0/factor; \n"
        "    vec2 scale = vec2(invFactor); \n"
        "    vec2 result = uv * scale; \n"
        "    if ( factor >= 1.0 ) \n"
        "    { \n"
        "        vec2 a = floor(oe_tile_key.xy * invFactor); \n"
        "        vec2 b = a * factor; \n"
        "        vec2 c = (a+1.0) * factor; \n"
        "        vec2 offset = (oe_tile_key.xy-b)/(c-b); \n"
        "        result += offset; \n"
        "    } \n"
        "    return result; \n"
        "} \n";

    if ( vp )
    {
        osg::Shader* VS = new osg::Shader(osg::Shader::VERTEX, libVS);
        VS->setName( "oe_terrain_SDK_mp_VS" );
        vp->setShader( VS );
        
        osg::Shader* FS = new osg::Shader(osg::Shader::FRAGMENT, libFS);
        FS->setName( "oe_terrain_SDK_mp_FS" );
        vp->setShader( FS );

        vp->addBindAttribLocation( "oe_terrain_attr",  osg::Drawable::ATTRIBUTE_6 );
        vp->addBindAttribLocation( "oe_terrain_attr2", osg::Drawable::ATTRIBUTE_7 );
    }

    return (vp != 0L);
}

void
MPTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );
    //nop.
}

void
MPTerrainEngineNode::postInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::postInitialize( map, options );

    // Initialize the map frames. We need one for the update thread and one for the
    // cull thread. Someday we can detect whether these are actually the same thread
    // (depends on the viewer's threading mode).
    _update_mapf = new MapFrame( map, Map::ENTIRE_MODEL );

    // merge in the custom options:
    _terrainOptions.merge( options );

    // A shared registry for tile nodes in the scene graph. Enable revision tracking
    // if requested in the options. Revision tracking lets the registry notify all
    // live tiles of the current map revision so they can inrementally update
    // themselves if necessary.
    _liveTiles = new TileNodeRegistry("live");
    _liveTiles->setRevisioningEnabled( _terrainOptions.incrementalUpdate() == true );
    _liveTiles->setMapRevision( _update_mapf->getRevision() );

    // Facility to properly release GL objects
    _releaser = new ResourceReleaser();
    this->addChild(_releaser.get());

    // reserve GPU resources. Must do this before initializing the model factory.
    if ( _primaryUnit < 0 )
    {
        getResources()->reserveTextureImageUnit( _primaryUnit, "MP Engine Primary" );
    }

    // "Secondary" unit serves double duty; it's used for parent textures BUT it's also
    // used at the "slot" for the tile coordinates.
    if ( _secondaryUnit < 0 )
    {
        getResources()->reserveTextureImageUnit( _secondaryUnit, "MP Engine Secondary" );
    }
    
    // initialize the model factory:
    _tileModelFactory = new TileModelFactory(_liveTiles.get(), _terrainOptions, this);

    // Normal map texture unit
    if ( _terrainOptions.normalMaps() == true )
    {
        this->_requireNormalTextures = true;
        getResources()->reserveTextureImageUnit( _normalMapUnit, "MP Normal Maps" );
        _tileModelFactory->setNormalMapUnit( _normalMapUnit );
    }

    // handle an already-established map profile:
    if ( _update_mapf->getProfile() )
    {
        // NOTE: this will initialize the map with the startup layers
        onMapInfoEstablished( MapInfo(map) );
    }

    // install a layer callback for processing further map actions:
    map->addMapCallback( new MPTerrainEngineNodeMapCallbackProxy(this) );

    // Prime with existing layers:
    _batchUpdateInProgress = true;

    ElevationLayerVector elevationLayers;
    map->getElevationLayers( elevationLayers );
    for( ElevationLayerVector::const_iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i )
        addElevationLayer( i->get() );

    ImageLayerVector imageLayers;
    map->getImageLayers( imageLayers );
    for( ImageLayerVector::iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
        addImageLayer( i->get() );

    _batchUpdateInProgress = false;

    // register this instance to the osgDB plugin can find it.
    registerEngine( this );

    // set up the initial shaders and reserve the texture image units.
    updateState();

    // now that we have a map, set up to recompute the bounds
    dirtyBound();

    OE_INFO << LC << "Edge normalization is " << (_terrainOptions.normalizeEdges() == true? "ON" : "OFF") << std::endl;
}


osg::BoundingSphere
MPTerrainEngineNode::computeBound() const
{
    //if ( _terrain && _terrain->getNumChildren() > 0 )
    //{
    //    return _terrain->getBound();
    //}
    //else
    {
        return TerrainEngineNode::computeBound();
    }
}

void
MPTerrainEngineNode::invalidateRegion(const GeoExtent& extent,
                                      unsigned         minLevel,
                                      unsigned         maxLevel)
{
    if (_terrainOptions.incrementalUpdate() == true && _liveTiles.valid())
    {
        GeoExtent extentLocal = extent;
        if ( !extent.getSRS()->isEquivalentTo(this->getMap()->getSRS()) )
        {
            extent.transform(this->getMap()->getSRS(), extentLocal);
        }
        
        _liveTiles->setDirty(extentLocal, minLevel, maxLevel);
    }
}

void
MPTerrainEngineNode::refresh(bool forceDirty)
{
    // if we're in the middle of a batch update OR if the terrain has not
    // been fully initialized for rendering, mark it for later.
    if ( _batchUpdateInProgress || _update_mapf == 0L )
    {
        _refreshRequired = true;
    }
    else
    {
        if ( _terrainOptions.incrementalUpdate() == true )
        {
            // run an atomic "dirty" operation:
            //_update_mapf->sync();
            //_liveTiles->setMapRevision( _update_mapf->getRevision(), forceDirty );
        }
        else
        {
            dirtyTerrain();
        }

        _refreshRequired = false;
    }
}

void
MPTerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    if ( _update_mapf != 0L )
    {
        dirtyTerrain();
    }
}

osg::StateSet*
MPTerrainEngineNode::getTerrainStateSet()
{
#ifdef USE_RENDER_BINS
    return _terrainRenderBinPrototype->getStateSet();
#else
    return _terrain ? _terrain->getOrCreateStateSet() : 0L;
#endif
}

namespace
{
    struct NotifyExistingNodesOp : public TileNodeRegistry::ConstOperation
    {
        TerrainEngine::NodeCallback* _cb;

        NotifyExistingNodesOp(TerrainEngine::NodeCallback* cb) : _cb(cb) { }

        void operator()(const TileNodeRegistry::TileNodeMap& tiles) const
        {
            for(TileNodeRegistry::TileNodeMap::const_iterator i = tiles.begin();
                i != tiles.end();
                ++i)
            {
                _cb->operator()(i->first, i->second.get());
            }
        }
    };
}

void
MPTerrainEngineNode::notifyExistingNodes(TerrainEngine::NodeCallback* cb)
{
    NotifyExistingNodesOp op( cb );
    _liveTiles->run( op );
}


osg::StateSet*
MPTerrainEngineNode::getPayloadStateSet()
{
    return _payloadRenderBinPrototype->getStateSet();
}

void
MPTerrainEngineNode::dirtyTerrain()
{
    // scrub the heightfield cache.
    if (_tileModelFactory)
    {
        _tileModelFactory->clearCaches();
    }

    // remove existing:
    if ( _terrain )
    {
        this->removeChild( _terrain );
    }

    // New terrain
    _terrain = new TerrainNode();

    // Clear out the tile registry:
    _liveTiles->releaseAll(_releaser.get());


#ifdef USE_RENDER_BINS
    _terrain->getOrCreateStateSet()->setRenderBinDetails( 0, _terrainRenderBinPrototype->getName() );
    _terrain->getOrCreateStateSet()->setNestRenderBins(false);
#else
    _terrain->getOrCreateStateSet()->setRenderBinDetails(0, "SORT_FRONT_TO_BACK");
#endif

    this->addChild( _terrain );
    // Build the first level of the terrain.
    // Collect the tile keys comprising the root tiles of the terrain.
    if ( _update_mapf )
    {
        // Factory to create the root keys:
        KeyNodeFactory* factory = getKeyNodeFactory();
    
        std::vector< TileKey > keys;
        _update_mapf->getProfile()->getAllKeysAtLOD( *_terrainOptions.firstLOD(), keys );

        // create a root node for each root tile key.
        OE_INFO << LC << "Creating " << keys.size() << " root keys.." << std::endl;

        TilePagedLOD* root = new TilePagedLOD( _uid, _liveTiles, _releaser.get() );
        root->setRangeFactor(_terrainOptions.minTileRangeFactor().get());
        _terrain->addChild( root );

        osg::ref_ptr<osgDB::Options> dbOptions = Registry::instance()->cloneOrCreateOptions();

        // Accumulate data from low to high resolution when necessary:
        bool accumulate = true;

        unsigned child = 0;
        for( unsigned i=0; i<keys.size(); ++i )
        {
            osg::ref_ptr<osg::Node> node = factory->createNode( keys[i], accumulate, true, 0L );
            if ( node.valid() )
            {
                root->addChild( node.get() );
                root->setRange( child++, 0.0f, FLT_MAX );
                root->setCenter( node->getBound().center() );
                root->setNumChildrenThatCannotBeExpired( child );
            }
            else
            {
                OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
            }
        }
    }

    updateState();

    // Call the base class
    TerrainEngineNode::dirtyTerrain();
}

namespace
{
    // debugging
    struct CheckForOrphans : public TileNodeRegistry::ConstOperation {
        void operator()( const TileNodeRegistry::TileNodeMap& tiles ) const {
            unsigned count = 0;
            for(TileNodeRegistry::TileNodeMap::const_iterator i = tiles.begin(); i != tiles.end(); ++i ) {
                if ( i->second->referenceCount() == 1 ) {
                    count++;
                }
            }
            if ( count > 0 )
                OE_DEBUG << LC << "Oh no! " << count << " orphaned tiles in the reg" << std::endl;
        }
    };
}


void
MPTerrainEngineNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // Inform the registry of the current frame so that Tiles have access
        // to the information.
        if ( _liveTiles.valid() && nv.getFrameStamp() )
        {
            _liveTiles->setTraversalFrame( nv.getFrameStamp()->getFrameNumber() );
        }
    }

#if 0
    static int c = 0;
    if ( ++c % 60 == 0 )
    {
        //OE_NOTICE << LC << "Live = " << _liveTiles->size() << ", Dead = " << _deadTiles->size() << std::endl;
        _liveTiles->run( CheckForOrphans() );
        Registry::instance()->startActivity("MP live tiles", Stringify() << _liveTiles->size());
    }
#endif

    TerrainEngineNode::traverse( nv );
}


KeyNodeFactory*
MPTerrainEngineNode::getKeyNodeFactory()
{
    osg::ref_ptr<KeyNodeFactory>& knf = _perThreadKeyNodeFactories.get(); // thread-safe get
    if ( !knf.valid() )
    {
        // create a compiler for compiling tile models into geometry
        bool optimizeTriangleOrientation = 
            getMap()->getMapOptions().elevationInterpolation() != INTERP_TRIANGULATE;

        // A compiler specific to this thread:
        TileModelCompiler* compiler = new TileModelCompiler(
            _update_mapf->terrainMaskLayers(),
            _update_mapf->modelLayers(),
            _primaryUnit,
            optimizeTriangleOrientation,
            _terrainOptions );

        // initialize a key node factory.
        knf = new SingleKeyNodeFactory(
            getMap(),
            _tileModelFactory.get(),
            compiler,
            _liveTiles.get(),
            _releaser.get(),
            _terrainOptions,
            this );
    }

    return knf.get();
}

osg::Node*
MPTerrainEngineNode::createNode(const TileKey&    key,
                                ProgressCallback* progress)
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 )
        return 0L;

    OE_DEBUG << LC << "Create node for \"" << key.str() << "\"" << std::endl;

    bool accumulate    = true;  // use parent data to help build tiles if neccesary
    bool setupChildren = true;  // prepare the tile for subdivision

    // create the node:
    osg::ref_ptr<osg::Node> node = getKeyNodeFactory()->createNode(key, accumulate, setupChildren, progress);

    // release the reference and return it.
    return node.release();
}

osg::Node*
MPTerrainEngineNode::createStandaloneNode(const TileKey&    key,
                                          ProgressCallback* progress)
{
    // if the engine has been disconnected from the scene graph, bail out and don't
    // create any more tiles
    if ( getNumParents() == 0 )
        return 0L;

    OE_DEBUG << LC << "Create standalone node for \"" << key.str() << "\"" << std::endl;

    return getKeyNodeFactory()->createNode( key, true, false, progress );
}

osg::Node*
MPTerrainEngineNode::createTile( const TileKey& key )
{
    osg::ref_ptr<TileModel> model = new TileModel( _update_mapf->getRevision(), _update_mapf->getMapInfo() );
    model->_tileKey = key;
    model->_tileLocator = GeoLocator::createForKey(key, _update_mapf->getMapInfo());

    // Build the heightfield

    const MapInfo& mapInfo = _update_mapf->getMapInfo();

    const osgEarth::ElevationInterpolation& interp = _update_mapf->getMapOptions().elevationInterpolation().get();

    // Request a heightfield from the map, falling back on lower resolution tiles
    int tileSize = _terrainOptions.tileSize().get();    
    osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), tileSize, tileSize );

    TileKey sampleKey = key;
    bool populated = false;
    if (_update_mapf->elevationLayers().size() > 0)
    {
        while (!populated)
        {
            populated = _update_mapf->populateHeightField(hf, sampleKey, true, 0L);
            if (!populated)
            {
                // Fallback on the parent
                sampleKey = sampleKey.createParentKey();
                if (!sampleKey.valid())
                {
                    return 0;
                }
            }
        }       
    }

    if (!populated)
    {
        // We have no heightfield so just create a reference heightfield.
        int tileSize = _terrainOptions.tileSize().get();
        hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), tileSize, tileSize );
        sampleKey = key;
    }

    model->_elevationData = TileModel::ElevationData(
        hf,
        GeoLocator::createForKey( sampleKey, mapInfo ),
        false );        

    bool optimizeTriangleOrientation = getMap()->getMapOptions().elevationInterpolation() != INTERP_TRIANGULATE;

    osg::ref_ptr<TileModelCompiler> compiler = new TileModelCompiler(
        _update_mapf->terrainMaskLayers(),
        _update_mapf->modelLayers(),
        _primaryUnit,
        optimizeTriangleOrientation,
        _terrainOptions );

    return compiler->compile(model.get(), *_update_mapf, 0L);
}


void
MPTerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    if ( change.getAction() == MapModelChange::BEGIN_BATCH_UPDATE )
    {
        _batchUpdateInProgress = true;
    }

    else if ( change.getAction() == MapModelChange::END_BATCH_UPDATE )
    {
        _batchUpdateInProgress = false;

        if ( _refreshRequired )
            refresh();

        if ( _stateUpdateRequired )
            updateState();
    }

    else
    {
        // update the thread-safe map model copy:
        if ( _update_mapf->sync() )
        {
            _liveTiles->setMapRevision( _update_mapf->getRevision() );
        }

        // dispatch the change handler
        if ( change.getLayer() )
        {
            // then apply the actual change:
            switch( change.getAction() )
            {
            case MapModelChange::ADD_IMAGE_LAYER:
                addImageLayer( change.getImageLayer() );
                break;
            case MapModelChange::REMOVE_IMAGE_LAYER:
                removeImageLayer( change.getImageLayer() );
                break;
            case MapModelChange::ADD_ELEVATION_LAYER:
                addElevationLayer( change.getElevationLayer() );
                break;
            case MapModelChange::REMOVE_ELEVATION_LAYER:
                removeElevationLayer( change.getElevationLayer() );
                break;
            case MapModelChange::MOVE_IMAGE_LAYER:
                moveImageLayer( change.getFirstIndex(), change.getSecondIndex() );
                break;
            case MapModelChange::MOVE_ELEVATION_LAYER:
                moveElevationLayer( change.getFirstIndex(), change.getSecondIndex() );
                break;
            case MapModelChange::TOGGLE_ELEVATION_LAYER:
                toggleElevationLayer( change.getElevationLayer() );
                break;
            case MapModelChange::ADD_MODEL_LAYER:
            case MapModelChange::REMOVE_MODEL_LAYER:
            case MapModelChange::MOVE_MODEL_LAYER:
            default: 
                break;
            }
        }
    }
}


void
MPTerrainEngineNode::addImageLayer( ImageLayer* layerAdded )
{
    if ( layerAdded && layerAdded->getEnabled() )
    {
        // for a shared layer, allocate a shared image unit if necessary.
        if ( layerAdded->isShared() )
        {
            optional<int>& unit = layerAdded->shareImageUnit();
            if ( !unit.isSet() )
            {
                int temp;
                if ( getResources()->reserveTextureImageUnit(temp, "MP Engine Shared Layer") )
                {
                    unit = temp;
                    OE_INFO << LC << "Image unit " << temp << " assigned to shared layer " << layerAdded->getName() << std::endl;
                }
                else
                {
                    OE_WARN << LC << "Insufficient GPU image units to share layer " << layerAdded->getName() << std::endl;
                }
            }

            optional<std::string>& texUniformName = layerAdded->shareTexUniformName();
            if ( !texUniformName.isSet() )
            {
                texUniformName = Stringify() << "oe_layer_" << layerAdded->getUID() << "_tex";
            }

            optional<std::string>& texMatUniformName = layerAdded->shareTexMatUniformName();
            if ( !texMatUniformName.isSet() )
            {
                texMatUniformName = Stringify() << "oe_layer_" << layerAdded->getUID() << "_texMatrix";
                OE_INFO << LC << "Layer \"" << layerAdded->getName() << "\" texmat uniform = \"" << texMatUniformName.get() << "\"\n";
            }
        }
    }

    refresh();
}


void
MPTerrainEngineNode::removeImageLayer( ImageLayer* layerRemoved )
{
    if ( layerRemoved )
    {
        // for a shared layer, release the shared image unit.
        if ( layerRemoved->getEnabled() && layerRemoved->isShared() )
        {
            if ( layerRemoved->shareImageUnit().isSet() )
            {
                getResources()->releaseTextureImageUnit( *layerRemoved->shareImageUnit() );
                layerRemoved->shareImageUnit().unset();
            }
        }
    }

    refresh();
}

void
MPTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    updateState();
}

void
MPTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if ( layer == 0L || layer->getEnabled() == false )
        return;

    layer->addCallback( _elevationCallback.get() );

    refresh();
}

void
MPTerrainEngineNode::removeElevationLayer( ElevationLayer* layerRemoved )
{
    if ( layerRemoved->getEnabled() == false )
        return;

    layerRemoved->removeCallback( _elevationCallback.get() );

    refresh();
}

void
MPTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    refresh();
}

void
MPTerrainEngineNode::toggleElevationLayer( ElevationLayer* layer )
{
    refresh();
}


// Generates the main shader code for rendering the terrain.
void
MPTerrainEngineNode::updateState()
{
    if ( _batchUpdateInProgress )
    {
        _stateUpdateRequired = true;
    }
    else
    {
        if ( _elevationTextureUnit < 0 && elevationTexturesRequired() )
        {
            getResources()->reserveTextureImageUnit( _elevationTextureUnit, "MP Engine Elevation" );
        }

        osg::StateSet* terrainStateSet = getTerrainStateSet();
        if ( !terrainStateSet )
            return;
        
        // required for multipass tile rendering to work
        terrainStateSet->setAttributeAndModes(
            new osg::Depth(osg::Depth::LEQUAL, 0, 1, true) );

        // activate standard mix blending.
        terrainStateSet->setAttributeAndModes( 
            new osg::BlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON );

        // install shaders, if we're using them.
        if ( Registry::capabilities().supportsGLSL() )
        {
            VirtualProgram* vp = new VirtualProgram();
            vp->setName( "osgEarth.engine_mp.TerrainNode" );
            terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );

            Shaders package;

            package.replace( "$MP_PRIMARY_UNIT",   Stringify() << _primaryUnit );
            package.replace( "$MP_SECONDARY_UNIT", Stringify() << (_secondaryUnit>=0?_secondaryUnit:0) );

            package.define( "MP_USE_BLENDING", (_terrainOptions.enableBlending() == true) );

            package.load( vp, package.EngineVertexModel );
            package.load( vp, package.EngineVertexView );
            package.load( vp, package.EngineFragment );
            
            if ( this->normalTexturesRequired() )
            {
                package.load( vp, package.NormalMapVertex );
                package.load( vp, package.NormalMapFragment );

                terrainStateSet->addUniform( new osg::Uniform("oe_tile_normalTex", _normalMapUnit) );
            }

            // terrain background color; negative means use the vertex color.
            Color terrainColor = _terrainOptions.color().getOrUse( Color(1,1,1,-1) );
            terrainStateSet->addUniform(new osg::Uniform("oe_terrain_color", terrainColor));

            if ( _update_mapf )
            {
                // assemble color filter code snippets.
                bool haveColorFilters = false;
                {
                    // Color filter frag function:
                    std::string fs_colorfilters =
                        "#version " GLSL_VERSION_STR "\n"
                        GLSL_DEFAULT_PRECISION_FLOAT "\n"
                        "uniform int oe_layer_uid; \n"
                        "$COLOR_FILTER_HEAD"
                        "void oe_mp_apply_filters(inout vec4 color) \n"
                        "{ \n"
                            "$COLOR_FILTER_BODY"
                        "} \n";

                    std::stringstream cf_head;
                    std::stringstream cf_body;
                    const char* I = "    ";

                    // second, install the per-layer color filter functions AND shared layer bindings.
                    bool ifStarted = false;
                    int numImageLayers = _update_mapf->imageLayers().size();
                    for( int i=0; i<numImageLayers; ++i )
                    {
                        ImageLayer* layer = _update_mapf->getImageLayerAt(i);
                        if ( layer->getEnabled() )
                        {
                            // install Color Filter function calls:
                            const ColorFilterChain& chain = layer->getColorFilters();
                            if ( chain.size() > 0 )
                            {
                                haveColorFilters = true;
                                if ( ifStarted ) cf_body << I << "else if ";
                                else             cf_body << I << "if ";
                                cf_body << "(oe_layer_uid == " << layer->getUID() << ") {\n";
                                for( ColorFilterChain::const_iterator j = chain.begin(); j != chain.end(); ++j )
                                {
                                    const ColorFilter* filter = j->get();
                                    cf_head << "void " << filter->getEntryPointFunctionName() << "(inout vec4 color);\n";
                                    cf_body << I << I << filter->getEntryPointFunctionName() << "(color);\n";
                                    filter->install( terrainStateSet );
                                }
                                cf_body << I << "}\n";
                                ifStarted = true;
                            }
                        }
                    }

                    if ( haveColorFilters )
                    {
                        std::string cf_head_str, cf_body_str;
                        cf_head_str = cf_head.str();
                        cf_body_str = cf_body.str();

                        replaceIn( fs_colorfilters, "$COLOR_FILTER_HEAD", cf_head_str );
                        replaceIn( fs_colorfilters, "$COLOR_FILTER_BODY", cf_body_str );

                        vp->setFunction(
                            "oe_mp_apply_filters",
                            fs_colorfilters,
                            ShaderComp::LOCATION_FRAGMENT_COLORING,
                            0.5f );
                    }
                }
            }

            // binding for the terrain texture
            terrainStateSet->getOrCreateUniform( 
                "oe_layer_tex", osg::Uniform::SAMPLER_2D )->set( _primaryUnit );

            // binding for the secondary texture (for LOD blending)
            if ( parentTexturesRequired() )
            {
                terrainStateSet->getOrCreateUniform(
                    "oe_layer_tex_parent", osg::Uniform::SAMPLER_2D )->set( _secondaryUnit );

                // binding for the default secondary texture matrix
                osg::Matrixf parent_mat;
                parent_mat(0,0) = 0.0f;
                terrainStateSet->getOrCreateUniform(
                    "oe_layer_parent_matrix", osg::Uniform::FLOAT_MAT4 )->set( parent_mat );
            }

            // uniform for accessing the elevation texture sampler.
            if ( elevationTexturesRequired() )
            {
                terrainStateSet->getOrCreateUniform(
                    "oe_terrain_tex", osg::Uniform::SAMPLER_2D)->set( _elevationTextureUnit );
            }

            // uniform that controls per-layer opacity
            terrainStateSet->getOrCreateUniform(
                "oe_layer_opacity", osg::Uniform::FLOAT )->set( 1.0f );

            // uniform that conveys the layer UID to the shaders; necessary
            // for per-layer branching (like color filters)
            // UID -1 => no image layer (no texture)
            terrainStateSet->getOrCreateUniform(
                "oe_layer_uid", osg::Uniform::INT )->set( -1 );

            // uniform that conveys the render order, since the shaders
            // need to know which is the first layer in order to blend properly
            terrainStateSet->getOrCreateUniform(
                "oe_layer_order", osg::Uniform::INT )->set( 0 );

            // default min/max range uniforms. (max < min means ranges are disabled)
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_minRange", 0.0f) );
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_maxRange", FLT_MAX) );
            terrainStateSet->addUniform( new osg::Uniform("oe_layer_attenuationRange", _terrainOptions.attentuationDistance().get()) );
            
            terrainStateSet->getOrCreateUniform(
                "oe_min_tile_range_factor",
                osg::Uniform::FLOAT)->set( *_terrainOptions.minTileRangeFactor() );

            // special object ID that denotes the terrain surface.
            terrainStateSet->addUniform( new osg::Uniform(
                Registry::objectIndex()->getObjectIDUniformName().c_str(), OSGEARTH_OBJECTID_TERRAIN) );

            // assign the uniforms for each shared layer.
            if ( _update_mapf )
            {
                int numImageLayers = _update_mapf->imageLayers().size();
                for( int i=0; i<numImageLayers; ++i )
                {
                    ImageLayer* layer = _update_mapf->getImageLayerAt(i);
                    if ( layer->getEnabled() && layer->isShared() )
                    {
                        terrainStateSet->addUniform( new osg::Uniform(
                            layer->shareTexUniformName()->c_str(),
                            layer->shareImageUnit().get() ) );
                        
                    }
                }
            }
        }

        _stateUpdateRequired = false;
    }
}
