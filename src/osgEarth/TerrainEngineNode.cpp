/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Registry>
#include <osgEarth/MapModelChange>

#define LC "[TerrainEngineNode] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace osgEarth
{
    struct TerrainEngineNodeCallbackProxy : public MapCallback
    {
        TerrainEngineNodeCallbackProxy(TerrainEngineNode* node) : _node(node) { }

        osg::observer_ptr<TerrainEngineNode> _node;

        void onMapInfoEstablished( const MapInfo& mapInfo )
        {
            osg::ref_ptr<TerrainEngineNode> safeNode;
            if (_node.lock(safeNode))
                safeNode->onMapInfoEstablished( mapInfo );
        }

        void onMapModelChanged( const MapModelChange& change )
        {
            osg::ref_ptr<TerrainEngineNode> safeNode;
            if (_node.lock(safeNode))
                safeNode->onMapModelChanged( change );
        }
    };
}


//------------------------------------------------------------------------


TerrainEngineNode::ImageLayerController::ImageLayerController(TerrainEngineNode* engine) :
_engine( engine )
{    
    //nop
}


void
TerrainEngineNode::addEffect(TerrainEffect* effect)
{
    if ( effect )
    {
        effects_.push_back( effect );
        effect->onInstall( this );
        dirtyState();
    }
}


void
TerrainEngineNode::removeEffect(TerrainEffect* effect)
{
    if ( effect )
    {
        effect->onUninstall(this);
        TerrainEffectVector::iterator i = std::find(effects_.begin(), effects_.end(), effect);
        if ( i != effects_.end() )
            effects_.erase( i );
        dirtyState();
    }
}


TerrainResources*
TerrainEngineNode::getResources() const
{
    return _textureResourceTracker.get();
}

void
TerrainEngineNode::ImageLayerController::onColorFiltersChanged( ImageLayer* layer )
{
    _engine->updateTextureCombining();
}


//------------------------------------------------------------------------


TerrainEngineNode::TerrainEngineNode() :
_verticalScale           ( 1.0f ),
_initStage               ( INIT_NONE ),
_dirtyCount              ( 0 ),
_requireElevationTextures( false ),
_requireNormalTextures   ( false ),
_requireParentTextures   ( false ),
_requireElevationBorder  ( false ),
_requireFullDataAtFirstLOD( false ),
_redrawRequired          ( true ),
_updateScheduled( false )
{
    // register for event traversals so we can properly reset the dirtyCount
    ADJUST_EVENT_TRAV_COUNT(this, 1);

    // register for update traversals so we can process terrain callbacks
    //ADJUST_UPDATE_TRAV_COUNT(this, 1);
}

TerrainEngineNode::~TerrainEngineNode()
{
    OE_DEBUG << LC << "~TerrainEngineNode\n";

    //Remove any callbacks added to the image layers
    if (_map.valid())
    {
        ImageLayerVector imageLayers;
        _map->getLayers(imageLayers);

        for( ImageLayerVector::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
        {
            i->get()->removeCallback( _imageLayerController.get() );
        }
    }
}


void
TerrainEngineNode::requireNormalTextures()
{
    _requireNormalTextures = true;
    dirtyTerrain();
}

void
TerrainEngineNode::requireElevationTextures()
{
    _requireElevationTextures = true;
    dirtyTerrain();
}

void
TerrainEngineNode::requireParentTextures()
{
    _requireParentTextures = true;
    dirtyTerrain();
}


void
TerrainEngineNode::requestRedraw()
{
    if ( 0 == _dirtyCount++ )
    {
        // notify any attached Views
        ViewVisitor<RequestRedraw> visitor;
        this->accept(visitor);
    }
}

void
TerrainEngineNode::dirtyTerrain()
{
    requestRedraw();
}

void
TerrainEngineNode::setMap(const Map* map, const TerrainOptions& options)
{
    if (!map) return;

    _map = map;
    
    // Create a terrain utility interface. This interface can be used
    // to query the in-memory terrain graph, subscribe to tile events, etc.
    _terrainInterface = new Terrain( this, map->getProfile(), options );

    // Set up the CSN values. We support this because some manipulators look for it,
    // but osgEarth itself doesn't use it.
    _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( !_map->isGeocentric() )
        this->setEllipsoidModel( NULL );
    
    // Install an object to manage texture image unit usage:
    _textureResourceTracker = new TerrainResources();
    std::set<int> offLimits = osgEarth::Registry::instance()->getOffLimitsTextureImageUnits();
    for(std::set<int>::const_iterator i = offLimits.begin(); i != offLimits.end(); ++i)
        _textureResourceTracker->setTextureImageUnitOffLimits( *i );

    // Register a callback so we can process further map model changes
    _map->addMapCallback( new TerrainEngineNodeCallbackProxy(this) );

    // Force a render bin if specified in the options
    if ( options.binNumber().isSet() )
    {
        osg::StateSet* set = getOrCreateStateSet();
        set->setRenderBinDetails( options.binNumber().get(), "RenderBin" );
    }
   
    // This is the object that creates the data model for each terrain tile.
    _tileModelFactory = new TerrainTileModelFactory(options);

    // Manually trigger the map callbacks the first time:
    if (_map->getProfile())
        onMapInfoEstablished(MapInfo(_map.get()));

    // Create a layer controller. This object affects the uniforms
    // that control layer appearance properties
    _imageLayerController = new ImageLayerController(this);

    // register the layer Controller it with all pre-existing image layers:
    ImageLayerVector imageLayers;
    _map->getLayers(imageLayers);
    for (ImageLayerVector::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i)
    {
        i->get()->addCallback(_imageLayerController.get());
    }

    _initStage = INIT_POSTINIT_COMPLETE;
}

osg::BoundingSphere
TerrainEngineNode::computeBound() const
{
    if ( getEllipsoidModel() )
    {
        double maxRad = osg::maximum(
            getEllipsoidModel()->getRadiusEquator(),
            getEllipsoidModel()->getRadiusPolar());

        return osg::BoundingSphere( osg::Vec3(0,0,0), maxRad+25000 );
    }
    else
    {
        return osg::CoordinateSystemNode::computeBound();
    }
}

void
TerrainEngineNode::onMapInfoEstablished( const MapInfo& mapInfo )
{
    // set up the CSN values   
    mapInfo.getProfile()->getSRS()->populateCoordinateSystemNode( this );
    
    // OSG's CSN likes a NULL ellipsoid to represent projected mode.
    if ( !mapInfo.isGeocentric() )
        this->setEllipsoidModel( NULL );
}

void
TerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    if (change.getAction() == MapModelChange::ADD_LAYER &&
        change.getImageLayer() != 0L)
    {
        change.getImageLayer()->addCallback( _imageLayerController.get() );
    }
    else if (change.getAction() == MapModelChange::REMOVE_LAYER &&
        change.getImageLayer() != 0L)
    {
        change.getImageLayer()->removeCallback( _imageLayerController.get() );
    }

    if (change.getElevationLayer() != 0L)
    {
        getTerrain()->notifyMapElevationChanged();
    }

    // notify that a redraw is required.
    requestRedraw();
}

TerrainTileModel*
TerrainEngineNode::createTileModel(const Map*                   map,
                                   const TileKey&               key,
                                   const CreateTileModelFilter& filter,
                                   ProgressCallback*            progress)
{
    TerrainEngineRequirements* requirements = this;

    // Ask the factory to create a new tile model:
    osg::ref_ptr<TerrainTileModel> model = _tileModelFactory->createTileModel(
        map, 
        key, 
        filter,
        requirements,         
        progress);

    if ( model.valid() )
    {
        // Fire all registered tile model callbacks, so user code can 
        // add to or otherwise customize the model before it's returned
        Threading::ScopedReadLock sharedLock(_createTileModelCallbacksMutex);
        for(CreateTileModelCallbacks::iterator i = _createTileModelCallbacks.begin();
            i != _createTileModelCallbacks.end();
            ++i)
        {
            i->get()->onCreateTileModel(this, model.get());
        }
    }
    return model.release();
}

void 
TerrainEngineNode::addCreateTileModelCallback(CreateTileModelCallback* callback)
{
    Threading::ScopedWriteLock exclusiveLock(_createTileModelCallbacksMutex);
    _createTileModelCallbacks.push_back(callback);
}

void 
TerrainEngineNode::removeCreateTileModelCallback(CreateTileModelCallback* callback)
{
    Threading::ScopedWriteLock exclusiveLock(_createTileModelCallbacksMutex);
    for(CreateTileModelCallbacks::iterator i = _createTileModelCallbacks.begin(); i != _createTileModelCallbacks.end(); ++i)
    {
        if ( i->get() == callback )
        {
            _createTileModelCallbacks.erase( i );
            break;
        }
    }
}

void
TerrainEngineNode::addModifyTileBoundingBoxCallback(ModifyTileBoundingBoxCallback* callback)
{
    Threading::ScopedWriteLock exclusiveLock(_createTileModelCallbacksMutex);
    _modifyTileBoundingBoxCallbacks.push_back(callback);
}

void
TerrainEngineNode::removeModifyTileBoundingBoxCallback(ModifyTileBoundingBoxCallback* callback)
{
    Threading::ScopedWriteLock exclusiveLock(_createTileModelCallbacksMutex);
    for (ModifyTileBoundingBoxCallbacks::iterator i = _modifyTileBoundingBoxCallbacks.begin();
        i != _modifyTileBoundingBoxCallbacks.end();
        ++i)
    {
        if (i->get() == callback)
        {
            _modifyTileBoundingBoxCallbacks.erase(i);
            break;
        }
    }
}

void
TerrainEngineNode::fireModifyTileBoundingBoxCallbacks(const TileKey& key, osg::BoundingBox& box)
{
    Threading::ScopedReadLock sharedLock(_createTileModelCallbacksMutex);
    for (ModifyTileBoundingBoxCallbacks::iterator i = _modifyTileBoundingBoxCallbacks.begin();
        i != _modifyTileBoundingBoxCallbacks.end();
        ++i)
    {
        i->get()->modifyBoundingBox(key, box);
    }
}

namespace
{
    Threading::Mutex s_opqlock;
}

void
TerrainEngineNode::traverse( osg::NodeVisitor& nv )
{    
    if ( nv.getVisitorType() == nv.EVENT_VISITOR )
    {
        _dirtyCount = 0;
        if (_updateScheduled == false && _terrainInterface->_updateQueue->empty() == false)
        {
            ADJUST_UPDATE_TRAV_COUNT(this, +1);
            _updateScheduled = true;
        }
    }

    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        if (_updateScheduled == true )
        {
            _terrainInterface->update();
            ADJUST_UPDATE_TRAV_COUNT(this, -1);
            _updateScheduled = false;
        }
    }

    osg::CoordinateSystemNode::traverse( nv );
}

//todo: remove?
void
TerrainEngineNode::notifyOfTerrainTileNodeCreation(const TileKey& key, osg::Node* node)
{
    Threading::ScopedMutexLock lock(_tileNodeCallbacksMutex);
    for(unsigned i=0; i<_tileNodeCallbacks.size(); ++i)
    {
        _tileNodeCallbacks[i]->operator()(key, node);
    }
}

ComputeRangeCallback*
TerrainEngineNode::getComputeRangeCallback() const
{
    return _computeRangeCallback.get();
}

void
TerrainEngineNode::setComputeRangeCallback(ComputeRangeCallback* computeRangeCallback)
{
    _computeRangeCallback = computeRangeCallback;
}


//------------------------------------------------------------------------

#undef LC
#define LC "[TerrainEngineNodeFactory] "

TerrainEngineNode*
TerrainEngineNodeFactory::create(const TerrainOptions& options )
{
    osg::ref_ptr<TerrainEngineNode> node;

    std::string driver =
        Registry::instance()->overrideTerrainEngineDriverName().getOrUse(options.getDriver());

    if ( driver.empty() )
        driver = Registry::instance()->getDefaultTerrainEngineDriverName();

    std::string driverExt = std::string( ".osgearth_engine_" ) + driver;
    osg::ref_ptr<osg::Object> object = osgDB::readRefObjectFile( driverExt );
    node = dynamic_cast<TerrainEngineNode*>( object.release() );
    if ( !node )
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << driver << "\"" << std::endl;
    }

    return node.release();
}

