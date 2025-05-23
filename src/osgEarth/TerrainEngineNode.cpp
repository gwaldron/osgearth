/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
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

        void onMapModelChanged( const MapModelChange& change )
        {
            osg::ref_ptr<TerrainEngineNode> safeNode;
            if (_node.lock(safeNode))
                safeNode->onMapModelChanged( change );
        }
    };
}


//...................................................................

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


//------------------------------------------------------------------------


TerrainEngineNode::TerrainEngineNode() :
    _dirtyCount(0),
    _updateScheduled(false)
{
    // register for event traversals so we can properly reset the dirtyCount
    ADJUST_EVENT_TRAV_COUNT(this, 1);

    // Install an object to manage texture image unit usage:
    _textureResourceTracker = new TerrainResources();
    std::set<int> offLimits = osgEarth::Registry::instance()->getOffLimitsTextureImageUnits();
    for (std::set<int>::const_iterator i = offLimits.begin(); i != offLimits.end(); ++i)
        _textureResourceTracker->setTextureImageUnitOffLimits(*i);
}

TerrainEngineNode::~TerrainEngineNode()
{
    //nop
}

namespace
{
    struct RequestRedraw
    {
        void operator()(osg::View* view) {
            osgGA::GUIActionAdapter* aa = dynamic_cast<osgGA::GUIActionAdapter*>(view);
            if (aa) aa->requestRedraw();
        }
    };
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
TerrainEngineNode::shutdown()
{
    // DO NOT destroy the tile model factory; it may still be in use
    // by a loading thread via a ref_ptr lock (see LoadTileData).
    //_tileModelFactory = nullptr;
}

void
TerrainEngineNode::setMap(const Map* map, const TerrainOptions& options)
{
    if (!map) return;

    if (!map->getProfile())
    {
        OE_WARN << "Illegal: Map profile is not set" << std::endl;
        return;
    }

    _map = map;

    // store a const copy of the terrain options
    _optionsConcrete = options;

    // Create a terrain utility interface. This interface can be used
    // to query the in-memory terrain graph, subscribe to tile events, etc.
    _terrainInterface = new Terrain( this, map->getProfile() );

    // Register a callback so we can process further map model changes
    _map->addMapCallback( new TerrainEngineNodeCallbackProxy(this) );

    // Force a render bin if specified in the options
    if ( options.renderBinNumber().isSet() )
    {
        osg::StateSet* set = getOrCreateStateSet();
        set->setRenderBinDetails(options.renderBinNumber().get(), "RenderBin" );
    }

    // This is the object that creates the data model for each terrain tile.
    _tileModelFactory = new TerrainTileModelFactory(options);

    // Cross-compatibility stuff supporting CSN.
    // osgEarth doesn't use this but other systems might.
    if (_map->getProfile())
    {
        // set up the CSN values
        _map->getProfile()->getSRS()->populateCoordinateSystemNode( this );

        // OSG's CSN likes a NULL ellipsoid to represent projected mode.
        if (_map->getProfile()->getSRS()->isProjected())
        {
            this->setEllipsoidModel(nullptr);
        }
    }

    // Hide the terrain engine if it's marked as not visible.  This is mostly used when
    // rendering whole earth layers like Google Earth tiles and you don't want to see the globe.
    if (!*options.visible())
    {
        setNodeMask(0);
    }

    // invoke the callback for a subclass to do its thing
    onSetMap();
}

osg::BoundingSphere
TerrainEngineNode::computeBound() const
{
    if ( getMap() && getMap()->getSRS())
    {
        const Ellipsoid& e = getMap()->getSRS()->getEllipsoid();
        double maxRad = std::max(
            e.getRadiusEquator(),
            e.getRadiusPolar());

        return osg::BoundingSphere( osg::Vec3(0,0,0), maxRad+25000 );
    }
    else
    {
        return osg::CoordinateSystemNode::computeBound();
    }
}

void
TerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    if (change.getElevationLayer() != 0L)
    {
        getTerrain()->notifyMapElevationChanged();
    }

    // notify that a redraw is required.
    requestRedraw();
}

TerrainTileModel*
TerrainEngineNode::createTileModel(const Map* map,
                                   const TileKey& key,
                                   const CreateTileManifest& manifest,
                                   ProgressCallback* progress)
{
    if ( !_tileModelFactory.valid() )
        return nullptr;

    // Ask the factory to create a new tile model:
    osg::ref_ptr<TerrainTileModel> model = _tileModelFactory->createTileModel(
        map,
        key,
        manifest,
        getRequirements(),
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

TerrainEngineNode*
TerrainEngineNode::create(const TerrainOptions& options )
{
    osg::ref_ptr<TerrainEngineNode> node;

    std::string driver =
        Registry::instance()->overrideTerrainEngineDriverName().getOrUse(options.getDriver());

    if ( driver.empty() )
        driver = Registry::instance()->getDefaultTerrainEngineDriverName();

    std::string driverExt = std::string("osgearth_engine_") + driver;
    auto rw = osgDB::Registry::instance()->getReaderWriterForExtension(driverExt);
    if (rw)
    {
        osg::ref_ptr<osg::Object> object = rw->readObject("." + driverExt).getObject();
        node = dynamic_cast<TerrainEngineNode*>(object.release());
        if (!node)
        {
            OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << driver << "\"" << std::endl;
        }
    }

    return node.release();
}
