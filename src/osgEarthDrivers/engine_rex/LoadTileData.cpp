/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include "LoadTileData"
#include "SurfaceNode"
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Terrain>
#include <osg/NodeVisitor>

using namespace osgEarth::REX;
using namespace osgEarth;

#define LC "[LoadTileData] "


LoadTileData::LoadTileData(TileNode* tilenode, EngineContext* context) :
_tilenode(tilenode),
_context(context),
_enableCancel(true)
{
    this->setTileKey(tilenode->getKey());
    _map = context->getMap();
    _engine = context->getEngine();
}

void
LoadTileData::setLayerFilter(const std::set<UID>& layers)
{
    ScopedMutexLock lock(_mutex);
    _filter.clear();
    _filter.layers() = layers;
}

void
LoadTileData::addLayerToFilter(const UID& layer)
{
    ScopedMutexLock lock(_mutex);
    _filter.layers().insert(layer);
}

void
LoadTileData::clearLayerFilter()
{
    ScopedMutexLock lock(_mutex);
    _filter.clear();
}

// invoke runs in the background pager thread.
void
LoadTileData::invoke(ProgressCallback* progress)
{
    osg::ref_ptr<TileNode> tilenode;
    if (!_tilenode.lock(tilenode))
        return;

    osg::ref_ptr<TerrainEngineNode> engine;
    if (!_engine.lock(engine))
        return;

    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return;

    CreateTileModelFilter filter;
    {
        ScopedMutexLock lock(_mutex);
        filter = _filter;
    }

    // Assemble all the components necessary to display this tile
    _dataModel = engine->createTileModel(
        map.get(),
        tilenode->getKey(),
        filter,
        _enableCancel? progress : 0L);

    // if the operation was canceled, set the request to idle and delete the tile model.
    if (progress && progress->isCanceled())
    {
        _dataModel = 0L;
        setState(Request::IDLE);
    }

    // In the terrain engine, we have to keep our elevation rasters in 
    // memory since we use them to build intersection graphs.
    if (_dataModel.valid() && _dataModel->getElevationTexture())
    {
        _dataModel->getElevationTexture()->setUnRefImageDataAfterApply(false);
    }

    clearLayerFilter();
}


// apply() runs in the update traversal and can safely alter the scene graph
void
LoadTileData::apply(const osg::FrameStamp* stamp)
{
    osg::ref_ptr<EngineContext> context;
    if (!_context.lock(context))
        return;

    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return;

    // ensure we got an actual datamodel:
    if (_dataModel.valid())
    {
        // ensure it's in sync with the map revision (not out of date):
        if (map.valid() && _dataModel->getRevision() == map->getDataModelRevision())
        {
            // ensure the tile node hasn't expired:
            osg::ref_ptr<TileNode> tilenode;
            if ( _tilenode.lock(tilenode) )
            {
                const RenderBindings& bindings = context->getRenderBindings();

                // Merge the new data into the tile.
                tilenode->merge(_dataModel.get(), bindings);

                // Mark as complete. TODO: per-data requests will do something different.
                tilenode->setDirty( false );

                OE_DEBUG << LC << "apply " << _dataModel->getKey().str() << "\n";
            }
            else
            {
                OE_DEBUG << LC << "LoadTileData failed; TileNode disappeared\n";
            }
        }
        else
        {
            OE_INFO << LC << "apply " << _dataModel->getKey().str() << " ignored b/c it is out of date\n";
        }

        // Delete the model immediately
        _dataModel = 0L;
    }
}

namespace
{
    // Fake attribute that compiles everything in the TerrainTileModel
    // when the ICO is active.
    struct ModelCompilingAttribute : public osg::Texture2D
    {
        osg::observer_ptr<TerrainTileModel> _dataModel;
        
        // the ICO calls apply() directly instead of compileGLObjects
        void apply(osg::State& state) const
        {
            osg::ref_ptr<TerrainTileModel> dataModel;
            if (_dataModel.lock(dataModel))
            {
                OE_DEBUG << "MCA: compiling " << dataModel->getKey().str() << std::endl;
                dataModel->compileGLObjects(state);
            }
        }

        // no need to override release or resize since this is a temporary object
        // that exists only to service the ICO.

        META_StateAttribute(osgEarth, ModelCompilingAttribute, osg::StateAttribute::TEXTURE);
        int compare(const StateAttribute& sa) const { return 0; }
        ModelCompilingAttribute() { }
        ModelCompilingAttribute(const ModelCompilingAttribute& rhs, const osg::CopyOp& copy) { }
    };
}

osg::StateSet*
LoadTileData::createStateSet() const
{
    osg::ref_ptr<osg::StateSet> out;

    osg::ref_ptr<EngineContext> context;
    if (!_context.lock(context))
        return NULL;

    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return NULL;

    if (_dataModel.valid() && map.valid() &&
        _dataModel->getRevision() == map->getDataModelRevision())
    {
        // This stateset contains a "fake" attribute that the ICO will
        // try to GL-compile, thereby GL-compiling everything in the TerrainTileModel.
        out = new osg::StateSet();
        ModelCompilingAttribute* mca = new ModelCompilingAttribute();
        mca->_dataModel = _dataModel.get();
        out->setTextureAttribute(0, mca, 1);
    }

    return out.release();
}