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
#include <osgEarth/Metrics>
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

LoadTileData::LoadTileData(const CreateTileManifest& manifest, TileNode* tilenode, EngineContext* context) :
    _manifest(manifest),
    _tilenode(tilenode),
    _context(context),
    _enableCancel(true)
{
    this->setTileKey(tilenode->getKey());
    _map = context->getMap();
    _engine = context->getEngine();
}

// invoke runs in the background pager thread.
bool
LoadTileData::run(ProgressCallback* progress)
{
    osg::ref_ptr<TileNode> tilenode;
    if (!_tilenode.lock(tilenode))
        return false;

    osg::ref_ptr<TerrainEngineNode> engine;
    if (!_engine.lock(engine))
        return false;

    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return false;

    // if the operation was canceled, set the request to abandoned
    // so it can potentially retry later.
    // TODO: Consider the canceler setting some kind of retry delay
    // for failed network attempts
    if (progress && progress->isCanceled())
    {
        _dataModel = 0L;
        //OE_WARN << _key.str() << " .. canceled before createTileModel" << std::endl;
        setDelay(progress->getRetryDelay());
        return false;
    }

    // Assemble all the components necessary to display this tile
    _dataModel = engine->createTileModel(
        map.get(),
        tilenode->getKey(),
        _manifest,
        _enableCancel? progress : 0L);

    // if the operation was canceled, set the request to abandoned
    // so it can potentially retry later. 
    // TODO: Consider the canceler setting some kind of retry delay
    // for failed network attempts
    if (progress && progress->isCanceled())
    {
        _dataModel = 0L;
        //OE_WARN << _key.str() << " .. canceled after createTileModel" << std::endl;
        setDelay(progress->getRetryDelay());
        return false;
    }

    // In the terrain engine, we have to keep our elevation rasters in 
    // memory since we use them to build intersection graphs.
    if (_dataModel.valid() && _dataModel->getElevationTexture())
    {
        _dataModel->getElevationTexture()->setUnRefImageDataAfterApply(false);
    }

    return _dataModel.valid();
}


// apply() runs in the update traversal and can safely alter the scene graph
bool
LoadTileData::merge()
{
    // context went out of scope - bail
    osg::ref_ptr<EngineContext> context;
    if (!_context.lock(context))
        return true;

    // map went out of scope - bail
    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return true;

    // tilenode went out of scope - bail
    osg::ref_ptr<TileNode> tilenode;
    if (!_tilenode.lock(tilenode))
        return true;

    // no data model at all - done
    // GW: how does this happen? shouldn't happen.
    if (!_dataModel.valid())
    {
        OE_WARN << _key.str() << " bailing out of merge b/c data model is NULL" << std::endl;
        return false;
    }

    OE_PROFILING_ZONE;

    // Check the map data revision and scan the manifest and see if any
    // revisions don't match the revisions in the original manifest.
    // If there are mismatches, that means the map has changed since we
    // submitted this request, and the results are now invalid.
    if (_dataModel->getRevision() != map->getDataModelRevision() ||
        _manifest.inSyncWith(map.get()) == false)
    {
        // wipe the data model, update the revisions, and try again.
        _manifest.updateRevisions(map.get());
        _dataModel = 0L;
        OE_DEBUG << LC << "Request for tile " << _key.str() << " out of date and will be requeued" << std::endl;
        return false;
    }

    // Merge the new data into the tile.
    tilenode->merge(_dataModel.get(), this);

    OE_DEBUG << LC << "apply " << _dataModel->getKey().str() << "\n";

    // Delete the model immediately
    _dataModel = 0L;
    return true;
}

namespace
{
    // Fake attribute that compiles everything in the TerrainTileModel
    // when the ICO is active.
    struct TileICO : public osg::Texture2D
    {
        osg::observer_ptr<TerrainTileModel> _dataModel;
        
        // the ICO calls apply() directly instead of compileGLObjects
        void apply(osg::State& state) const
        {
            osg::ref_ptr<TerrainTileModel> dataModel;
            if (_dataModel.lock(dataModel))
            {
                OE_PROFILING_ZONE;
                OE_PROFILING_ZONE_TEXT(_dataModel->getKey().str());
                OE_DEBUG << "MCA: compiling " << dataModel->getKey().str() << std::endl;
                dataModel->compileGLObjects(state);
            }
        }

        // no need to override release or resize since this is a temporary object
        // that exists only to service the ICO.

        META_StateAttribute(osgEarth, TileICO, osg::StateAttribute::TEXTURE);
        int compare(const StateAttribute& sa) const { return 0; }
        TileICO() { }
        TileICO(const TileICO& rhs, const osg::CopyOp& copy) { }
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
        TileICO* mca = new TileICO();
        mca->_dataModel = _dataModel.get();
        out->setTextureAttribute(0, mca, 1);
    }

    return out.release();
}
