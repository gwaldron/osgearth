/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include "MPTexture"
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Terrain>
#include <osgEarth/Registry>
#include <osg/NodeVisitor>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[LoadTileData] "

namespace
{
    // Visitor that recalculates the sampler inheritance matrices in a graph.
    struct UpdateInheritance : public osg::NodeVisitor
    {
        UpdateInheritance(EngineContext* context, Loader::Request::ChangeSet& changeSet)
            : _context(context),
              _changeSet(changeSet)
        {
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
        }

        void apply(osg::Group& node)
        {
            TileNode* tilenode = dynamic_cast<TileNode*>(&node);
            if ( tilenode )
            {
                if ( tilenode->inheritState( _context ) )
                {
                    // return true = changes occurred.
                    _changeSet.push_back( tilenode );
                }
            }

            traverse(node);
        }

        Loader::Request::ChangeSet& _changeSet;
        EngineContext*              _context;
    };
}

//............................................................................


LoadTileData::LoadTileData(TileNode* tilenode, EngineContext* context) :
_tilenode(tilenode),
_context(context)
{
    //nop
}

namespace
{
    void applyDefaultUnRefPolicy(osg::Texture* tex)
    {
        const optional<bool>& unRefPolicy = Registry::instance()->unRefImageDataAfterApply();
        tex->setUnRefImageDataAfterApply( unRefPolicy.get() );
    }
}


// invoke runs in the background pager thread.
void
LoadTileData::invoke()
{
    osg::ref_ptr<TileNode> tilenode;
    if ( _tilenode.lock(tilenode) )
    {
        osg::ref_ptr<ProgressCallback> progress;

        // Assemble all the components necessary to display this tile
        _model = _context->getEngine()->createTileModel(
            _context->getMapFrame(),
            tilenode->getTileKey(),
            progress ); // progress

        // Prep the stateset for merging (and for GL pre-compile).
        if ( _model.valid() )
        {
#if 0
            if ( progress.valid() )
            {
                int count = (int)progress->stats("http_get_count");
                if ( count > 0 )
                {
                    double t = (progress->stats("http_get_time")*1000.0);
                    OE_NOTICE << LC << tilenode->getTileKey().str()
                        << " : http_get_time = " << t << " ms, "
                        << " : http_get_count = " << count << ", avg = " << (t/count) << std::endl;
                }
            }
#endif
            _data._passes.clear();
            _data._passesByUID.clear();

            const RenderBindings& bindings = _context->getRenderBindings();

            // Add color passes:
            const SamplerBinding& color = bindings[SamplerBinding::COLOR];
            if (color.isUsed())
            {
                for (TerrainTileImageLayerModelVector::const_iterator i = _model->colorLayers().begin();
                    i != _model->colorLayers().end();
                    ++i)
                {
                    TerrainTileImageLayerModel* layer = i->get();
                    if (layer && layer->getTexture())
                    {                        
                        RenderingPass& pass = _data.addPass(layer->getImageLayer()->getUID());
                        pass._valid = true;
                        pass._samplers[SamplerBinding::COLOR]._texture = layer->getTexture();
                        applyDefaultUnRefPolicy(layer->getTexture());
                    }
                }                
            }
            else
            {
                // No color layers? We need a rendering pass with a null texture then
                // to accomadate the other samplers.
                RenderingPass& pass = _data.addPass(-1);
                pass._valid = true;
            }

            // Elevation:
            const SamplerBinding& elevation = bindings[SamplerBinding::ELEVATION];
            if (elevation.isUsed() && _model->elevationModel().valid() && _model->elevationModel()->getTexture())
            {
                osg::Texture* tex = _model->elevationModel()->getTexture();
                // always keep the elevation image around because we use it for bounding box computation:
                tex->setUnRefImageDataAfterApply(false);
                for (unsigned p = 0; p<_data._passes.size(); ++p)
                {
                    _data._passes[p]._samplers[SamplerBinding::ELEVATION]._texture = tex;
                }
            }

            // Normals:
            const SamplerBinding& normals = bindings[SamplerBinding::NORMAL];
            if (normals.isUsed() && _model->normalModel().valid() && _model->normalModel()->getTexture())
            {
                osg::Texture* tex = _model->normalModel()->getTexture();
                applyDefaultUnRefPolicy(tex);
                for (unsigned p = 0; p<_data._passes.size(); ++p)
                {
                     _data._passes[p]._samplers[SamplerBinding::NORMAL]._texture = tex;
                }
            }

            // Shared Layers:
            for (unsigned i = 0; i < _model->sharedLayers().size(); ++i)
            {
                unsigned bindingIndex = SamplerBinding::SHARED + i;
                const SamplerBinding& binding = bindings[bindingIndex];
               
                TerrainTileImageLayerModel* layerModel = _model->sharedLayers().at(i);
                if ( layerModel->getTexture() )
                {
                    osg::Texture* tex = layerModel->getTexture();
                    applyDefaultUnRefPolicy(tex);
                    for (unsigned p = 0; p<_data._passes.size(); ++p)
                    {
                         _data._passes[p]._samplers[bindingIndex]._texture = tex;
                    }
                }
            }

#if 0
            // OLD CODE:
            {
                const RenderBindings& bindings = _context->getRenderBindings();

                osg::StateSet* stateSet = getStateSet();

                // Insert all the color layers into a new MPTexture state attribute,
                // which exists to facilitate GL pre-compilation.
                if (_model->colorLayers().size() > 0)
                {
                    const SamplerBinding* colorBinding = SamplerBinding::findUsage(bindings, SamplerBinding::COLOR);
                    if (colorBinding)
                    {
                        osg::ref_ptr<MPTexture> mptex = new MPTexture();

                        for (TerrainTileImageLayerModelVector::iterator i = _model->colorLayers().begin();
                            i != _model->colorLayers().end();
                            ++i)
                        {
                            TerrainTileImageLayerModel* layerModel = i->get();
                            if (layerModel && layerModel->getTexture())
                            {
                                applyDefaultUnRefPolicy(layerModel->getTexture());
                                mptex->setPass(layerModel->getImageLayer(), layerModel->getTexture(), layerModel->getOrder());
                            }
                        }

                        if (!mptex->getPasses().empty())
                        {
                            stateSet->setTextureAttribute(
                                colorBinding->unit(),
                                mptex);
                        }
                    }
                }

                // Insert the elevation texture and an identity matrix:
                if (_model->elevationModel().valid() && _model->elevationModel()->getTexture())
                {
                    const SamplerBinding* binding = SamplerBinding::findUsage(bindings, SamplerBinding::ELEVATION);
                    if (binding)
                    {
                        applyDefaultUnRefPolicy(_model->elevationModel()->getTexture());

                        stateSet->setTextureAttribute(
                            binding->unit(),
                            _model->elevationModel()->getTexture());

                        stateSet->removeUniform(binding->matrixName());

                        stateSet->addUniform(_context->getOrCreateMatrixUniform(
                            binding->matrixName(),
                            osg::Matrixf::identity()));
                    }
                }

                // Insert the normal texture and an identity matrix:
                if (_model->normalModel().valid() && _model->normalModel()->getTexture())
                {
                    const SamplerBinding* binding = SamplerBinding::findUsage(bindings, SamplerBinding::NORMAL);
                    if (binding)
                    {
                        //TODO: if we subload the normal texture later on, we will need to change unref to false.
                        applyDefaultUnRefPolicy(_model->normalModel()->getTexture());

                        stateSet->setTextureAttribute(
                            binding->unit(),
                            _model->normalModel()->getTexture());

                        stateSet->removeUniform(binding->matrixName());

                        stateSet->addUniform(_context->getOrCreateMatrixUniform(
                            binding->matrixName(),
                            osg::Matrixf::identity()));
                    }
                }

                // Process any shared image layers, each of which should have its
                // own sampler binding point
                for (TerrainTileImageLayerModelVector::iterator i = _model->sharedLayers().begin();
                    i != _model->sharedLayers().end();
                    ++i)
                {
                    TerrainTileImageLayerModel* layerModel = i->get();
                    if (layerModel->getTexture())
                    {
                        const SamplerBinding* binding = SamplerBinding::findUID(bindings, layerModel->getImageLayer()->getUID());
                        if (binding)
                        {
                            applyDefaultUnRefPolicy(layerModel->getTexture());

                            stateSet->setTextureAttribute(
                                binding->unit(),
                                layerModel->getTexture());

                            stateSet->removeUniform(binding->matrixName());

                            stateSet->addUniform(_context->getOrCreateMatrixUniform(
                                binding->matrixName(),
                                osg::Matrixf::identity()));
                        }
                    }
                }
            }
#endif
        }
    }
}


// apply() runs in the update traversal and can safely alter the scene graph
void
LoadTileData::apply(const osg::FrameStamp* stamp)
{
    if ( _model.valid() )
    {
        osg::ref_ptr<TileNode> tilenode;
        if ( _tilenode.lock(tilenode) )
        {
            const RenderBindings& bindings      = _context->getRenderBindings();
            const SelectionInfo&  selectionInfo = _context->getSelectionInfo();
            const MapInfo&        mapInfo       = _context->getMapFrame().getMapInfo();

#if 0
            const SamplerBinding* color = SamplerBinding::findUsage(bindings, SamplerBinding::COLOR);

            // Find the mptexture, and then remove it since it was only in the state set for ICO compilation.
            osg::ref_ptr<MPTexture> mptex = dynamic_cast<MPTexture*>(
                getStateSet()->getTextureAttribute( color->unit(), osg::StateAttribute::TEXTURE) );

            if ( mptex.valid() )
            {
                getStateSet()->removeTextureAttribute( color->unit(), mptex.get() );
            }

            // Merge our prepped stateset into the live one.
            tilenode->mergeStateSet( getStateSet(), mptex.get(), bindings);
#endif
            // Merge the new data into the tile.
            tilenode->merge( _data, bindings );

            // Update existing inheritance matrices as necessary.
            UpdateInheritance update( _context, getChangeSet() );
            tilenode->accept( update );

            // Mark as complete. TODO: per-data requests will do something different.
            tilenode->setDirty( false );

            // Notify listeners that we've added a tile.
            _context->getEngine()->getTerrain()->notifyTileAdded( _key, tilenode );

            OE_DEBUG << LC << "apply " << _model->getKey().str() << "\n";

            // Delete the model immediately
            _model = 0L;
        }
        else
        {
            OE_DEBUG << LC << "LoadTileData failed; TileNode disappeared\n";
        }
    }
}
