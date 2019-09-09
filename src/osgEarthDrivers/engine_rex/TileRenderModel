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
#ifndef OSGEARTH_REX_TILE_RENDER_MODEL
#define OSGEARTH_REX_TILE_RENDER_MODEL 1

#include "Common"
#include "RenderBindings"
#include <osgEarth/Common>
#include <osgEarth/Containers> // for AutoArray
#include <osgEarth/Layer>
#include <osgEarth/ImageLayer>
#include <osgEarth/PatchLayer>
#include <osg/Texture>
#include <osg/Matrix>
#include <vector>

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    /**
     * A single texture and its matrix. This corresponds to a 
     * single SamplerBinding (above).
     */
    struct Sampler
    {
        osg::ref_ptr<osg::Texture> _texture;
        osg::Matrixf _matrix;
    };
    typedef AutoArray<Sampler> Samplers;

    /**
     * Samplers (one per RenderBinding) specific to one rendering pass of a tile.
     * Typically this is just the color and color parent samplers.
     */
    struct RenderingPass
    {
    public:
        RenderingPass() :
            _sourceUID(-1),
            _samplers(SamplerBinding::COLOR_PARENT+1),
            _visibleLayer(0L),
            _terrainLayer(0L)
            { }
        
        UID sourceUID() const { return _sourceUID; }
        Samplers& samplers() { return _samplers; }
        const Samplers& samplers() const  { return _samplers; }
        const Layer* layer() const { return _layer.get(); }
        const VisibleLayer* visibleLayer() const { return _visibleLayer; }
        const TerrainLayer* terrainLayer() const { return _terrainLayer; }

        void releaseGLObjects(osg::State* state) const
        {
            for (unsigned s = 0; s<_samplers.size(); ++s)
                if (_samplers[s]._texture.valid() && _samplers[s]._matrix.isIdentity())
                    _samplers[s]._texture->releaseGLObjects(state);
        }

        void resizeGLObjectBuffers(unsigned size)
        {
            for (unsigned s = 0; s<_samplers.size(); ++s)
                if (_samplers[s]._texture.valid() && _samplers[s]._matrix.isIdentity())
                    _samplers[s]._texture->resizeGLObjectBuffers(size);
        }

        void setLayer(const Layer* layer) {
            _layer = layer;
            if (layer) {
                _visibleLayer = dynamic_cast<const VisibleLayer*>(layer);
                _terrainLayer = dynamic_cast<const TerrainLayer*>(layer);
                _sourceUID = layer->getUID();
            }
        }

    private:
        /** UID of the layer responsible for this rendering pass (usually an ImageLayer) */
        UID _sourceUID;

        /** Samplers specific to this rendering pass (COLOR, COLOR_PARENT) */
        Samplers _samplers;

        /** Layer respsonible for this rendering pass */
        osg::ref_ptr<const Layer> _layer;

        /** VisibleLayer responsible for this rendering pass (is _layer is a VisibleLayer) */
        const VisibleLayer* _visibleLayer;
        
        /** VisibleLayer responsible for this rendering pass (is _layer is a TerrainLayer) */
        const TerrainLayer* _terrainLayer;


    };

    /**
     * Unordered collection of rendering passes.
     */
    typedef std::vector<RenderingPass> RenderingPasses;

    /**
     * Everything necessary to render a single terrain tile.
     * REX renders the terrain in multiple passes, one pass for each visible layer.
     */
    struct TileRenderModel
    {
        /** Samplers that are bound for every rendering pass (elevation, normal map, etc.) */
        Samplers _sharedSamplers;

        /** Samplers bound for each visible layer (color) */
        RenderingPasses _passes;

        /** Add a new rendering pass to the end of the list. */
        RenderingPass& addPass()
        {
            _passes.resize(_passes.size()+1);
            return _passes.back();
        }

        /** Look up a rendering pass by the corresponding layer ID */
        const RenderingPass* getPass(UID uid) const
        {
            for (unsigned i = 0; i < _passes.size(); ++i) {
                if (_passes[i].sourceUID() == uid)
                    return &_passes[i];
            }
            return 0L;
        }

        /** Look up a rendering pass by the corresponding layer ID */
        RenderingPass* getPass(UID uid)
        {
            for (unsigned i = 0; i < _passes.size(); ++i) {
                if (_passes[i].sourceUID() == uid)
                    return &_passes[i];
            }
            return 0L;
        }

        /** Deallocate GPU objects associated with this model */
        void releaseGLObjects(osg::State* state) const
        {
            for (unsigned s = 0; s<_sharedSamplers.size(); ++s)
                if (_sharedSamplers[s]._texture.valid() && _sharedSamplers[s]._matrix.isIdentity())
                    _sharedSamplers[s]._texture->releaseGLObjects(state);

            for (unsigned p = 0; p<_passes.size(); ++p)
                _passes[p].releaseGLObjects(state);
        }

        /** Resize GL buffers associated with this model */
        void resizeGLObjectBuffers(unsigned size)
        {
            for (unsigned s = 0; s<_sharedSamplers.size(); ++s)
                if (_sharedSamplers[s]._texture.valid() && _sharedSamplers[s]._matrix.isIdentity())
                    _sharedSamplers[s]._texture->resizeGLObjectBuffers(size);

            for (unsigned p = 0; p<_passes.size(); ++p)
                _passes[p].resizeGLObjectBuffers(size);
        }
    };

} } } // namespace osgEarth::Drivers::RexTerrainEngine

#endif // OSGEARTH_REX_TILE_RENDER_MODEL
