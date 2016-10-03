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
#include "TerrainDrawable"
#include "TileNode"
#include "SurfaceNode"

using namespace osgEarth::Drivers::RexTerrainEngine;

#undef  LC
#define LC "[DrawTileCommand] "


void
DrawTileCommand::draw(osg::RenderInfo& ri, DrawState& ds) const
{
    osg::State& state = *ri.getState();

    //OE_INFO << LC << "      TILE: " << _geom << std::endl;
        
    // Tile key encoding, if the uniform is required.
    if (ds._tileKeyUL >= 0 )
    {
        ds._ext->glUniform4fv(ds._tileKeyUL, 1, _keyValue.ptr());
    }

    // Elevation coefficients (can probably be terrain-wide)
    if (ds._elevTexelCoeffUL >= 0 && !ds._elevTexelCoeff.isSetTo(_elevTexelCoeff))
    {
        ds._ext->glUniform2fv(ds._elevTexelCoeffUL, 1, _elevTexelCoeff.ptr());
        ds._elevTexelCoeff = _elevTexelCoeff;
    }

    // Morphing constants for this LOD
    if (ds._morphConstantsUL >= 0 && !ds._morphConstants.isSetTo(_morphConstants))
    {
        ds._ext->glUniform2fv(ds._morphConstantsUL, 1, _morphConstants.ptr());
        ds._morphConstants = _morphConstants;
    }

    // MVM for this tile:
    state.applyModelViewMatrix(_modelViewMatrix);
    
    // MVM uniforms for GL3 core:
    if (state.getUseModelViewAndProjectionUniforms())
    {
        state.applyModelViewAndProjectionUniformsIfRequired();
    }

    // Apply samplers for this tile draw:
    for(unsigned s=0; s<_pass->_samplers.size(); ++s)
    {
        const Sampler& sampler = _pass->_samplers[s];

        SamplerState& samplerState = ds._samplerState._samplers[s];

        if (sampler._texture.valid() && !samplerState._texture.isSetTo(sampler._texture))
        {
            state.setActiveTextureUnit((*ds._bindings)[s].unit());
            sampler._texture->apply(state);
            samplerState._texture = sampler._texture.get();
        }

        if (samplerState._matrixUL >= 0 && !samplerState._matrix.isSetTo(sampler._matrix))
        {
            ds._ext->glUniformMatrix4fv(samplerState._matrixUL, 1, GL_FALSE, sampler._matrix.ptr());
            samplerState._matrix = sampler._matrix;
        }

        // Need a special uniform for color parents.
        if (s == SamplerBinding::COLOR_PARENT)
        {
            if (ds._parentTextureExistsUL >= 0 && !ds._parentTextureExists.isSetTo(sampler._texture != 0L))
            {
                ds._ext->glUniform1f(ds._parentTextureExistsUL, sampler._texture.valid() ? 1.0f : 0.0f);
                ds._parentTextureExists = sampler._texture.valid();
            }
        }
    }


    // Set up the vertex arrays:
    _geom->drawVertexArraysImplementation(ri);

    for (unsigned i = 0; i < _geom->getNumPrimitiveSets(); ++i)
    {
        _geom->getPrimitiveSet(i)->draw(*ri.getState(), true);
    }
}


void
TerrainRenderData::sortDrawCommands()
{
    for (LayerDrawableList::iterator i = _layerList.begin(); i != _layerList.end(); ++i)
    {
        i->get()->_tiles.sort();
    }
}

void
TerrainRenderData::setup(const MapFrame& frame, const RenderBindings& bindings)
{
    _drawState = new DrawState();
    _drawState->_bindings = &bindings;

    for (ImageLayerVector::const_iterator i = frame.imageLayers().begin();
        i != frame.imageLayers().end();
        ++i)
    {
        const ImageLayer* imageLayer = i->get();
        if (imageLayer->getEnabled())
        {
            LayerDrawable* ld = addLayer(i->get());

            // someday, replace this with the stateset from an ImageLayerRenderer..?
            ld->setStateSet(imageLayer->getStateSet());
        }
    }

    if (layers().empty())
    {
        addLayer(0L);
    }

    // The last layer needs to clear out the OSG state,
    layers().back()->_clearOsgState = true;

    _bindings = &bindings;
}

LayerDrawable*
TerrainRenderData::addLayer(const ImageLayer* imageLayer)
{
    UID uid = imageLayer ? imageLayer->getUID() : -1;
    LayerDrawable* ld = new LayerDrawable();
    _layerList.push_back( ld );
    _layerMap[uid] = ld;
    ld->_layer = imageLayer;
    ld->_order = _layerList.size()-1;
    ld->_drawState = _drawState.get();
    return ld;
}

LayerDrawable::LayerDrawable() :
_order(0),
_layer(0L),
_terrain(0L),
_clearOsgState(false)
{
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

void
LayerDrawable::drawImplementation(osg::RenderInfo& ri) const
{    
    DrawState& ds = *_drawState;

    if (!ds._stateInitialized)
    {
        ds.initialize(ri);
    }
    else
    {
        ds.refreshUniformLocations(ri);
    }

    if (ds._layerOrderUL >= 0)
    {
        ds._ext->glUniform1i(ds._layerOrderUL, (GLint)_order);
    }

    if (_layer)
    {
        if (ds._layerUidUL >= 0)
            ds._ext->glUniform1i(ds._layerUidUL,      (GLint)_layer->getUID());
        if (ds._layerOpacityUL >= 0)
            ds._ext->glUniform1f(ds._layerOpacityUL,  (GLfloat)_layer->getOpacity());
        if (ds._layerMinRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMinRangeUL, (GLfloat)_layer->getMinVisibleRange());
        if (ds._layerMaxRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMaxRangeUL, (GLfloat)_layer->getMaxVisibleRange());
    }
    else
    {
        if (ds._layerUidUL >= 0)
            ds._ext->glUniform1i(ds._layerUidUL,      (GLint)-1);
        if (ds._layerOpacityUL >= 0)
            ds._ext->glUniform1f(ds._layerOpacityUL,  (GLfloat)1.0f);
        if (ds._layerMinRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMinRangeUL, (GLfloat)0.0f);
        if (ds._layerMaxRangeUL >= 0)
            ds._ext->glUniform1f(ds._layerMaxRangeUL, (GLfloat)FLT_MAX);
    }

    for (DrawTileCommands::const_iterator tile = _tiles.begin(); tile != _tiles.end(); ++tile)
    {
        tile->draw(ri, ds);
    }

    // If set, dirty all OSG state to prevent any leakage - this is sometimes
    // necessary when doing custom OpenGL within a Drawable.
    if (_clearOsgState)
    {
        ri.getState()->dirtyAllAttributes();
        ri.getState()->dirtyAllModes();
        ri.getState()->dirtyAllVertexArrays();
        
        // unbind local buffers when finished.
        ds._ext->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
        ds._ext->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
    }
}
