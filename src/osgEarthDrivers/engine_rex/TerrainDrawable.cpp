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
DrawTileCommand::draw(osg::RenderInfo& ri, DrawState& env) const
{
    osg::State& state = *ri.getState();

    //OE_INFO << LC << "      TILE: " << _geom << std::endl;
        
    // Tile key encoding, if the uniform is required.
    if (env._tileKeyUL >= 0 )
    {
        env._ext->glUniform4fv(env._tileKeyUL, 1, _keyValue.ptr());
    }

    // Elevation coefficients (can probably be terrain-wide)
    if (env._elevTexelCoeffUL >= 0 && !env._elevTexelCoeff.isSetTo(_elevTexelCoeff))
    {
        env._ext->glUniform2fv(env._elevTexelCoeffUL, 1, _elevTexelCoeff.ptr());
        env._elevTexelCoeff = _elevTexelCoeff;
    }

    // Morphing constants for this LOD
    if (env._morphConstantsUL >= 0 && !env._morphConstants.isSetTo(_morphConstants))
    {
        env._ext->glUniform2fv(env._morphConstantsUL, 1, _morphConstants.ptr());
        env._morphConstants = _morphConstants;
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

        SamplerState& samplerState = env._samplerState._samplers[s];

        if (sampler._texture.valid() && !samplerState._texture.isSetTo(sampler._texture))
        {
            state.setActiveTextureUnit((*env._bindings)[s].unit());
            sampler._texture->apply(state);
            samplerState._texture = sampler._texture.get();
        }

        if (samplerState._matrixUL >= 0 && !samplerState._matrix.isSetTo(sampler._matrix))
        {
            env._ext->glUniformMatrix4fv(samplerState._matrixUL, 1, GL_FALSE, sampler._matrix.ptr());
            samplerState._matrix = sampler._matrix;
        }

        // Need a special uniform for color parents.
        if (s == SamplerBinding::COLOR_PARENT)
        {
            if (env._parentTextureExistsUL >= 0 && !env._parentTextureExists.isSetTo(sampler._texture != 0L))
            {
                env._ext->glUniform1f(env._parentTextureExistsUL, sampler._texture.valid() ? 1.0f : 0.0f);
                env._parentTextureExists = sampler._texture.valid();
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


#if 0
#undef  LC
#define LC "[DrawLayerCommand] "

void
DrawLayerCommand::draw(osg::RenderInfo& ri, DrawState& env) const
{
    if (_tiles.empty())
        return;

    //OE_INFO << LC << "   LAYER; tiles = " << _tiles.size() << std::endl;

    //if (_layer && _layer->getStateSet())
    //{
    //    ri.getState()->apply(_layer->getStateSet());
    //    OE_NOTICE << "ype\n";
    //}

    if (_stateSet.valid())
    {
        //OE_WARN << " Applying stateset for layer\n";
        //ri.getState()->apply(_stateSet.get());
        ri.getState()->dirtyAllAttributes();
        ri.getState()->apply(_stateSet.get());
    }
    
    if (_layer)
    {
        if (env._layerUidUL >= 0)
            env._ext->glUniform1i(env._layerUidUL,      (GLint)_layer->getUID());
        if (env._layerOpacityUL >= 0)
            env._ext->glUniform1f(env._layerOpacityUL,  (GLfloat)_layer->getOpacity());
        if (env._layerMinRangeUL >= 0)
            env._ext->glUniform1f(env._layerMinRangeUL, (GLfloat)_layer->getMinVisibleRange());
        if (env._layerMaxRangeUL >= 0)
            env._ext->glUniform1f(env._layerMaxRangeUL, (GLfloat)_layer->getMaxVisibleRange());
    }
    else
    {
        if (env._layerUidUL >= 0)
            env._ext->glUniform1i(env._layerUidUL,      (GLint)-1);
        if (env._layerOpacityUL >= 0)
            env._ext->glUniform1f(env._layerOpacityUL,  (GLfloat)1.0f);
        if (env._layerMinRangeUL >= 0)
            env._ext->glUniform1f(env._layerMinRangeUL, (GLfloat)0.0f);
        if (env._layerMaxRangeUL >= 0)
            env._ext->glUniform1f(env._layerMaxRangeUL, (GLfloat)FLT_MAX);
    }

    for (DrawTileCommands::const_iterator tile = _tiles.begin(); tile != _tiles.end(); ++tile)
    {
        tile->draw(ri, env);
    }    

    if (_stateSet.valid())
    {
    //    ri.getState()->popStateSet();
    }
}
#endif


TerrainDrawable::TerrainDrawable()
{
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

void
TerrainDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    //OE_INFO << LC << "TERRAIN (camera=" << ri.getCurrentCamera()->getName() << ") layers = " << _layerList.size() << "\n";

    DrawState env;

    env._bindings = _bindings;

    env._ext = osg::GLExtensions::Get(ri.getContextID(), true);

    // Size the sampler states property:
    env._samplerState._samplers.resize(_bindings->size());


    draw(ri, env);

    // unbind local buffers when finished.
    env._ext->glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
    env._ext->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
}

void
TerrainDrawable::sortDrawCommands()
{
    //// finish preparing the drawable by sorting each layer's tiles
    //// for maximum state sharing.
    //for (DrawLayerCommandList::iterator layer = _layerList.begin(); layer != _layerList.end(); ++layer)
    //{
    //    layer->_tiles.sort();
    //}

    for (LayerDrawableList::iterator i = _layerList.begin(); i != _layerList.end(); ++i)
    {
        i->get()->_tiles.sort();
    }
}

void
TerrainDrawable::draw(osg::RenderInfo& ri, DrawState& env) const
{
    //unsigned order = 0;

    ////if (getStateSet())
    ////{
    ////    ri.getState()->apply(getStateSet());
    ////}

    //for (DrawLayerCommandList::const_iterator layer = _layerList.begin();
    //    layer != _layerList.end();
    //    ++layer, ++order)
    //{
    //    if (env._layerOrderUL >= 0 && !env._layerOrder.isSetTo(order))
    //    {
    //        env._ext->glUniform1i(env._layerOrderUL, (GLint)order);
    //        env._layerOrder = order;
    //    }

    //    layer->draw(ri, env);
    //}
}

void
TerrainDrawable::setup(const MapFrame& frame, const RenderBindings& bindings)
{
    _drawState = new DrawState();
    _drawState->_bindings = &bindings;

    //addLayer(0L);

    for (ImageLayerVector::const_iterator i = frame.imageLayers().begin();
        i != frame.imageLayers().end();
        ++i)
    {
        LayerDrawable* ld = addLayer(i->get());
        ld->setStateSet(i->get()->getStateSet());
    }

    if (layers().empty())
    {
        addLayer(0L);
    }

    _bindings = &bindings;
}

LayerDrawable*
TerrainDrawable::addLayer(const ImageLayer* imageLayer)
{
    UID uid = imageLayer ? imageLayer->getUID() : -1;
    LayerDrawable* ld = new LayerDrawable();
    _layerList.push_back( ld );
    _layerMap[uid] = ld;
    ld->_layer = imageLayer;
    ld->_order = _layerList.size()-1;
    ld->_env = _drawState.get();
    return ld;
}

void
TerrainDrawable::compileGLObjects(osg::RenderInfo& renderInfo) const
{
    //for (DrawLayerCommandList::const_iterator layer = _layerList.begin(); layer != _layerList.end(); ++layer)
    //{
    //    for (DrawTileCommands::const_iterator tile = layer->_tiles.begin(); tile != layer->_tiles.end(); ++tile)
    //    {
    //        tile->_geom->compileGLObjects(renderInfo);
    //    }
    //}
}

LayerDrawable::LayerDrawable() :
_layer(0L)
{
    setDataVariance(DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
}

void
LayerDrawable::drawImplementation(osg::RenderInfo& ri) const
{    
    DrawState& env = *_env;

    if (!env._stateInitialized)
    {
        env.initialize(ri);
    }
    else
    {
        env.refreshUniformLocations(ri);
    }

    if (_layer)
    {
        if (env._layerUidUL >= 0)
            env._ext->glUniform1i(env._layerUidUL,      (GLint)_layer->getUID());
        if (env._layerOpacityUL >= 0)
            env._ext->glUniform1f(env._layerOpacityUL,  (GLfloat)_layer->getOpacity());
        if (env._layerMinRangeUL >= 0)
            env._ext->glUniform1f(env._layerMinRangeUL, (GLfloat)_layer->getMinVisibleRange());
        if (env._layerMaxRangeUL >= 0)
            env._ext->glUniform1f(env._layerMaxRangeUL, (GLfloat)_layer->getMaxVisibleRange());
    }
    else
    {
        if (env._layerUidUL >= 0)
            env._ext->glUniform1i(env._layerUidUL,      (GLint)-1);
        if (env._layerOpacityUL >= 0)
            env._ext->glUniform1f(env._layerOpacityUL,  (GLfloat)1.0f);
        if (env._layerMinRangeUL >= 0)
            env._ext->glUniform1f(env._layerMinRangeUL, (GLfloat)0.0f);
        if (env._layerMaxRangeUL >= 0)
            env._ext->glUniform1f(env._layerMaxRangeUL, (GLfloat)FLT_MAX);
    }

    for (DrawTileCommands::const_iterator tile = _tiles.begin(); tile != _tiles.end(); ++tile)
    {
        tile->draw(ri, env);
    }
}
