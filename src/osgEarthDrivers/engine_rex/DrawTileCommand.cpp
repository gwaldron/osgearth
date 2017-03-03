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
#include "DrawTileCommand"

using namespace osgEarth::Drivers::RexTerrainEngine;

#undef  LC
#define LC "[DrawTileCommand] "

void
DrawTileCommand::draw(osg::RenderInfo& ri, DrawState& dsMaster, osg::Referenced* layerData) const
{
    PerContextDrawState& ds = dsMaster.getPCDS(ri.getContextID());

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
    unsigned s = 0;

    if (_colorSamplers)
    {
        for (s = 0; s <= SamplerBinding::COLOR_PARENT; ++s)
        {
            const Sampler& sampler = (*_colorSamplers)[s];
            SamplerState& samplerState = ds._samplerState._samplers[s];

            if (sampler._texture.valid() && !samplerState._texture.isSetTo(sampler._texture))
            {
                state.setActiveTextureUnit((*dsMaster._bindings)[s].unit());
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
    }

    if (_sharedSamplers)
    {
        for (; s < _sharedSamplers->size(); ++s)
        {
            const Sampler& sampler = (*_sharedSamplers)[s];
            SamplerState& samplerState = ds._samplerState._samplers[s];

            if (sampler._texture.valid() && !samplerState._texture.isSetTo(sampler._texture))
            {
                state.setActiveTextureUnit((*dsMaster._bindings)[s].unit());
                sampler._texture->apply(state);
                samplerState._texture = sampler._texture.get();
            }

            if (samplerState._matrixUL >= 0 && !samplerState._matrix.isSetTo(sampler._matrix))
            {
                ds._ext->glUniformMatrix4fv(samplerState._matrixUL, 1, GL_FALSE, sampler._matrix.ptr());
                samplerState._matrix = sampler._matrix;
            }
        }
    }

    if (_drawCallback)
    {
        PatchLayer::DrawContext dc;

        //TODO: might not need any of this. review. -gw
        dc.colorTexture = _colorSamplers? (*_colorSamplers)[SamplerBinding::COLOR]._texture.get() : 0L;
        if (_sharedSamplers)
        {
            dc.elevationTexture = (*_sharedSamplers)[SamplerBinding::ELEVATION]._texture.get();
            dc.normalTexture    = (*_sharedSamplers)[SamplerBinding::NORMAL]._texture.get();
            dc.coverageTexture  = (*_sharedSamplers)[SamplerBinding::COVERAGE]._texture.get();
        }
        dc.key = &_key;
        dc.range = _range;
        _drawCallback->draw(ri, dc, layerData);

        // evaluate this.
        ds._samplerState.clear();
    }

    else
    // If there's a geometry, draw it now:
    if (_geom.valid())
    {
        GLenum ptype = _drawPatch ? GL_PATCHES : GL_TRIANGLES;

        _geom->render(ptype, ri);
#if 0
        // Set up the vertex arrays:
        _geom->drawVertexArraysImplementation(ri);

        if (_drawPatch)
        {
            for (unsigned i = 0; i < _geom->getNumPrimitiveSets(); ++i)
            {
                osg::DrawElementsUShort* de = static_cast<osg::DrawElementsUShort*>(_geom->getPrimitiveSet(i));
                osg::GLBufferObject* ebo = de->getOrCreateGLBufferObject(state.getContextID());
                state.bindElementBufferObject(ebo);
                if (ebo)
                    glDrawElements(GL_PATCHES, de->size(), GL_UNSIGNED_SHORT, (const GLvoid *)(ebo->getOffset(de->getBufferIndex())));
            }
        }

        else
        {
            if (_geom->referenceCount() <= 1)
            {
                OE_WARN << LC << "Big trouble\n";
                exit(-1);
            }

            for (unsigned i = 0; i < _geom->getNumPrimitiveSets(); ++i)
            {
                osg::PrimitiveSet* ps = _geom->getPrimitiveSet(i);
                if (ps)
                {
                    ps->draw(*ri.getState(), true);
                }
            }
        }
#endif
    }    
}
