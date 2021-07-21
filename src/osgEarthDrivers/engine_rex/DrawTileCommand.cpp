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
#include "DrawTileCommand"
#include <osgDB/WriteFile>

using namespace osgEarth::REX;

#undef  LC
#define LC "[DrawTileCommand] "

void
DrawTileCommand::apply(
    osg::RenderInfo& ri,
    void* implData) const
{
    DrawState& ds = *static_cast<DrawState*>(implData);
    ProgramState& pps = ds.getProgramState(ri);

    osg::State& state = *ri.getState();
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    // Tile key encoding, if the uniform is required.
    if (pps._tileKeyUL >= 0)
    {
        ext->glUniform4fv(pps._tileKeyUL, 1, _keyValue.ptr());
    }

    // Apply the layer draw order for this tile so we can blend correctly:
    if (pps._layerOrderUL >= 0 && !pps._layerOrder.isSetTo(_layerOrder))
    {
        ext->glUniform1i(pps._layerOrderUL, (GLint)_layerOrder);
        pps._layerOrder = _layerOrder;
    }

    // Elevation coefficients (can probably be terrain-wide)
    if (pps._elevTexelCoeffUL >= 0 && !pps._elevTexelCoeff.isSetTo(_elevTexelCoeff))
    {
        ext->glUniform2fv(pps._elevTexelCoeffUL, 1, _elevTexelCoeff.ptr());
        pps._elevTexelCoeff = _elevTexelCoeff;
    }

    // Morphing constants for this LOD
    if (pps._morphConstantsUL >= 0 && !pps._morphConstants.isSetTo(_morphConstants))
    {
        ext->glUniform2fv(pps._morphConstantsUL, 1, _morphConstants.ptr());
        pps._morphConstants = _morphConstants;
    }

    // MVM for this tile:
    state.applyModelViewMatrix(_modelViewMatrix.get());

    // MVM uniforms for GL3 core:
    if (state.getUseModelViewAndProjectionUniforms())
    {
        state.applyModelViewAndProjectionUniformsIfRequired();
    }

    // Apply samplers for this tile draw:
    unsigned s = 0;

    if (_colorSamplers)
    {
        for (s = SamplerBinding::COLOR; s <= SamplerBinding::COLOR_PARENT; ++s)
        {
            const Sampler& sampler = (*_colorSamplers)[s];
            SamplerState& samplerState = pps._samplerState._samplers[s];

            if (sampler._texture.valid() && !samplerState._texture.isSetTo(sampler._texture.get()))
            {
                // test for a "placeholder" texture, i.e. a texture whose image
                // is not yet available -- if encountered, bail and render nothing.
                if (sampler._texture->getNumImages() > 0 &&
                    sampler._texture->getImage(0) != nullptr &&
                    sampler._texture->getImage(0)->valid() == false)
                {
                    return;
                }

                state.setActiveTextureUnit((*ds._bindings)[s].unit());
                sampler._texture->apply(state);
                samplerState._texture = sampler._texture.get();
            }

            if (samplerState._matrixUL >= 0 && !samplerState._matrix.isSetTo(sampler._matrix))
            {
                ext->glUniformMatrix4fv(samplerState._matrixUL, 1, GL_FALSE, sampler._matrix.ptr());
                samplerState._matrix = sampler._matrix;
            }

            // Need a special uniform for color parents.
            if (s == SamplerBinding::COLOR_PARENT)
            {
                if (pps._parentTextureExistsUL >= 0 && !pps._parentTextureExists.isSetTo(sampler._texture.get() != 0L))
                {
                    ext->glUniform1f(pps._parentTextureExistsUL, sampler._texture.valid() ? 1.0f : 0.0f);
                    pps._parentTextureExists = sampler._texture.valid();
                }
            }
        }
    }

    if (_sharedSamplers)
    {
        for (; s < _sharedSamplers->size(); ++s)
        {
            const Sampler& sampler = (*_sharedSamplers)[s];
            SamplerState& samplerState = pps._samplerState._samplers[s];

            if (sampler._texture.valid() && !samplerState._texture.isSetTo(sampler._texture.get()))
            {
                state.setActiveTextureUnit((*ds._bindings)[s].unit());
                sampler._texture->apply(state);
                samplerState._texture = sampler._texture.get();
            }

            if (samplerState._matrixUL >= 0 && !samplerState._matrix.isSetTo(sampler._matrix))
            {
                ext->glUniformMatrix4fv(samplerState._matrixUL, 1, GL_FALSE, sampler._matrix.ptr());
                samplerState._matrix = sampler._matrix;
            }
        }
    }
}
void
DrawTileCommand::debug(
    osg::RenderInfo& ri,
    void* implData) const
{
    DrawState& ds = *static_cast<DrawState*>(implData);
    ProgramState& pps = ds.getProgramState(ri);

    // Tile key encoding, if the uniform is required.
    OE_INFO << "\nKey " << _key->str() << std::endl;

    if (pps._tileKeyUL >= 0)
        OE_INFO << "  tileKey UL = " << pps._tileKeyUL << ", value = " << _keyValue[0] << ", " << _keyValue[1] << ", " << _keyValue[2] << ", " << _keyValue[3] << std::endl;

    if (pps._layerOrderUL >= 0)
        OE_INFO << "  layerOrder UL = " << pps._layerOrderUL << ", value = " << _layerOrder << std::endl;

    if (pps._elevTexelCoeffUL >= 0)
        OE_INFO << "  _elevTexelCoeff UL = " << pps._elevTexelCoeffUL << ", value = " << (*pps._elevTexelCoeff)[0] << ", " << (*pps._elevTexelCoeff)[1] << std::endl;

    if (pps._morphConstantsUL >= 0 )
        OE_INFO << "  _morphConstantsUL UL = " << pps._morphConstantsUL << ", value = " << (*pps._morphConstants)[0] << ", " << (*pps._elevTexelCoeff)[1] << std::endl;
    
    OE_INFO << "  samplers:" << std::endl;
    int s = 0;
    if (_sharedSamplers)
    {
        for (; s < _sharedSamplers->size(); ++s)
        {
            const Sampler& sampler = (*_sharedSamplers)[s];
            SamplerState& samplerState = pps._samplerState._samplers[s];

            //TODO: Check the _matrixUL first, and if it's not set, don't apply the texture
            // because it's not used either.

            if (samplerState._matrixUL >= 0)
            {
                OE_INFO << "    name = " << samplerState._name << ", mUL = " << samplerState._matrixUL
                    << ", scale = " << sampler._matrix(0,0) << std::endl;

#if 0
                if (samplerState._texture.isSet())
                {
                    osg::ref_ptr<osg::Image> c = osg::clone(
                        samplerState._texture.get()->getImage(0),
                        osg::CopyOp::DEEP_COPY_ALL);

                    ImageUtils::PixelReader r(c.get());
                    ImageUtils::PixelWriter w(c.get());
                    ImageUtils::ImageIterator i(r);
                    i.forEachPixel([&]() {
                        osg::Vec4f p;
                        r(p, i.s(), i.t());
                        p.a() = 1.0f;
                        w(p, i.s(), i.t());
                        });

                    osgDB::writeImageFile(
                        *c.get(),
                        Stringify() << "out/" << samplerState._name
                        << "." << ri.getState()->getFrameStamp()->getFrameNumber()
                        << ".png");
                }
#endif
            }
        }
    }
}

void
DrawTileCommand::draw(osg::RenderInfo& ri) const
{
    OE_SOFT_ASSERT_AND_RETURN(_geom.valid(), void());

    _geom->_ptype[ri.getContextID()] = _drawPatch ? GL_PATCHES : _geom->getDrawElements()->getMode();
    _geom->draw(ri);
}

#if 0
void
DrawTileCommand::visit(osg::RenderInfo& ri) const
{
    if (_drawCallback)
    {
        PatchLayer::DrawContext tileData;

        tileData._key = _key;
        tileData._revision = _tileRevision;
        //tileData._geomBBox = &_geom->getBoundingBox();
        tileData._tileBBox = &_tile->getBoundingBox();
        tileData._modelViewMatrix = _modelViewMatrix.get();
        _drawCallback->visitTile(ri, tileData);
    }
}
#endif

void DrawTileCommand::accept(osg::PrimitiveFunctor& functor) const
{
    if (_geom.valid() && _geom->supports(functor))
    {
        _geom->accept(functor);
    }
}

void DrawTileCommand::accept(osg::PrimitiveIndexFunctor& functor) const
{
    if (_geom.valid() && _geom->supports(functor))
    {
        _geom->accept(functor);
    }
}
