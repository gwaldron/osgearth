/* osgEarth
 * Copyright 2008-2014 Pelican Mapping
 * MIT License
 */
#include "DrawTileCommand"
#include <osgDB/WriteFile>

using namespace osgEarth::REX;

#undef  LC
#define LC "[DrawTileCommand] "

bool
DrawTileCommand::apply(osg::RenderInfo& ri, void* implData) const
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

    // Morphing constants for this LOD
    if (pps._morphConstantsUL >= 0 && !pps._morphConstants.isSetTo(_morphConstants))
    {
        ext->glUniform2fv(pps._morphConstantsUL, 1, _morphConstants.ptr());
        pps._morphConstants = _morphConstants;
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
        for (s = SamplerBinding::COLOR; s <= SamplerBinding::COLOR_PARENT; ++s)
        {
            const Sampler& sampler = (*_colorSamplers)[s];
            SamplerState& samplerState = pps._samplerState._samplers[s];

            if (sampler._texture &&
                !samplerState._texture.isSetTo(sampler._texture))
            {
                if (!sampler._texture->dataLoaded())
                    return false;

                state.setActiveTextureUnit((*ds._bindings)[s].unit());
                sampler._texture->osgTexture()->apply(state);
                samplerState._texture = sampler._texture;
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
                    ext->glUniform1f(pps._parentTextureExistsUL, sampler._texture ? 1.0f : 0.0f);
                    pps._parentTextureExists = sampler._texture != nullptr;
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

            if (sampler._texture && !samplerState._texture.isSetTo(sampler._texture))
            {
                state.setActiveTextureUnit((*ds._bindings)[s].unit());
                sampler._texture->osgTexture()->apply(state);
                samplerState._texture = sampler._texture;
            }

            if (samplerState._matrixUL >= 0 && !samplerState._matrix.isSetTo(sampler._matrix))
            {
                ext->glUniformMatrix4fv(samplerState._matrixUL, 1, GL_FALSE, sampler._matrix.ptr());
                samplerState._matrix = sampler._matrix;
            }
        }
    }

    return true;
}

void
DrawTileCommand::debug(osg::RenderInfo& ri, void* implData) const
{
    DrawState& ds = *static_cast<DrawState*>(implData);
    ProgramState& pps = ds.getProgramState(ri);

    // Tile key encoding, if the uniform is required.
    OE_INFO << "\nKey " << _key->str() << std::endl;

    if (pps._tileKeyUL >= 0)
        OE_INFO << "  tileKey UL = " << pps._tileKeyUL << ", value = " << _keyValue[0] << ", " << _keyValue[1] << ", " << _keyValue[2] << ", " << _keyValue[3] << std::endl;

    if (pps._layerOrderUL >= 0)
        OE_INFO << "  layerOrder UL = " << pps._layerOrderUL << ", value = " << _layerOrder << std::endl;

    //if (pps._elevTexelCoeffUL >= 0)
    //    OE_INFO << "  _elevTexelCoeff UL = " << pps._elevTexelCoeffUL << ", value = " << (*pps._elevTexelCoeff)[0] << ", " << (*pps._elevTexelCoeff)[1] << std::endl;

    if (pps._morphConstantsUL >= 0 )
        OE_INFO << "  _morphConstantsUL UL = " << pps._morphConstantsUL << ", value = " << (*pps._morphConstants)[0] << ", " << (*pps._morphConstants)[1] << std::endl;
    
    OE_INFO << "  samplers:" << std::endl;
    unsigned s = 0;
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

    auto cid = GLUtils::getSharedContextID(*ri.getState());
    _geom->_ptype[cid] = _drawPatch ? GL_PATCHES : _geom->getDrawElements()->getMode();
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
