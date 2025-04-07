/* osgEarth
 * Copyright 2008-2014 Pelican Mapping
 * MIT License
 */
#include "DrawState"

using namespace osgEarth::REX;

#undef  LC
#define LC "[DrawState] "

DrawState::Ptr
DrawState::create()
{
    return std::make_shared<DrawState>();
}

void
ProgramState::init(
    const osg::Program::PerContextProgram* pcp,
    const RenderBindings* bindings)
{
    _pcp = pcp;

    // Size the sampler states property:
    _samplerState._samplers.resize(bindings->size());

    // for each sampler binding, initialize its state tracking structure
    // and resolve its matrix uniform location:
    for (unsigned i = 0; i < bindings->size(); ++i)
    {
        const SamplerBinding& binding = (*bindings)[i];
        _samplerState._samplers[i]._name = binding.samplerName();
        if (_pcp)
        {
            _samplerState._samplers[i]._matrixUL = _pcp->getUniformLocation(
                osg::Uniform::getNameID(binding.matrixName()));
        }
    }

    // resolve all the other uniform locations:
    if (_pcp)
    {
        _tileKeyUL = _pcp->getUniformLocation(osg::Uniform::getNameID("oe_tile_key_u"));
        _parentTextureExistsUL = _pcp->getUniformLocation(osg::Uniform::getNameID("oe_layer_texParentExists"));
        _layerUidUL = _pcp->getUniformLocation(osg::Uniform::getNameID("oe_layer_uid"));
        _layerOrderUL = _pcp->getUniformLocation(osg::Uniform::getNameID("oe_layer_order"));
        _morphConstantsUL = _pcp->getUniformLocation(osg::Uniform::getNameID("oe_tile_morph"));
    }

    // Reset all optional states
    reset();
}

void
ProgramState::reset()
{
    //_elevTexelCoeff.clear();
    _morphConstants.clear();
    _parentTextureExists.clear();
    _layerOrder.clear();
    _samplerState.clear();
}
