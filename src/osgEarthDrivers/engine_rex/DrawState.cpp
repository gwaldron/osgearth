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
#include "DrawState"

using namespace osgEarth::Drivers::RexTerrainEngine;

#undef  LC
#define LC "[DrawState] "

void
PerContextDrawState::refresh(osg::RenderInfo& ri, const RenderBindings* bindings)
{
    // Establish a GL Extensions handle:
    if (!_ext.valid())
    {
        _ext = osg::GLExtensions::Get(ri.getContextID(), true);
    }

    // Size the sampler states property:
    if (_samplerState._samplers.size() < bindings->size())
    {
        _samplerState._samplers.resize(bindings->size());
    }

    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (pcp && (pcp != _pcp))
    {
        // Reset all sampler matrix states since their uniform locations are going to change.
        //_runningLayerDrawOrder = 0;
        _elevTexelCoeff.clear();
        _morphConstants.clear();
        _parentTextureExists.clear();
        _samplerState.clear();

        // for each sampler binding, initialize its state tracking structure 
        // and resolve its matrix uniform location:
        for (unsigned i = 0; i < bindings->size(); ++i)
        {
            const SamplerBinding& binding = (*bindings)[i];
            _samplerState._samplers[i]._matrixUL = pcp->getUniformLocation(osg::Uniform::getNameID(binding.matrixName()));
        }

        // resolve all the other uniform locations:
        _tileKeyUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_tile_key"));
        _elevTexelCoeffUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_tile_elevTexelCoeff"));
        _parentTextureExistsUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_layer_texParentExists"));
        _layerUidUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_layer_uid"));
        _layerOrderUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_layer_order"));
        _morphConstantsUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_tile_morph"));
    }

    _pcp = pcp;
}

void
PerContextDrawState::clear()
{
    _samplerState.clear();
    _pcp = 0L;
}
