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
#ifndef OSGEARTH_REX_TERRAIN_DRAW_STATE_H
#define OSGEARTH_REX_TERRAIN_DRAW_STATE_H 1

#include "RenderBindings"

#include <osg/RenderInfo>
#include <osg/GLExtensions>
#include <osg/StateSet>
#include <osg/Program>

#include <vector>

using namespace osgEarth;

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    /**
     * Tracks the state of a single sampler through the draw process,
     * to prevent redundant OpenGL texture binding and matrix uniform sets.
     */
    struct SamplerState
    {
        SamplerState() : _matrixUL(-1) { }
        optional<osg::Texture*> _texture;    // Texture currently bound
        optional<osg::Matrixf> _matrix;      // Matrix that is currently set
        GLint _matrixUL;                     // Matrix uniform location

        void clear() {
            _texture.clear();
            _matrix.clear();
        }

        void clearUniformData() {
            _matrix.clear();
            _matrixUL = -1;
        }
    };

    /**
     * Tracks the state of all samplers used in render a tile,
     * to prevent redundant OpenGL binds.
     */
    struct TileSamplerState
    {
        std::vector<SamplerState> _samplers;

        void clear() {
            for (unsigned i = 0; i<_samplers.size(); ++i)
                _samplers[i].clear();
        }

        void clearUniformData() {
            for (unsigned i = 0; i<_samplers.size(); ++i)
                _samplers[i].clearUniformData();
        }
    };

    struct PerContextDrawState
    {
        // uniform locations
        GLint _tileKeyUL;
        GLint _parentTextureExistsUL;
        GLint _layerUidUL;
        GLint _layerOrderUL;
        GLint _elevTexelCoeffUL;
        GLint _morphConstantsUL;

        optional<osg::Vec2f> _elevTexelCoeff;
        optional<osg::Vec2f> _morphConstants;
        optional<bool>       _parentTextureExists;
        optional<int>        _layerOrder;

        const osg::Program::PerContextProgram* _pcp;

        osg::ref_ptr<osg::GLExtensions> _ext;

        TileSamplerState _samplerState;

        PerContextDrawState() :
            _tileKeyUL(-1),
            _parentTextureExistsUL(-1),
            _layerUidUL(-1),
            _layerOrderUL(-1),
            _elevTexelCoeffUL(-1),
            _morphConstantsUL(-1),
            _ext(0L),
            _pcp(0L)
        {
            //nop
        }

        // Ensures that the Uniform Locations in the DrawState correspond to
        // the currently applied program object.
        void refresh(osg::RenderInfo& ri, const RenderBindings* bindings);

        // Clears all saved state.
        void clear();
    };

    /**
     * Tracks the state of terrain drawing settings in a single frame,
     * to prevent redundant OpenGL calls.
     */
    struct DrawState : public osg::Referenced
    {
        unsigned _frame;

        const RenderBindings* _bindings;

        osg::BoundingSphere _bs;
        osg::BoundingBox    _box;

        osg::buffered_object<PerContextDrawState> _pcds;

        DrawState() :
            _frame(0u),
            _bindings(0L)
        {
            //nop
            _pcds.resize(64);
        }

        PerContextDrawState& getPCDS(unsigned contextID) { return _pcds[contextID]; }
    };

} } } // namespace 

#endif // OSGEARTH_REX_TERRAIN_DRAW_STATE_H
