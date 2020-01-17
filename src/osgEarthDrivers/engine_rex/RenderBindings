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
#ifndef OSGEARTH_DRIVERS_REX_RENDER_BINDINGS_H
#define OSGEARTH_DRIVERS_REX_RENDER_BINDINGS_H 1

#include "Common"
#include <osgEarth/Common>
#include <osgEarth/optional>
#include <osgEarth/Containers>
#include <osg/Texture>
#include <osg/Matrix>
#include <vector>
#include <map>

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    /**
     * Defines the usage information for a single texture sampler.
     */
    class SamplerBinding
    {
    public:
        enum Usage
        {
            COLOR         = 0,
            COLOR_PARENT  = 1,
            ELEVATION     = 2,
            NORMAL        = 3,
            COVERAGE      = 4,
            SHARED        = 5   // non-core shared layers start at this index
        };

    public:
        SamplerBinding() : _unit(-1) { }

        /** Optional UID of the source (usually a layer) to which this binding applies */
        optional<osgEarth::UID>& sourceUID()             { return _sourceUID; }
        const optional<osgEarth::UID>& sourceUID() const { return _sourceUID; }

        /** Optional usage hint */
        optional<Usage>& usage()             { return _usage; }
        const optional<Usage>& usage() const { return _usage; }

        /** Texture image unit of the sampler */
        int& unit() { return _unit; }
        const int& unit() const { return _unit; }

        /** Uniform name of the sampler */
        std::string& samplerName() { return _samplerName; }
        const std::string& samplerName() const { return _samplerName; }

        /** Uniform name of the sampler matrix */
        std::string& matrixName() { return _matrixName; }
        const std::string& matrixName() const { return _matrixName; }

    public:
        /** True if this binding is bound to a teture image unit and therefore active. */
        bool isActive() const { return _unit >= 0; }

    private:
        optional<UID>   _sourceUID;
        optional<Usage> _usage;
        int             _unit;
        std::string     _samplerName;
        std::string     _matrixName;
    };

    /**
     * Array of render bindings. This array is always indexed by the 
     * SamplerBinding::USAGE itself. So for example, RenderBindings[0] is always
     * the COLOR usage, [2] is the ELEVATION usage, etc. Shared layer bindings
     * (i.e., custom samplers) start at index SHARED and go up from there.
     */
    typedef AutoArray<SamplerBinding> RenderBindings;

} } } // namespace osgEarth::Drivers::RexTerrainEngine

#endif // OSGEARTH_DRIVERS_REX_RENDER_BINDINGS_H
