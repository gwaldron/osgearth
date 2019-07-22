/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#ifndef OSGEARTH_SHADER_MERGER_H
#define OSGEARTH_SHADER_MERGER_H 1

#include <osgEarth/GLSLChunker>
#include <osg/Shader>

namespace osg {
    class Program;
}

namespace osgEarth
{
    /**
     * Merges multiple shader objects into one (per type) and installs 
     * them in a program. Some platforms (like GLES) only support one
     * shader object, neccesiting this function.
     */
    class OSGEARTH_EXPORT ShaderMerger
    {
    public:
        //! Adds a shader to the merge set
        void add(const osg::Shader* shader);

        //! Merges all shaders and installs them into the provided program.
        void merge(osg::Program* program);

    private:
        std::vector<const osg::Shader*> _shaders;
        std::set<osg::Shader::Type> _types;

        //! Merge shaders of shaders's type into shader, and returns
        //! the number of shaders merged.
        unsigned merge(osg::Shader* shader);
    };
}

#endif // OSGEARTH_SHADER_MERGER_H
