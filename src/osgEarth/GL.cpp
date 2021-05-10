/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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

#include <osgEarth/GL>
#include <osg/GLExtensions>

static osg::ref_ptr< osg::GLExtensions > s_glext;


void osgEarth::initGLExtensions(unsigned int contextID)
{
    s_glext = osg::GLExtensions::Get(contextID, true);
}

void glGenQueries(GLsizei n, GLuint *ids)
{
    s_glext->glGenQueries(n, ids);
}

void glGetInteger64v(GLenum pname, GLint64 *data)
{
    s_glext->glGetInteger64v(pname, data);
}

void glGetQueryiv(GLenum target, GLenum pname, GLint *params)
{
    s_glext->glGetQueryiv(target, pname, params);
}

void glGetQueryObjectiv(GLuint id, GLenum pname, GLint *params)
{
    s_glext->glGetQueryObjectiv(id, pname, params);
}

void glGetQueryObjectui64v(GLuint id, GLenum pname, GLuint64 *params)
{
    s_glext->glGetQueryObjectui64v(id, pname, params);
}

void glQueryCounter(GLuint id, GLenum target)
{
    s_glext->glQueryCounter(id, target);
}