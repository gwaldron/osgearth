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
#include <osgEarth/ShaderMerger>
#include <osgEarth/GLSLChunker>
#include <osgEarth/StringUtils>
#include <osg/Program>
#include <sstream>

using namespace osgEarth;


void
ShaderMerger::add(const osg::Shader* shader)
{
    _shaders.push_back(shader);
    _types.insert(shader->getType());
}

void
ShaderMerger::merge(osg::Program* program)
{
    for(std::set<osg::Shader::Type>::const_iterator i = _types.begin(); i != _types.end(); ++i)
    {
        osg::ref_ptr<osg::Shader> shader = new osg::Shader(*i);
        if (merge(shader.get()) > 0u)
        {
            program->addShader(shader.get());
        }
    }
}

unsigned
ShaderMerger::merge(osg::Shader* output)
{
    std::set<std::string> unique;
    GLSLChunker::Chunks version;
    GLSLChunker::Chunks extensions;
    GLSLChunker::Chunks directives; // relocated directives, like extensions and pragmas (but not if's)
    GLSLChunker::Chunks statements; // relocated statements (ignoring if blocks)
    GLSLChunker::Chunks body;       // main body, including functions and if blocks

    unsigned count = 0u;

    for(std::vector<const osg::Shader*>::const_iterator i = _shaders.begin();
        i != _shaders.end();
        ++i)
    {
        const osg::Shader* shader = *i;
        if (shader->getType() == output->getType())
        {
            ++count;

            GLSLChunker chunker;
            GLSLChunker::Chunks chunks;
            chunker.read(shader->getShaderSource(), chunks);

            for(GLSLChunker::Chunks::const_iterator c = chunks.begin();
                c != chunks.end();
                ++c)
            {
                if (c->type == c->TYPE_DIRECTIVE && c->text.substr(0, 8) == "#version")
                {
                    if (version.empty())
                        version.push_back(*c);
                }
                else if (c->type == c->TYPE_DIRECTIVE && c->text.substr(0, 10) == "#extension")
                {
                    std::string normalized = trimAndCompress(c->text);
                    if (unique.find(normalized) == unique.end())
                    {
                        unique.insert(normalized);
                        extensions.push_back(*c);
                    }
                }
                else if (c->type == c->TYPE_DIRECTIVE && c->text.substr(0, 7) == "#pragma")
                {
                    if (c->tokens.size() >= 2 && c->tokens[1].substr(0,3) != "vp_")
                    {
                        std::string normalized = trimAndCompress(c->text);
                        if (unique.find(normalized) == unique.end())
                        {
                            unique.insert(normalized);
                            directives.push_back(*c);
                        }
                    }
                }
                else if (c->type == c->TYPE_STATEMENT)
                {
                    std::string normalized = trimAndCompress(c->text);
                    if (unique.find(normalized) == unique.end())
                    {
                        unique.insert(normalized);
                        statements.push_back(*c);
                    }
                }
                else if (c->type == c->TYPE_COMMENT)
                {
                    //nop - discard it for now
                }
                else
                {
                    body.push_back(*c);
                }
            }
        }
    }

    std::stringstream buf;

    if (version.empty() == false)
    {
#if defined(OSG_GLES3_AVAILABLE)
    // force version numbers for gles
    #if __ANDROID__
        buf << "#version 310 es" << std::endl;
    #else
        buf << "#version 300 es" << std::endl;
    #endif
#else
        buf << version[0].text << std::endl;
#endif
    }
    
    // android requires the following extension is defined
#if defined(OSG_GLES3_AVAILABLE)
    #if __ANDROID__
        buf << "#extension GL_EXT_shader_io_blocks : require" << std::endl;
    #endif
#endif

    for(GLSLChunker::Chunks::const_iterator i = extensions.begin(); 
        i != extensions.end();
        ++i)
    {
        buf << i->text << std::endl;
    }

    for(GLSLChunker::Chunks::const_iterator i = directives.begin(); 
        i != directives.end();
        ++i)
    {
        buf << i->text << std::endl;
    }

    for(GLSLChunker::Chunks::const_iterator i = statements.begin(); 
        i != statements.end();
        ++i)
    {
        buf << i->text << std::endl;
    }

    for(GLSLChunker::Chunks::const_iterator i = body.begin(); 
        i != body.end();
        ++i)
    {
        buf << i->text << std::endl;
    }

    output->setShaderSource( buf.str() );

    return count;
}
