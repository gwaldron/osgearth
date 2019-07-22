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
#ifndef OSGEARTH_NATIVE_PROGRAM_ADAPTER
#define OSGEARTH_NATIVE_PROGRAM_ADAPTER

#include <osg/State>
#include <osg/Uniform>
#include <osg/GL2Extensions>
#include <osg/Referenced>
#include <osg/MixinVector>
#include <osgEarth/Notify>
#include <osgEarth/Common>
#include <vector>

#undef  LC
#define LC "[NativeProgramAdapter] "

#define OE_LOCAL OE_DEBUG
//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

namespace osgEarth
{
    /**
     * Wraps a native glProgram handle so we can earch it form uniforms and
     * apply values to them.
     */
    class NativeProgramAdapter : public osg::Referenced
    {
    public:
        /** Create a program adapter under the current state that wraps the provided glProgram handle. */
        NativeProgramAdapter(const osg::State* state, GLint handle, const char* prefix, const std::string& name)
        {
            OE_LOCAL << LC << "Create adapter for glProgram " << name << " (handle=" << handle << ")" << std::endl;

            _handle = handle;
            _ext    = osg::GL2Extensions::Get( state->getContextID(), true );

            GLint   numUniforms = 0;
            GLsizei maxLen      = 0;

            _ext->glGetProgramiv( _handle, GL_ACTIVE_UNIFORMS,      &numUniforms );
            _ext->glGetProgramiv( _handle, GL_ACTIVE_UNIFORM_MAX_LENGTH, &maxLen );

            if ( (numUniforms > 0) && (maxLen > 1) )
            {
                GLint   size = 0;
                GLenum  type = 0;
                GLchar* name = new GLchar[maxLen];

                for( GLint i = 0; i < numUniforms; ++i )
                {
                    _ext->glGetActiveUniform( _handle, i, maxLen, 0, &size, &type, name );      
                    if ( !prefix || (::strlen(name) >= ::strlen(prefix) && ::strncmp(name, prefix, ::strlen(prefix)) == 0) )
                    {
                        GLint loc = _ext->glGetUniformLocation( _handle, name );
                        if ( loc != -1 )
                        {
                            _uniformLocations[osg::Uniform::getNameID(reinterpret_cast<const char*>(name))] = loc;
                        
                            OE_LOCAL << LC << "    Uniform = \"" << name << "\", location = " << loc << std::endl;
                        }
                    }
                }
            }
            else
            {
                OE_LOCAL << LC << "    No uniforms found." << std::endl;
            }
        }

        void apply(const osg::State* state)
        {
            bool useProgram = true;
            for (osg::State::UniformMap::const_iterator i = state->getUniformMap().begin(); i != state->getUniformMap().end(); ++i)
            {
                const osg::State::UniformStack& as = i->second;
                if (!as.uniformVec.empty())
                {
                    const osg::Uniform* uniform = static_cast<const osg::Uniform*>(as.uniformVec.back().first);
                    if (apply(uniform, useProgram))
                        useProgram = false;
                }
            }

#if 0
            // Apply each of the osg_*Matrix uniforms, which may be used by the native program
            if (apply(_modelViewMatrixUniform.get(), useProgram))
                useProgram = false;
            if (apply(_projectionMatrixUniform.get(), useProgram))
                useProgram = false;
            if (apply(_modelViewProjectionMatrixUniform.get(), useProgram))
                useProgram = false;
            if (apply(_normalMatrixUniform.get(), useProgram))
                useProgram = false;
#endif
        }

    private:
        GLint  _handle;
        typedef std::map<unsigned, GLint> UniformMap;
        UniformMap _uniformLocations;
        const osg::GL2Extensions* _ext;

        /** Apply the uniform to this program, optionally calling glUseProgram if necessary. */
        bool apply(const osg::Uniform* uniform, bool useProgram) const
        {
#if OSG_VERSION_GREATER_OR_EQUAL(3,7,0)
            UniformMap::const_iterator location = _uniformLocations.find( ((osg::UniformBase*)uniform)->getNameID() );
#else
            UniformMap::const_iterator location = _uniformLocations.find(uniform->getNameID());
#endif
            if ( location != _uniformLocations.end() )
            {        
                if ( useProgram )
                    _ext->glUseProgram( _handle );
                uniform->apply( _ext, location->second );
                return true;
            }
            else return false;
        }
    };

    /**
     * Collection of program adapters.
     */
    class NativeProgramAdapterCollection: public osg::MixinVector< osg::ref_ptr<NativeProgramAdapter> >
    {
    public:
        void apply(const osg::State* state) const
        {
            for(const_iterator i = begin(); i != end(); ++i )
                i->get()->apply( state );
        }
    };

}

#undef LC

#endif // OSGEARTH_NATIVE_PROGRAM_ADAPTER
