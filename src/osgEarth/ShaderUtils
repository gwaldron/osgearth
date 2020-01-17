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
#ifndef OSGEARTH_SHADER_UTILS_H
#define OSGEARTH_SHADER_UTILS_H 1

#include <osgEarth/Common>
#include <osg/NodeCallback>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/Light>
#include <osg/Material>
#include <osg/observer_ptr>
#include <osgDB/Options>
#include <map>

namespace osgEarth
{
    /**
     * ShaderPolicy encodes general behavior when deciding how to
     * employ shaders in certain situations
     */
    enum ShaderPolicy
    {
        SHADERPOLICY_DISABLE,
        SHADERPOLICY_GENERATE,
        SHADERPOLICY_INHERIT
    };

    /**
     * Preprocesses GLES shader source to include our osg_LightProducts and osg_LightSourceParameters
     * definitions and uniforms.
     */
    class OSGEARTH_EXPORT ShaderPreProcessor
    {
    public:
        static void run(osg::Shader* shader);

        //static void applySupportForNoFFP(osg::Shader* shader);
    };

    /**
     * Helper class for dealing with array uniforms. Array uniform naming works
     * differently on different drivers (ATI vs NVIDIA), so this class helps mitigate
     * those differences.
     */
    class OSGEARTH_EXPORT ArrayUniform //  : public osg::Referenced
    {
    public:
        /** Empty array uniform */
        ArrayUniform() { }

        /**
         * Creates or retrieves a named uniform array.
         */
        ArrayUniform(
            const std::string& name,
            osg::Uniform::Type type,
            osg::StateSet*     stateSet,
            unsigned           size =1 );

        /** dtor */
        virtual ~ArrayUniform() { }

        void attach(
            const std::string& name,
            osg::Uniform::Type type,
            osg::StateSet*     stateSet,
            unsigned           size =1 );

        void detach();

        void setElement( unsigned index, int value );
        void setElement( unsigned index, unsigned value );
        void setElement( unsigned index, bool value );
        void setElement( unsigned index, float value );
        void setElement( unsigned index, const osg::Matrix& value );
        void setElement( unsigned index, const osg::Vec3& value );

        bool getElement( unsigned index, int& out_value ) const;
        bool getElement( unsigned index, unsigned& out_value ) const;
        bool getElement( unsigned index, bool& out_value ) const;
        bool getElement( unsigned index, float& out_value ) const;
        bool getElement( unsigned index, osg::Matrix& out_value ) const;
        bool getElement( unsigned index, osg::Vec3& out_value ) const;

        bool isValid() const { return _uniform.valid() && _uniformAlt.valid(); }
        int getNumElements() const { return isValid() ? _uniform->getNumElements() : -1; }

        bool isDirty() const { return
            (_uniform.valid() && _uniform->getModifiedCount() > 0) ||
            (_uniformAlt.valid() && _uniformAlt->getModifiedCount() > 0); }

    private:
        osg::ref_ptr<osg::Uniform>       _uniform;
        osg::ref_ptr<osg::Uniform>       _uniformAlt;
        osg::observer_ptr<osg::StateSet> _stateSet;

        void ensureCapacity( unsigned newSize );
    };


    /**
     * Cull callback that installs a range (distance to view point) uniform
     * in the State based on the bounding center of the node being culled.
     * The actual name of the range uniform can is returned by
     * ShaderFactory::getRangeUniformName().
     */
    class OSGEARTH_EXPORT RangeUniformCullCallback : public osg::NodeCallback
    {
    public:
        RangeUniformCullCallback();
        void operator()(osg::Node*, osg::NodeVisitor* nv);

        // testing
        void setDump(bool v) { _dump = true; }

    private:
        osg::ref_ptr<osg::StateSet> _stateSet;
        osg::ref_ptr<osg::Uniform>  _uniform;
        bool                        _dump;
    };


    /**
     * Shader that will discard fragments whose alpha component falls below
     * the specified threshold.
     */
    class OSGEARTH_EXPORT DiscardAlphaFragments
    {
    public:
        void install(osg::StateSet* ss, float minAlpha) const;
        void uninstall(osg::StateSet* ss) const;
    };
}

#endif // OSGEARTH_SHADER_UTILS_H
