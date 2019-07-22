/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef OSGEARTH_LIGHTING_H
#define OSGEARTH_LIGHTING_H 1

#include <osgEarth/Common>
#include <osgEarth/ThreadingUtils>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/LightSource>
#include <osg/Light>
#include <osg/Material>
#include <set>

// Use this with StateSet::setDefine to toggle lighting in osgEarth lighting shaders.
#define OE_LIGHTING_DEFINE "OE_LIGHTING"

namespace osgEarth
{
    struct OSGEARTH_EXPORT Lighting
    {
        //! Sets the lighting mode on a stateset
        static void set(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue value);

        //! Removed the lighting mode from a stateset
        static void remove(osg::StateSet* stateSet);
    };

    /**
     * Traverses a graph, looking for Lights and Materials, and generates either static
     * uniforms or dynamic cull callbacks for them so they will work with core profile
     * shaders. (This is necessary for GL3+ core, OSX, Mesa etc. that don't support
     * compatibility mode.)
     */
    class OSGEARTH_EXPORT GenerateGL3LightingUniforms : public osg::NodeVisitor
    {
    public:
        GenerateGL3LightingUniforms();

    public: // osg::NodeVisitor
        virtual void apply(osg::Node& node);
        virtual void apply(osg::LightSource& node);

    protected:
        std::set<osg::StateSet*> _statesets;

        template<typename T> bool alreadyInstalled(osg::Callback* cb) const {
            return !cb ? false : dynamic_cast<T*>(cb)!=0L ? true : alreadyInstalled<T>(cb->getNestedCallback());
        }
    };

    /**
     * Material that will work in both FFP and non-FFP mode, by using the uniform
     * osg_FrontMaterial in place of gl_FrontMaterial.
     */
    class OSGEARTH_EXPORT MaterialGL3 : public osg::Material
    {
    public:
        MaterialGL3() : osg::Material() { }
        MaterialGL3(const MaterialGL3& mat, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY) : osg::Material(mat, copyop) {}
        MaterialGL3(const osg::Material& mat) : osg::Material(mat) { }

        META_StateAttribute(osgEarth, MaterialGL3, MATERIAL);

        virtual void apply(osg::State&) const;
    };

    /**
     * StateAttributeCallback that will update osg::Material properties as Uniforms
     */
    class OSGEARTH_EXPORT MaterialCallback : public osg::StateAttributeCallback
    {
    public:
        virtual void operator() (osg::StateAttribute* attr, osg::NodeVisitor* nv);        
    };

    /**
     * Light that will work in both FFP and non-FFP mode.
     * To use this, find a LightSource and replace the Light with a LightGL3.
     * Then install the LightSourceGL3UniformGenerator cull callback on the LightSource.
     */
    class OSGEARTH_EXPORT LightGL3 : public osg::Light
    {
    public:
        LightGL3() : osg::Light(), _enabled(true) { }
        LightGL3(int lightnum) : osg::Light(lightnum), _enabled(true) { }
        LightGL3(const LightGL3& light, const osg::CopyOp& copyop =osg::CopyOp::SHALLOW_COPY)
            : osg::Light(light, copyop), _enabled(light._enabled) {}
        LightGL3(const osg::Light& light)
            : osg::Light(light), _enabled(true) { }

        void setEnabled(bool value) { _enabled = value; }
        bool getEnabled() const { return _enabled; }

        virtual osg::Object* cloneType() const { return new LightGL3(_lightnum); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new LightGL3(*this,copyop); }
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const LightGL3 *>(obj)!=NULL; }
        virtual const char* libraryName() const { return "osgEarth"; }
        virtual const char* className() const { return "LightGL3"; }
        virtual Type getType() const { return LIGHT; }

        virtual void apply(osg::State&) const;

    protected:
        bool _enabled;
    };

    /**
     * Cull callback that will install Light uniforms based on the Light in a LightSource.
     * Install this directly on the LightSource node. 
     */
    class OSGEARTH_EXPORT LightSourceGL3UniformGenerator : public osg::NodeCallback
    {
    public:
        /**
         * Creates and installs Uniforms on the stateset for the Light components
         * of the Light that are non-positional (everything but the position and direction)
         */
        void generateNonPositionalData(osg::StateSet* ss, osg::Light* light);

    public: // osg::NodeCallback

        bool run(osg::Object* obj, osg::Object* data);

    public: // osg::Object

        void resizeGLBufferObjects(unsigned maxSize);
        void releaseGLObjects(osg::State* state) const;

        mutable std::vector<osg::ref_ptr<osg::StateSet> > _statesets;
        mutable Threading::Mutex _statesetsMutex;
    };
}

#endif // OSGEARTH_LIGHTING_H
