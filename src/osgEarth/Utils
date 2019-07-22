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
#ifndef OSGEARTH_UTILS_H
#define OSGEARTH_UTILS_H 1

#include <osgEarth/Common>
#include <osgEarth/StringUtils>

#include <osg/Vec3f>
#include <osg/AutoTransform>
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>
#include <osgUtil/CullVisitor>
#include <osgUtil/RenderBin>

#include <string>
#include <list>
#include <map>

namespace osg
{
    class EllipsoidModel;
}

namespace osgEarth
{    
    //------------------------------------------------------------------------

    /**
     * Utility for storing observables in an osg::Object
     */
    template<typename T>
    class OptionsData : public osg::Object {
    public:
        static void set(osg::Object* o, const std::string& name, T* obj) {            
            osg::UserDataContainer* udc = o->getOrCreateUserDataContainer();
            unsigned i = udc->getUserObjectIndex(name);
            if (i == udc->getNumUserObjects()) udc->removeUserObject(i);
            udc->addUserObject(new OptionsData<T>(name, obj));
        }
        static bool lock(const osg::Object* o, const std::string& name, osg::ref_ptr<T>& out) {
            if (!o) return false;
            const OptionsData<T>* data = dynamic_cast<const OptionsData<T>*>(osg::getUserObject(o, name));
            return data ? data->_obj.lock(out) : false;
        }
        static osg::ref_ptr<T> get(const osg::Object* o, const std::string& name) {
            osg::ref_ptr<T> result;
            bool ok = lock(o, name, result);
            return result;
        }
    public:
        META_Object(osgEarth,OptionsData);
        OptionsData(const std::string& name, T* obj) : _obj(obj) { setName(name); }
        OptionsData() { }
        OptionsData(const OptionsData& rhs, const osg::CopyOp& copy) : osg::Object(rhs, copy), _obj(rhs._obj) { }
    private:
        osg::observer_ptr<T> _obj;
    };

    struct Utils
    {
        /**
         * Clamps v to [vmin..vmax], then remaps its range to [r0..r1]. 
         */
        static double remap( double v, double vmin, double vmax, double r0, double r1 )
        {
            float vr = (osg::clampBetween(v, vmin, vmax)-vmin)/(vmax-vmin);
            return r0 + vr * (r1-r0);
        }
    };

    /**
     * Proxy class that registers a custom render bin's prototype with the
     * rendering system
     */
    template<class T>
    struct osgEarthRegisterRenderBinProxy
    {
        osgEarthRegisterRenderBinProxy(const std::string& name)
        {
            _prototype = new T();
            osgUtil::RenderBin::addRenderBinPrototype(name, _prototype.get());
        }

        ~osgEarthRegisterRenderBinProxy()
        {
            osgUtil::RenderBin::removeRenderBinPrototype(_prototype.get());
            _prototype = 0L;
        }

        osg::ref_ptr<T> _prototype;
    };

    struct OSGEARTH_EXPORT RenderBinUtils
    {
        static unsigned getTotalNumRenderLeaves(osgUtil::RenderBin* bin);
    };

    /**
     * Shim to apply vertex cache optimizations to geometry when it's legal.
     * This is really only here to work around an OSG bug in the VertexAccessOrder
     * optimizer, which corrupts non-Triangle geometries.
     */
    struct OSGEARTH_EXPORT VertexCacheOptimizer : public osg::NodeVisitor
    {
        VertexCacheOptimizer();
        virtual ~VertexCacheOptimizer() { }
        void apply(osg::Drawable& drawable);
    };

    /**
     * Sets the data variance on all discovered drawables.
     */
    struct OSGEARTH_EXPORT SetDataVarianceVisitor : public osg::NodeVisitor
    {
        SetDataVarianceVisitor(osg::Object::DataVariance value);
        virtual ~SetDataVarianceVisitor() { }
        void apply(osg::Drawable& drawable);
        osg::Object::DataVariance _value;
    };

    /**
     * Scans geometry and validates that it's set up properly.
     */    
    struct OSGEARTH_EXPORT GeometryValidator : public osg::NodeVisitor
    {
        GeometryValidator();
        void apply(osg::Group& group);
        void apply(osg::Geometry& geom);
    };

    /**
     * Allocates and merges buffer objects for each Drawable in the scene graph.
     */
    class OSGEARTH_EXPORT AllocateAndMergeBufferObjectsVisitor : public osg::NodeVisitor
    {
    public:
        AllocateAndMergeBufferObjectsVisitor();
        virtual ~AllocateAndMergeBufferObjectsVisitor() { }
        void apply(osg::Drawable& drawable);
    };
}

#endif // OSGEARTH_UTILS_H
