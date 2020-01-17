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
#ifndef OSGEARTH_SCENE_GRAPH_CALLBACK_H
#define OSGEARTH_SCENE_GRAPH_CALLBACK_H 1

#include <osgEarth/Common>
#include <osgEarth/ThreadingUtils>
#include <osg/Node>
#include <osg/PagedLOD>
#include <osg/observer_ptr>

namespace osgEarth
{
    /**
     * Callback for getting notifications of scene graph events.
     * Not all methods may be invoked in all situations. It's up to
     * the implementation to call then when it wants.
     */
    class SceneGraphCallback : public osg::Referenced
    {
    public:
        //! Called before a node is added to the live scene graph, possibly from a background pager thread
        virtual void onPreMergeNode(osg::Node* node) { }
        virtual void onPreMergeNode(osg::Node* node, osg::Object* sender) { onPreMergeNode(node); }

        //! Called after a node is added to the live scene graph in the main/update thread
        virtual void onPostMergeNode(osg::Node* node) { }
        virtual void onPostMergeNode(osg::Node* node, osg::Object* sender) { onPostMergeNode(node); }

        //! Called after a node is remoevd from the live scene graph in the main/update thread
        virtual void onRemoveNode(osg::Node* node) { }
        virtual void onRemoveNode(osg::Node* node, osg::Object* sender) { onRemoveNode(node); }
    };
    typedef std::vector<osg::ref_ptr<SceneGraphCallback> > SceneGraphCallbackVector;

    /**
     * Cotnainer for scene graph callbacks. Typically an object that 
     * supports scene graph callbacks will host an instance of this
     * object and use it register and fire callbacks at the appropriate times.
     */
    class OSGEARTH_EXPORT SceneGraphCallbacks : public osg::Referenced
    {
    public:
        SceneGraphCallbacks(osg::Object* sender);

    public:
        //! Add a new callback
        virtual void add(SceneGraphCallback* cb);

        //! Remove an existing callback
        virtual void remove(SceneGraphCallback* cb);

        //! Invoke all pre-merge callbacks on the provided node
        virtual void firePreMergeNode(osg::Node* node);

        //! Invoke all post-merge callbacks on the provided node
        virtual void firePostMergeNode(osg::Node* node);

        //! Invoke all remove callbacks on the provided node
        virtual void fireRemoveNode(osg::Node* node);

    private:
        SceneGraphCallbackVector _callbacks;
        Threading::Mutex _mutex;
        osg::observer_ptr<osg::Object> _sender;
    };

    /**
     * Extends the osg::PagedLOD class to invoke SceneGraphCallbacks
     * when nodes are added or removed from the live scene graph.
     * (Note, this object does not invoke the preMergeNode method.)
     */
    class OSGEARTH_EXPORT PagedLODWithSceneGraphCallbacks : public osg::PagedLOD
    {
    public:
        PagedLODWithSceneGraphCallbacks(SceneGraphCallbacks* host);

        /**
         * Gets the SceneGraphCallbacks.
         */
        SceneGraphCallbacks* getSceneGraphCallbacks() const;

        /**
         * Sets the SceneGraphCallbacks
         */
        void setSceneGraphCallbacks(SceneGraphCallbacks* host);

    public: // osg::Group
        
        virtual bool addChild(osg::Node* child);
        virtual bool insertChild(unsigned index, osg::Node* child);
        virtual bool replaceChild(osg::Node* origChild, osg::Node* newChild);
        virtual void childRemoved(unsigned pos, unsigned num);

    private:
        osg::observer_ptr<SceneGraphCallbacks> _host;
        osg::observer_ptr<osg::Object> _sender;
    };
}

#endif // OSGEARTH_SCENE_GRAPH_CALLBACK_H
