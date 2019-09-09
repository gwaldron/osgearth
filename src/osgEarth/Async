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

#ifndef OSGEARTH_ASYNC_H
#define OSGEARTH_ASYNC_H

#include <osgEarth/Common>
#include <osgEarth/IOTypes>
#include <osgEarth/ThreadingUtils>
#include <osg/Group>

/**
 * Classes for asynchronous loading of scene graph nodes.
 */
namespace osgEarth 
{
    /**
     * Result returned from an asynchronous function
     */
    class OSGEARTH_EXPORT AsyncResult : public osg::Node
    {
    public:
        AsyncResult(osg::Node* node);
        osg::ref_ptr<osg::Node> _node;

    private:
        unsigned _requestId;
        friend struct AsyncNodePseudoLoader;
        friend class AsyncLOD;
    };

    //! Function that runs in the background to load a node
    class OSGEARTH_EXPORT AsyncFunction : public osg::Referenced
    {
    public:
        virtual osgEarth::ReadResult operator()() const = 0;
    };

    class AsyncNode;

    //! LOD group that loads its children asynchronously.
    //! This is a osg::PagedLOD replacement with more explicit
    //! sematics and expanded functionality.
    class AsyncLOD : public osg::Group
    {
    public:
        enum Mode
        {
            MODE_GEOMETRIC_ERROR,
            MODE_PIXEL_SIZE,
            MODE_RANGE,
        };

        enum Policy
        {
            //! Load child 0, then add child 1, then add child 2, etc.
            //! This policy lets you incrementally add more detail as you go.
            //! Children must be in detail order (lowest->highest)
            POLICY_ACCUMULATE,

            //! Load child 0, replace child 0 with child 1, replace child 1 with 2, etc.
            //! This policy is for levels of detail that wholly replace the lower LODs.            
            //! Children must be in detail order (lowest->highest)
            POLICY_REPLACE,

            //! Treat each child independently of the others.
            //! Like have each child under its own separate AsyncLOD node.
            //! Children can be in any order.
            POLICY_INDEPENDENT
        };

        AsyncLOD();

        //! Method used to determine child node visibility
        void setMode(const Mode& value);
        const Mode& getMode() const;

        //! How to display parent/child nodes with respect to each other
        void setPolicy(const Policy& value);
        const Policy& getPolicy() const;

        //! Center point of this node (since it cannot be determined
        //! before the contents are loaded)
        void setCenter(const osg::Vec3d& center);

        //! Radius of this node (since it cannot be determined
        //! before the contents are loaded)
        void setRadius(double radius);

        //! Adds a new potential child that will load asynchronously
        //! by calling the provided function in a background thread
        void addChild(AsyncFunction* function, float minValue, float maxValue);

        //! Adds a new "real" child
        void addChild(osg::Node* node, float minValue, float maxValue);

        //! Removes all children, real and potential
        void clear();

    public:

        // INTERNAL - only to be called by the pager
        // TODO: move this to a helper object and pass that in the myNodePath instead
        // so that this method will not sully our API
        bool addChild(osg::Node* node);

    public:

        //! Computes the bounding sphere, using a user-defined one first
        //! if it's available
        virtual osg::BoundingSphere computeBound() const;

        virtual void traverse(osg::NodeVisitor& nv);

    private:

        //! Made a request to the pager to schedule the async function call.
        void ask(AsyncNode& child, osg::NodeVisitor& nv);

        //! Determines whether a child node is visible and therefore
        //! should be processed for loading or viewing.
        virtual bool isVisible(const AsyncNode& async, osg::NodeVisitor& nv) const;

    private:

        // collection of child nodes (potential or real)
        std::vector<AsyncNode> _children;

        // internal, used to tell the pager where to attach newly loaded nodes
        osg::NodePath _nodePath;

        // osgDB loader options
        osg::ref_ptr<osgDB::Options> _readOptions;

        // talbe mapping a request ID to an asynchronous request object
        std::map<unsigned, AsyncNode*> _lookup;

        // protects the lookup table
        Threading::Mutex _mutex;

        // how to decide whether to start loading a child
        Mode _mode;

        // bounding sphere for the as-yet-unloaded data
        osg::BoundingSphere _userDefinedBound;

        // how child node should interact with each other
        Policy _policy;
    };
}

#endif // OSGEARTH_ASYNC_H
