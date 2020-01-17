/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2011 Pelican Mapping
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

#ifndef OSGEARTH_CLAMPABLE_NODE_H
#define OSGEARTH_CLAMPABLE_NODE_H 1

#include <osgEarth/Common>
#include <osg/Group>
#include <osg/observer_ptr>

namespace osgEarth
{
    class MapNode;

    /**
     * Node graph that will be GPU-clamped on the terrain, if supported.
     * This group must be a descendant of a MapNode.
     */
    class OSGEARTH_EXPORT ClampableNode : public osg::Group
    {
    public:
        //! Constructs a new clampable node.
        ClampableNode();

		//! Whether the given camera is a depth camera.
		static bool isDepthCamera(const osg::Camera* camera);

    public: // osg::Node

        virtual void traverse(osg::NodeVisitor& nv);

    protected:
        void setUniforms();
        void dirty();

        bool _mapNodeUpdateRequested;
        osg::observer_ptr<MapNode> _mapNode;

        virtual ~ClampableNode() { }
    };

} // namespace osgEarth

#endif // OSGEARTH_CLAMPABLE_NODE_H
