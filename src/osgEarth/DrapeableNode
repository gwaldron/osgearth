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

#ifndef OSGEARTH_DRAPEABLE_NODE_H
#define OSGEARTH_DRAPEABLE_NODE_H 1

#include <osgEarth/Common>
#include <osgEarth/optional>
#include <osgEarth/MapNode>
#include <osgEarth/MapNodeObserver>
#include <osg/Group>

namespace osgEarth
{
    /**
     * Base class for a graph that can be "draped" on the terrain using
     * projective texturing.
     *
     * Usage: Create this node and place it as a descendant of a MapNode.
     */
    class OSGEARTH_EXPORT DrapeableNode : public osg::Group, public MapNodeObserver
    {
    public:
        META_Object(osgEarth, DrapeableNode);

        /**
         * Constructs a new drapeable node.
         */
        DrapeableNode();

        /** Copy */
        DrapeableNode(const DrapeableNode& rhs, const osg::CopyOp& copy);

        /**
         * Whether draping is enabled.
         */
        void setDrapingEnabled(bool value);
        bool getDrapingEnabled() const     { return _drapingEnabled; }

    public: // osg::Group/Node

        virtual void traverse(osg::NodeVisitor& nv);

    public: // osgEarth::MapNodeObserver

        virtual void setMapNode(MapNode* mapNode) { _mapNode = mapNode; }
        virtual MapNode* getMapNode() { return _mapNode.get(); }

    protected:
        /** dtor */
        virtual ~DrapeableNode() { }

        bool _drapingEnabled;
        bool _updateRequested;
        osg::observer_ptr<MapNode> _mapNode;
    };

} // namespace osgEarth

#endif // OSGEARTH_DRAPEABLE_NODE_H
