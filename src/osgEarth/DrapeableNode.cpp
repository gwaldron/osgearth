/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include <osgEarth/DrapeableNode>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>

#define LC "[DrapeableNode] "

using namespace osgEarth;


DrapeableNode::DrapeableNode() :
_drapingEnabled( true )
{
    //nop
}

void
DrapeableNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // access the cull visitor:
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

        // find the cull set for this thread:
        DrapingCullSet& cullSet = Registry::drapingCullSet();
        cullSet.push( this, cv->getNodePath() );
    }
    else
    {
        osg::Group::traverse( nv );
    }
}
