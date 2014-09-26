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
#include <osgEarth/OverlayDecorator>
#include <osgEarth/DrapingTechnique>
#include <osgEarth/MapNode>

#define LC "[DrapeableNode] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    static osg::Group* getTechniqueGroup(MapNode* m)
    {
        return m ? m->getOverlayDecorator()->getGroup<DrapingTechnique>() : 0L;
    }
}

//------------------------------------------------------------------------

DrapeableNode::DrapeableNode( MapNode* mapNode, bool draped ) :
OverlayNode( mapNode, draped, &getTechniqueGroup )
{
    //nop
}

void
DrapeableNode::setRenderOrder(int order)
{
    _renderOrder = order;
    osg::StateSet* s = _overlayProxyContainer->getOrCreateStateSet();
    s->setRenderBinDetails(order, "RenderBin");
}
