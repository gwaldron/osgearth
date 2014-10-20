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
#include "ModelSplatter"

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[ModelSplatter] "


ModelSplatter::ModelSplatter()
{
    //nop
}

void 
ModelSplatter::setModel(osg::Node* node)
{
    _model = node;
}

void
ModelSplatter::setNumInstances(unsigned num)
{
    _count = num;
}

void 
ModelSplatter::setMinLOD(unsigned lod)
{
    _minLOD = lod;
}

void
ModelSplatter::operator()(const TileKey& key, osg::Node* node)
{
    osg::Group* group = node->asGroup();
    if ( !group )
        return;
}