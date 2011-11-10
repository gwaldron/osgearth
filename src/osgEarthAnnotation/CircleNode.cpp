/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/GeometryNode>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarth/MapNode>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


CircleNode::CircleNode(MapNode*           mapNode,
                       const osg::Vec3d&  position,
                       const Linear&      radius,
                       const Style&       style,
                       bool               draped,
                       unsigned           numSegments) :

LocalizedNode( mapNode->getMap()->getProfile()->getSRS(), position )
{
    // construct a local-origin circle.
    GeometryFactory factory;
    Geometry* geom = factory.createCircle(osg::Vec3d(0,0,0), radius, numSegments);
    if ( geom )
    {
        LocalGeometryNode* dg = new LocalGeometryNode( mapNode, geom, style, draped, getTransform() );
        this->addChild( dg );
    }
}
