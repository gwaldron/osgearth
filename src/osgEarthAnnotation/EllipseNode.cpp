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
#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthFeatures/FeatureNode>
#include <osgEarthSymbology/GeometryFactory>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


EllipseNode::EllipseNode(MapNode*          mapNode,
                         const osg::Vec3d& center,
                         const Linear&     radiusMajor,
                         const Linear&     radiusMinor,
                         const Angular&    rotationAngle,
                         const Style&      style,
                         bool              draped,
                         unsigned          numSegments) :
FeatureNode( mapNode, 0L, draped )
{
    if ( mapNode )
    {
        GeometryFactory factory( mapNode->getMap()->getProfile()->getSRS() );
        Geometry* geom = factory.createEllipse(center, radiusMajor, radiusMinor, rotationAngle, numSegments);
        if ( geom )
        {
            Feature* feature = new Feature( geom, style );
            feature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
            setFeature( feature );
        }
    }
}