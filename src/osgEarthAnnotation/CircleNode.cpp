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
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarthSymbology/ExtrusionSymbol>
#include <osgEarth/MapNode>
#include <osgEarth/DrapeableNode>
#include <osg/MatrixTransform>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


CircleNode::CircleNode(MapNode*           mapNode,
                       const GeoPoint&    position,
                       const Linear&      radius,
                       const Style&       style,
                       bool               draped,
                       unsigned           numSegments) :

LocalizedNode( mapNode, position, false ),
_radius( radius ),
_style( style),
_draped( draped ),
_numSegments( numSegments )
{
    rebuild();
}

const Linear&
CircleNode::getRadius() const
{
    return _radius;
}

void
CircleNode::setRadius( const Linear& radius )
{
    if (_radius != radius )
    {
        _radius = radius;
        rebuild();
    }
}

unsigned int
CircleNode::getNumSegments() const
{
    return _numSegments;
}

void
CircleNode::setNumSegments(unsigned int numSegments )
{
    if (_numSegments != numSegments )
    {
        _numSegments = numSegments;
        rebuild();
    }
}

const Style&
CircleNode::getStyle() const
{
    return _style;
}

void
CircleNode::setStyle( const Style& style )
{
    _style = style;
    rebuild();
}

void
CircleNode::rebuild()
{
    std::string currentDecoration = getDecoration();
    clearDecoration();

    //Remove all children from this node
    removeChildren( 0, getNumChildren() );

    //Remove all children from the attach point
    getAttachPoint()->removeChildren( 0, getAttachPoint()->getNumChildren() );

    // construct a local-origin circle.
    GeometryFactory factory;
    Geometry* geom = factory.createCircle(osg::Vec3d(0,0,0), _radius, _numSegments);
    if ( geom )
    {
        //const SpatialReference* featureSRS = mapNode->getMapSRS()->createTangentPlaneSRS(position);

        GeometryCompiler compiler;
        osg::ref_ptr<Feature> feature = new Feature(geom, 0L); //todo: consider the SRS
        osg::Node* node = compiler.compile( feature.get(), _style, FilterContext(0L) );
        if ( node )
        {           
            getAttachPoint()->addChild( node );

            if ( _draped )
            {
                DrapeableNode* drapeable = new DrapeableNode( _mapNode.get(), true );
                drapeable->addChild( getAttachPoint() );
                this->addChild( drapeable );
            }

            else
            {
                this->addChild( getAttachPoint() );
            }
        }

        applyStyle( _style, _draped );
    }

    setDecoration( currentDecoration );
}
