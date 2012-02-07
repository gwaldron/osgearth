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

#include <osgEarthAnnotation/RectangleNode>
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


RectangleNode::RectangleNode(
            MapNode*          mapNode,
            const osg::Vec3d& position,
            const Linear&     width,
            const Linear&     height,
            const Style&      style,
            bool              draped ) :
LocalizedNode( mapNode, position, false ),
_width( width ),
_height( height ),
_style( style ),
_draped( draped )
{       
    rebuild();
}

const Linear&
RectangleNode::getWidth() const
{
    return _width;
}

const Linear&
RectangleNode::getHeight() const
{
    return _height;
}

void
RectangleNode::setWidth( const Linear& width )
{
    setSize( width, _height );
}

void
RectangleNode::setHeight( const Linear& height )
{
    setSize( _width, height );
}

void
RectangleNode::setSize( const Linear& width, const Linear& height)
{
    if (_width != width || _height != height)
    {
        _width = width;
        _height = height;
        rebuild();
    }
}

const Style&
RectangleNode::getStyle() const
{
    return _style;
}

void
RectangleNode::setStyle( const Style& style )
{
    _style = style;
    rebuild();
}

void
RectangleNode::rebuild()
{    
    //Remove all children from this node
    removeChildren( 0, getNumChildren() );

    //Remove all children from the attach point
    getAttachPoint()->removeChildren( 0, getAttachPoint()->getNumChildren() );

    // construct a local-origin circle.
    GeometryFactory factory;    
    Geometry* geom = factory.createRectangle(osg::Vec3d(0,0,0), _width, _height);
    if ( geom )
    {
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
}
