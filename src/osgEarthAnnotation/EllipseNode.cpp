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
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarth/DrapeableNode>
#include <osgEarth/MapNode>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


EllipseNode::EllipseNode(MapNode*          mapNode,
                         const GeoPoint&   position,
                         const Linear&     radiusMajor,
                         const Linear&     radiusMinor,
                         const Angular&    rotationAngle,
                         const Style&      style,
                         bool              draped,
                         unsigned          numSegments) :
LocalizedNode( mapNode, position ),
_radiusMajor( radiusMajor ),
_radiusMinor( radiusMinor ),
_rotationAngle( rotationAngle ),
_style(style ),
_draped( draped ),
_numSegments( numSegments )
{
    rebuild();
}

const Style&
EllipseNode::getStyle() const
{
    return _style;
}

void
EllipseNode::setStyle( const Style& style )
{
    _style = style;
    rebuild();
}

unsigned int
EllipseNode::getNumSegments() const
{
    return _numSegments;
}

void
EllipseNode::setNumSegments(unsigned int numSegments )
{
    if (_numSegments != numSegments )
    {
        _numSegments = numSegments;
        rebuild();
    }
}


const Linear&
EllipseNode::getRadiusMajor() const
{
    return _radiusMajor;
}

const Linear&
EllipseNode::getRadiusMinor() const
{
    return _radiusMinor;
}

void
EllipseNode::setRadiusMajor( const Linear& radiusMajor )
{
    setRadii( radiusMajor, _radiusMinor );
}


void
EllipseNode::setRadiusMinor( const Linear& radiusMinor )
{
    setRadii( _radiusMajor, radiusMinor );
}

void
EllipseNode::setRadii( const Linear& radiusMajor, const Linear& radiusMinor )
{
    if (_radiusMajor != radiusMajor || _radiusMinor != radiusMinor )
    {
        _radiusMajor = radiusMajor;
        _radiusMinor = radiusMinor;
        rebuild();
    }
}

const Angular&
EllipseNode::getRotationAngle() const
{
    return _rotationAngle;
}

void 
EllipseNode::setRotationAngle(const Angular& rotationAngle)
{
    if (_rotationAngle != rotationAngle)
    {
        _rotationAngle = rotationAngle;
        rebuild();
    }
}


void
EllipseNode::rebuild()
{
    std::string currentDecoration = getDecoration();
    clearDecoration();

    //Remove all children from this node
    removeChildren( 0, getNumChildren() );

    //Remove all children from the attach point
    getAttachPoint()->removeChildren( 0, getAttachPoint()->getNumChildren() );

    // construct a local-origin ellipse.
    GeometryFactory factory;
    Geometry* geom = factory.createEllipse(osg::Vec3d(0,0,0), _radiusMajor, _radiusMinor, _rotationAngle, _numSegments);
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
                DrapeableNode* drapeable = new DrapeableNode( _mapNode.get() );
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



//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( ellipse, osgEarth::Annotation::EllipseNode );


EllipseNode::EllipseNode(MapNode*      mapNode,
                         const Config& conf ) :
LocalizedNode( mapNode ),
_draped      ( false ),
_numSegments ( 0 )
{
    conf.getObjIfSet( "radius_major", _radiusMajor );
    conf.getObjIfSet( "radius_minor", _radiusMinor );
    conf.getObjIfSet( "rotation", _rotationAngle );
    conf.getObjIfSet( "style",  _style );
    conf.getIfSet   ( "draped", _draped );
    conf.getIfSet   ( "num_segments", _numSegments );

    if ( conf.hasChild("position") )
        setPosition( GeoPoint(conf.child("position")) );

    rebuild();
}

Config
EllipseNode::getConfig() const
{
    Config conf( "ellipse" );
    conf.addObj( "radius_major", _radiusMajor );
    conf.addObj( "radius_minor", _radiusMinor );
    conf.addObj( "rotation", _rotationAngle );
    conf.addObj( "style", _style );

    if ( _numSegments != 0 )
        conf.add( "num_segments", _numSegments );
    if ( _draped != false )
        conf.add( "draped", _draped );

    conf.addObj( "position", getPosition() );

    return conf;
}
