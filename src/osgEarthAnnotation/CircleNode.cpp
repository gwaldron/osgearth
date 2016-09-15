/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarthSymbology/ExtrusionSymbol>
#include <osgEarth/MapNode>
#include <osgEarth/DrapeableNode>
#include <osg/MatrixTransform>
#include <cmath>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


CircleNode::CircleNode(MapNode*           mapNode,
                       const GeoPoint&    position,
                       const Linear&      radius,
                       const Style&       style,
                       const Angle&       arcStart,
                       const Angle&       arcEnd,
                       const bool         pie):

LocalGeometryNode(mapNode),
_radius      ( radius ),
_arcStart    ( arcStart ),
_arcEnd      ( arcEnd ),
_pie         ( pie ),
_numSegments ( 0 )
{
    initCircleNode();
    setStyle( style );
    setPosition( position );
}


void
CircleNode::initCircleNode()
{
    setName("Circle");
    rebuildGeometry();
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
        rebuildGeometry();
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
        rebuildGeometry();
    }
}

const Angle&
CircleNode::getArcStart(void) const
{
	return (_arcStart);
}

void
CircleNode::setArcStart(const Angle& arcStart)
{
	_arcStart = arcStart;
	rebuildGeometry();
}

const Angle&
CircleNode::getArcEnd(void) const
{
	return (_arcEnd);
}

void
CircleNode::setArcEnd(const Angle& arcEnd)
{
	_arcEnd = arcEnd;
	rebuildGeometry();
}

const bool&
CircleNode::getPie(void) const
{
	return (_pie);
}

void
CircleNode::setPie(const bool& pie)
{
    _pie = pie;
    rebuildGeometry();
}

void
CircleNode::rebuildGeometry()
{
    // construct a local-origin circle.
    GeometryFactory factory;
    Geometry* geom = NULL;
    if (std::abs(_arcEnd.as(Units::DEGREES) - _arcStart.as(Units::DEGREES)) >= 360.0)
    {
        geom = factory.createCircle(osg::Vec3d(0,0,0), _radius, _numSegments);
    }
    else
    {
        geom = factory.createArc(osg::Vec3d(0,0,0), _radius, _arcStart, _arcEnd, _numSegments, 0L, _pie);
    }

    if ( geom )
    {
        setGeometry( geom );        
    }
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( circle, osgEarth::Annotation::CircleNode );


CircleNode::CircleNode(MapNode*              mapNode,
                       const Config&         conf,
                       const osgDB::Options* dbOptions) :
LocalGeometryNode( mapNode, conf, dbOptions ),
_radius      ( 1.0, Units::KILOMETERS ),
_numSegments ( 0 )
{
    conf.getObjIfSet( "radius",       _radius );
    conf.getIfSet   ( "num_segments", _numSegments );
    initCircleNode();
}

Config
CircleNode::getConfig() const
{
    Config conf = LocalGeometryNode::getConfig();
    conf.key() = "circle";

    conf.addObj( "radius", _radius );

    if ( _numSegments != 0 )
        conf.add( "num_segments", _numSegments );

    return conf;
}
