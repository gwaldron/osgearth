/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarth/DrapeableNode>
#include <osgEarth/MapNode>
#include <cmath>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


EllipseNode::EllipseNode() :
LocalGeometryNode()
{
    construct();
}

void
EllipseNode::construct()
{
    _radiusMajor.set(1.0, Units::KILOMETERS);
    _radiusMinor.set(0.5, Units::KILOMETERS);
    _rotationAngle = 0.0;
    _arcStart.set(0.0, Units::RADIANS);
    _arcEnd.set(2.0*osg::PI, Units::RADIANS);
    _pie = false;
    _numSegments = 0;
}

void
EllipseNode::set(const GeoPoint& position,
                 const Distance& radiusMajor,
                 const Distance& radiusMinor,
                 const Angle&    rotationAngle,
                 const Style&    style,
                 const Angle&    arcStart,
                 const Angle&    arcEnd,
                 const bool      pie)
{
    _radiusMajor = radiusMajor;
    _radiusMinor = radiusMinor;
    _rotationAngle = rotationAngle;
    _arcStart = arcStart;
    _arcEnd = arcEnd;
    _pie = pie;

    _style = style;

    setPosition(position);

    buildGeometry();
}

unsigned int
EllipseNode::getNumSegments() const
{
    return _numSegments;
}

void
EllipseNode::setNumSegments(unsigned numSegments)
{
    if (_numSegments != numSegments )
    {
        _numSegments = numSegments;
        buildGeometry();
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
        buildGeometry();
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
        buildGeometry();
    }
}
const Angle&
EllipseNode::getArcStart(void) const
{
    return _arcStart;
}

void
EllipseNode::setArcStart(const Angle& arcStart)
{
    _arcStart = arcStart;
    buildGeometry();
}

const Angle&
EllipseNode::getArcEnd(void) const
{
    return _arcEnd;
}

void
EllipseNode::setArcEnd(const Angle& arcEnd)
{
    _arcEnd = arcEnd;
    buildGeometry();
}

const bool&
EllipseNode::getPie(void) const
{
	return _pie;
}

void
EllipseNode::setPie(const bool& pie)
{
	_pie = pie;
	buildGeometry();
}

void
EllipseNode::buildGeometry()
{
    // construct a local-origin ellipse.
    GeometryFactory factory;

    osg::ref_ptr<Geometry> geom;

    if (std::abs(_arcEnd.as(Units::DEGREES) - _arcStart.as(Units::DEGREES)) >= 360.0)
    {
        geom = factory.createEllipse(osg::Vec3d(0,0,0), _radiusMajor, _radiusMinor, _rotationAngle, _numSegments);
    }
    else
    {
        geom = factory.createEllipticalArc(osg::Vec3d(0,0,0), _radiusMajor, _radiusMinor, _rotationAngle, _arcStart, _arcEnd, _numSegments, 0L, _pie);
    }

    if ( geom.valid() )
    {
        setGeometry( geom.get() );
    }
}



//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( ellipse, osgEarth::Annotation::EllipseNode );


EllipseNode::EllipseNode(const Config&         conf,
                         const osgDB::Options* dbOptions) :
LocalGeometryNode(conf, dbOptions)
{
    construct();

    conf.get( "radius_major", _radiusMajor );
    conf.get( "radius_minor", _radiusMinor );
    conf.get( "rotation", _rotationAngle );
    conf.get( "num_segments", _numSegments );

    buildGeometry();
}

Config
EllipseNode::getConfig() const
{
    Config conf = LocalGeometryNode::getConfig();
    conf.key() = "ellipse";

    conf.set( "radius_major", _radiusMajor );
    conf.set( "radius_minor", _radiusMinor );
    conf.set( "rotation", _rotationAngle );

    if ( _numSegments != 0 )
        conf.set( "num_segments", _numSegments );

    return conf;
}
