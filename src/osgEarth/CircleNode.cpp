/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/CircleNode>
#include <osgEarth/AnnotationRegistry>
#include <osgEarth/GeometryFactory>
#include <osgEarth/MapNode>
#include <cmath>

using namespace osgEarth;


CircleNode::CircleNode() :
LocalGeometryNode()
{
    construct();
}

void
CircleNode::set(const GeoPoint&    position,
                const Distance&    radius,
                const Style&       style,
                const Angle&       arcStart,
                const Angle&       arcEnd,
                const bool         pie)
{
    _radius = radius;
    _arcStart = arcStart;
    _arcEnd = arcEnd;
    _pie = pie;
    _style = style;

    setPosition(position);

    buildGeometry();
}

void
CircleNode::construct()
{
    setName("Circle");

    _radius.set(1.0, Units::KILOMETERS);
    _arcStart.set(0.0, Units::RADIANS);
    _arcEnd.set(2.0*osg::PI, Units::RADIANS);
    _pie = false;
    _numSegments = 0;
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
        buildGeometry();
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
        buildGeometry();
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
	buildGeometry();
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
	buildGeometry();
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
    buildGeometry();
}

void
CircleNode::buildGeometry()
{
    // construct a local-origin circle.
    GeometryFactory factory;
    osg::ref_ptr<Geometry> geom;
    if (std::abs(_arcEnd.as(Units::DEGREES) - _arcStart.as(Units::DEGREES)) >= 360.0)
    {
        geom = factory.createCircle(osg::Vec3d(0,0,0), _radius, _numSegments);
    }
    else
    {
        geom = factory.createArc(osg::Vec3d(0,0,0), _radius, _arcStart, _arcEnd, _numSegments, 0L, _pie);
    }

    if ( geom.valid() )
    {
        setGeometry(geom.get());
    }
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( circle, osgEarth::CircleNode );


CircleNode::CircleNode(const Config&         conf,
                       const osgDB::Options* dbOptions) :
LocalGeometryNode(conf, dbOptions)
{
    construct();

    conf.get( "radius",       _radius );
    conf.get( "num_segments", _numSegments );

    buildGeometry();
}

Config
CircleNode::getConfig() const
{
    Config conf = LocalGeometryNode::getConfig();
    conf.key() = "circle";

    conf.set( "radius", _radius );

    if ( _numSegments != 0 )
        conf.add( "num_segments", _numSegments );

    return conf;
}
