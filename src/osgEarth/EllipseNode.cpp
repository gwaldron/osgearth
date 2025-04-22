/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/EllipseNode>
#include <osgEarth/AnnotationRegistry>
#include <osgEarth/GeometryFactory>
#include <osgEarth/MapNode>
#include <cstdlib>

using namespace osgEarth;


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
    _rotationAngle.set(0.0, Units::RADIANS);
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

OSGEARTH_REGISTER_ANNOTATION( ellipse, osgEarth::EllipseNode );


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
