/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/LocalTangentPlane>

using namespace osgEarth;

#define LC "[LTP] "

// --------------------------------------------------------------------------

TangentPlaneSpatialReference::TangentPlaneSpatialReference(
    const Key& key,
    const osg::Vec3d& originLLA) :

    SpatialReference(key),
    _originLLA(originLLA)
{
    _is_user_defined = true;
    _is_ltp = true;
    _domain = PROJECTED;
    _name = "Tangent Plane";

    // set up the LTP matrixes.

    osg::Vec3d xyz = getEllipsoid().geodeticToGeocentric(_originLLA);
    _local2world = getEllipsoid().geocentricToLocalToWorld(xyz);

    //getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(
    //    osg::DegreesToRadians(_originLLA.y()),
    //    osg::DegreesToRadians(_originLLA.x()),
    //    _originLLA.z(),
    //    _local2world);

    _world2local.invert( _local2world );
}

const SpatialReference*
TangentPlaneSpatialReference::preTransform(std::vector<osg::Vec3d>& points) const
{
    for(std::vector<osg::Vec3d>::iterator i = points.begin(); i != points.end(); ++i)
    {
        osg::Vec3d world = (*i) * _local2world;
        //double lat, lon, height;
        i->set(getEllipsoid().geocentricToGeodetic(world));
        //getEllipsoid()->convertXYZToLatLongHeight(world.x(), world.y(), world.z(), lat, lon, height);
        //i->x() = osg::RadiansToDegrees(lon);
        //i->y() = osg::RadiansToDegrees(lat);
        //i->z() = height;
    }
    return getGeodeticSRS();
}

const SpatialReference*
TangentPlaneSpatialReference::postTransform(std::vector<osg::Vec3d>& points) const
{
    osg::Vec3d world;
    for(std::vector<osg::Vec3d>::iterator i = points.begin(); i != points.end(); ++i)
    {
        world = getEllipsoid().geodeticToGeocentric(*i);
        //getEllipsoid()->convertLatLongHeightToXYZ(
        //    osg::DegreesToRadians(i->y()), osg::DegreesToRadians(i->x()), i->z(),
        //    world.x(), world.y(), world.z() );
        i->set( world * _world2local );
    }
    return getGeodeticSRS();
}

bool
TangentPlaneSpatialReference::_isEquivalentTo( const SpatialReference* srs, bool considerVDatum ) const
{
    return 
        srs->isLTP() && 
        _originLLA == static_cast<const TangentPlaneSpatialReference*>(srs)->_originLLA ;
    // todo: check the reference ellipsoids?
}
