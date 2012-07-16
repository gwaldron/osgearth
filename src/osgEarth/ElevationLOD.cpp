/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/ElevationLOD>
#include <osgEarth/GeoData>

#include <osgUtil/CullVisitor>
#include <osg/CoordinateSystemNode>

using namespace osgEarth;



ElevationLOD::ElevationLOD(const SpatialReference* srs):
_minElevation(-DBL_MAX),
_maxElevation(DBL_MAX),
_srs( srs )
{
}

ElevationLOD::ElevationLOD(const SpatialReference* srs, double minElevation, double maxElevation):
_minElevation( minElevation ),
_maxElevation( maxElevation ),
_srs( srs )
{
}

ElevationLOD::~ElevationLOD()
{
}

double ElevationLOD::getMinElevation() const
{
    return _minElevation;
}
        
void ElevationLOD::setMinElevation( double minElevation )
{
    _minElevation = minElevation;
}

double ElevationLOD::getMaxElevation() const
{
    return _maxElevation;
}

void ElevationLOD::setMaxElevation(double maxElevation )
{
    _maxElevation = maxElevation;
}

void ElevationLOD::setElevations( double minElevation, double maxElevation )
{
    _minElevation = minElevation;
    _maxElevation = maxElevation;
}

void ElevationLOD::traverse( osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() ==  osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( &nv );
        osg::Vec3d eye, center, up;        
        eye = cv->getViewPoint();

        float height = eye.z();
        if (_srs)
        {
            GeoPoint mapPoint;
            mapPoint.fromWorld( _srs, eye );        
            height = mapPoint.z();
        }

        //OE_NOTICE << "Height " << height << std::endl;

        if (height >= _minElevation && height <= _maxElevation)
        {
            osg::Group::traverse( nv );
        }
        else
        {
            //OE_NOTICE << "Elevation " << height << " outside of range " << _minElevation << " to " << _maxElevation << std::endl;
        }
    }
    else
    {
        osg::Group::traverse( nv );
    }
}
