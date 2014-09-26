/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/CullingUtils>
#include <osgEarth/GeoData>
#include <osg/CoordinateSystemNode>

using namespace osgEarth;


ElevationLOD::ElevationLOD() :
_minElevation( -DBL_MAX ),
_maxElevation( DBL_MAX ),
_minRange    ( 0.0f ),
_maxRange    ( FLT_MAX )
{
    //nop
}

ElevationLOD::ElevationLOD( const ElevationLOD& rhs, const osg::CopyOp& op) :
_minElevation( rhs._minElevation ),
_maxElevation( rhs._maxElevation ),
_minRange    ( rhs._minRange ),
_maxRange    ( rhs._maxRange ),
_srs         ( rhs._srs.get() )
{
    //nop
}

ElevationLOD::ElevationLOD(const SpatialReference* srs):
_minElevation( -DBL_MAX ),
_maxElevation( DBL_MAX ),
_minRange    ( 0.0f ),
_maxRange    ( FLT_MAX ),
_srs         ( srs )
{
    //nop
}

ElevationLOD::ElevationLOD(const SpatialReference* srs, double minElevation, double maxElevation):
_minRange    ( 0.0f ),
_maxRange    ( FLT_MAX ),
_srs         ( srs )
{
    _minElevation = minElevation;
    _maxElevation = maxElevation;
}

ElevationLOD::~ElevationLOD()
{
    //nop
}

double ElevationLOD::getMinElevation() const
{
    return *_minElevation;
}
        
void ElevationLOD::setMinElevation( double minElevation )
{
    _minElevation = minElevation;
}

double ElevationLOD::getMaxElevation() const
{
    return *_maxElevation;
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

void ElevationLOD::setMinRange(float value)
{
    _minRange = value;
}

float ElevationLOD::getMinRange() const
{
    return *_minRange;
}

void ElevationLOD::setMaxRange(float value)
{
    _maxRange = value;
}

float ElevationLOD::getMaxRange() const
{
    return *_maxRange;
}

void ElevationLOD::traverse( osg::NodeVisitor& nv)
{
    if (nv.getVisitorType()   == osg::NodeVisitor::CULL_VISITOR &&
        nv.getTraversalMode() == osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN )
    {
        bool rangeOK     = true;
        bool altitudeOK  = true;

        // first test the range:
        if ( _minRange.isSet() || _maxRange.isSet() )
        {
            float range = nv.getDistanceToViewPoint( getBound().center(), true );
            rangeOK =
                (!_minRange.isSet() || (range >= *_minRange)) &&
                (!_maxRange.isSet() || (range <= *_maxRange));
        }

        if ( rangeOK )
        {
            if ( _minElevation.isSet() || _maxElevation.isSet() )
            {
                double alt;

                // first see if we have a precalculated elevation:
                osgUtil::CullVisitor*  cv = Culling::asCullVisitor(nv);
                Culling::CullUserData* ud = Culling::getCullUserData(cv);
                if ( ud && ud->_cameraAltitude.isSet() )
                {
                    // yes; use it
                    alt = ud->_cameraAltitude.get();
                }
                else
                {
                    // no; need to calculate elevation here:
                    osg::Vec3d eye = cv->getViewPoint();

                    if ( _srs && !_srs->isProjected() )
                    {
                        GeoPoint mapPoint;
                        mapPoint.fromWorld( _srs.get(), eye );
                        alt = mapPoint.z();
                    }
                    else
                    {
                        alt = eye.z();
                    }
                }

                // account for the LOD scale
                alt *= cv->getLODScale();

                altitudeOK =
                    (!_minElevation.isSet() || (alt >= *_minElevation)) &&
                    (!_maxElevation.isSet() || (alt <= *_maxElevation));
            }

            if ( altitudeOK )
            {
                std::for_each(_children.begin(),_children.end(),osg::NodeAcceptOp(nv));
            }
        }
    }
    else
    {
        osg::Group::traverse( nv );
    }
}
