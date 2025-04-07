/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ElevationLOD>
#include <osgEarth/CullingUtils>
#include <osgEarth/Utils>
#include "Horizon"

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
    if (nv.getVisitorType()   == osg::NodeVisitor::CULL_VISITOR)
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
                osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

                osg::Vec3d eye = cv->getViewPoint();

                osg::ref_ptr<Horizon> horizon;
                if (ObjectStorage::get(&nv, horizon))
                {
                    double R = horizon->getRadius();
                    alt = eye.length() - R;
                }

                else if ( _srs && !_srs->isProjected() )
                {
                    GeoPoint mapPoint;
                    mapPoint.fromWorld( _srs.get(), eye );
                    alt = mapPoint.z();
                }
                else
                {
                    alt = eye.z();
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
