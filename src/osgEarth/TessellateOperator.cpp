/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/TessellateOperator>
#include <osgEarth/Geometry>
#include <osgEarth/GeoMath>

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

void 
TessellateOperator::tessellateLinear( const osg::Vec3d& p0, const osg::Vec3d& p1, unsigned parts, Vec3dVector& out )
{
    osg::Vec3d vec = (p1-p0)/double(parts);
    out.push_back( p0 );
    for( unsigned i=1; i<parts; ++i )
    {
        out.push_back( p0 + vec*double(i) );
    }
}

void 
TessellateOperator::tessellateGeo( const osg::Vec3d& p0, const osg::Vec3d& p1, unsigned parts, GeoInterpolation interp, Vec3dVector& out )
{
    double step = 1.0/double(parts);
    double zdelta = p1.z() - p0.z();

    out.push_back( p0 );

    GeoPoint beg(SpatialReference::get("wgs84"), p0);
    GeoPoint end(SpatialReference::get("wgs84"), p1);

    for( unsigned i=1; i<parts; ++i )
    {
        double t = step*double(i);
        osg::Vec3d p;

        if ( interp == GEOINTERP_GREAT_CIRCLE )
        {
            GeoPoint r = beg.interpolate(end, t);
            p = r.vec3d();
        }
        else // GEOINTERP_RHUMB_LINE
        {
            // rhumb lines break down at the poles to introduce a fudge factor if necessary
            double lat1 = p0.y(), lon1 = p0.x();
            double lat2 = p1.y(), lon2 = p1.x();

            lat1 = osg::clampBetween(lat1, -89.99999, 89.99999);
            lat2 = osg::clampBetween(lat2, -89.99999, 89.99999);

            lat1 = osg::DegreesToRadians(lat1), lon1 = osg::DegreesToRadians(lon1);
            lat2 = osg::DegreesToRadians(lat2), lon2 = osg::DegreesToRadians(lon2);

            double totalDistance = GeoMath::rhumbDistance( lat1, lon1, lat2, lon2 );
            double bearing = GeoMath::rhumbBearing( lat1, lon1, lat2, lon2 );

            double interpDistance = t * totalDistance;

            double lat3, lon3;
            GeoMath::rhumbDestination(lat1, lon1, bearing, interpDistance, lat3, lon3);

            p.set( osg::RadiansToDegrees(lon3), osg::RadiansToDegrees(lat3), p0.z() + t*zdelta );
        }

        out.push_back(p);
    }
}

//------------------------------------------------------------------------

TessellateOperator::TessellateOperator() :
_numPartitions( 20 ),
_defaultInterp( GEOINTERP_GREAT_CIRCLE )
{
    //nop
}

void
TessellateOperator::operator()( Feature* feature, FilterContext& context ) const
{
    if (_numPartitions <= 1 ||
        !feature || 
        !feature->getGeometry() || 
        feature->getGeometry()->getComponentType() == Geometry::TYPE_POINTSET )
    {
        return;
    }

    Units featureUnits = feature->getSRS() ? feature->getSRS()->getUnits() : Units::METERS;
    bool isGeo = feature->getSRS() ? feature->getSRS()->isGeographic() : false;
    GeoInterpolation geoInterp = feature->geoInterp().isSet() ? *feature->geoInterp() : _defaultInterp;

    double sliceSize = 0.0;
    int    numPartitions = _numPartitions;

    if ( _maxDistance.isSet() )
    {
        sliceSize = _maxDistance->as(Units::METERS);
    }

    GeometryIterator i( feature->getGeometry(), true );
    while( i.hasMore() )
    {
        Geometry* g = i.next();
        bool isRing = dynamic_cast<Ring*>( g ) != 0L;

        Vec3dVector newVerts;

        for( Geometry::const_iterator v = g->begin(); v != g->end(); ++v )
        {
            unsigned slices = _numPartitions;

            const osg::Vec3d& p0 = *v;
            if ( v != g->end()-1 ) // not last vert
            {
                // calculate slice count
                if ( sliceSize > 0.0 )
                {
                    double dist = GeoMath::distance(*v, *(v + 1), feature->getSRS());
                    slices = osg::maximum( 1u, (unsigned)(dist / sliceSize) );
                }

                if ( isGeo )
                    tessellateGeo( *v, *(v+1), slices, geoInterp, newVerts );
                else
                    tessellateLinear( *v, *(v+1), slices, newVerts );
            }
            else if ( isRing )
            {
                // calculate slice count
                if ( sliceSize > 0.0 )
                {
                    double dist = GeoMath::distance(*v, *g->begin(), feature->getSRS());
                    slices = osg::maximum( 1u, (unsigned)(dist / sliceSize) );
                }

                if ( isGeo )
                    tessellateGeo( *v, *g->begin(), slices, geoInterp, newVerts );
                else
                    tessellateLinear( *v, *g->begin(), slices, newVerts );
            }
            else 
            {
                // get the final vert.
                newVerts.push_back( *v );
            }
        }

        g->swap( newVerts );
    }
}

FilterContext
TessellateOperator::push(FeatureList& input, FilterContext& context) const
{
    for (FeatureList::iterator i = input.begin(); i != input.end(); ++i) {
        operator()(i->get(), context);
    }
    return context;
}