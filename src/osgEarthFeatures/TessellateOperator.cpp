/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthFeatures/TessellateOperator>
#include <osgEarthSymbology/Geometry>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

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

    for( unsigned i=1; i<parts; ++i )
    {
        double t = step*double(i);
        osg::Vec3d p;

        if ( interp == GEOINTERP_GREAT_CIRCLE )
        {
            double lat, lon;
            GeoMath::interpolate(
                osg::DegreesToRadians(p0.y()), osg::DegreesToRadians(p0.x()),
                osg::DegreesToRadians(p1.y()), osg::DegreesToRadians(p1.x()),
                t,
                lat, lon );
            p.set( osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), p0.z() + t*zdelta );
        }
        else // GEOINTERP_RHUMB_LINE
        {
            double lat1 = osg::DegreesToRadians(p0.y()), lon1 = osg::DegreesToRadians(p0.x());
            double lat2 = osg::DegreesToRadians(p1.y()), lon2 = osg::DegreesToRadians(p1.x());

            double totalDistance = GeoMath::rhumbDistance( lat1, lon1, lat2, lon2 );
            double bearing  = GeoMath::rhumbBearing( lat1, lon1, lat2, lon2 );

            double interpDistance = t * totalDistance;
           
            double lat3, lon3;
            GeoMath::rhumbDestination(lat1, lon1, bearing, interpDistance, lat3, lon3);

            p.set( osg::RadiansToDegrees(lon3), osg::RadiansToDegrees(lat3), p0.z() + t*zdelta );
        }

        out.push_back(p);
    }
}

//------------------------------------------------------------------------

TessellateOperator::TessellateOperator(unsigned                numPartitions,
                                       GeoInterpolation        defaultInterp ) :
_numPartitions( numPartitions ),
_defaultInterp( defaultInterp )
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

    bool isGeo = feature->getSRS() ? feature->getSRS()->isGeographic() : true;
    //bool isGeo = context.profile() ? context.profile()->getSRS()->isGeographic() : true;
    GeoInterpolation interp = feature->geoInterp().isSet() ? *feature->geoInterp() : _defaultInterp;

    GeometryIterator i( feature->getGeometry(), true );
    while( i.hasMore() )
    {
        Geometry* g = i.next();
        bool isRing = dynamic_cast<Ring*>( g ) != 0L;

        Vec3dVector newVerts;
        newVerts.reserve( g->size() * _numPartitions );

        for( Geometry::const_iterator v = g->begin(); v != g->end(); ++v )
        {
            const osg::Vec3d& p0 = *v;
            if ( v != g->end()-1 ) // not last vert
            {
                if ( isGeo )
                    tessellateGeo( *v, *(v+1), _numPartitions, interp, newVerts );
                else
                    tessellateLinear( *v, *(v+1), _numPartitions, newVerts );
            }
            else if ( isRing )
            {
                if ( isGeo )
                    tessellateGeo( *v, *g->begin(), _numPartitions, interp, newVerts );
                else
                    tessellateLinear( *v, *g->begin(), _numPartitions, newVerts );
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
