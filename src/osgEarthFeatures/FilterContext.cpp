/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthFeatures/FilterContext>

using namespace osgEarth;
using namespace osgEarth::Features;

FilterContext::FilterContext(Session*               session,
                             const FeatureProfile*  profile,
                             const GeoExtent&       workingExtent ) :
_session( session ),
_profile( profile ),
_extent( workingExtent, workingExtent ),
_isGeocentric( false )
{
    //NOP
}

FilterContext::FilterContext( const FilterContext& rhs ) :
_profile( rhs._profile.get() ),
_session( rhs._session.get() ),
_isGeocentric( rhs._isGeocentric ),
_extent( rhs._extent ),
_referenceFrame( rhs._referenceFrame ),
_inverseReferenceFrame( rhs._inverseReferenceFrame ),
_optimizerHints( rhs._optimizerHints )
{
    //nop
}

void
FilterContext::toLocal( Geometry* geom ) const
{
    if ( hasReferenceFrame() )
    {
        GeometryIterator gi( geom );
        while( gi.hasMore() )
        {
            Geometry* g = gi.next();
            for( osg::Vec3dArray::iterator i = g->begin(); i != g->end(); ++i )
                *i = *i * _referenceFrame;
        }
    }
}

void
FilterContext::toWorld( Geometry* geom ) const
{
    if ( hasReferenceFrame() )
    {
        GeometryIterator gi( geom );
        while( gi.hasMore() )
        {
            Geometry* g = gi.next();
            for( osg::Vec3dArray::iterator i = g->begin(); i != g->end(); ++i )
                *i = *i * _inverseReferenceFrame;
        }
    }
}

std::string
FilterContext::toString() const
{
    std::stringstream buf;

    buf << std::fixed
        << "CONTEXT: ["
        << "profile extent = "  << profile()->getExtent().toString()
        << ", working extent = " << extent()->toString()
        << ", geocentric = "     << osgEarth::toString(_isGeocentric)
        << "]";

    std::string str = buf.str();
    return str;
}
