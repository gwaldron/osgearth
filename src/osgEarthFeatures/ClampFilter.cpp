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
#include <osgEarthFeatures/ClampFilter>
#include <osgEarth/ElevationQuery>
#include <osgEarth/GeoData>

#define LC "[ClampFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

ClampFilter::ClampFilter() :
_ignoreZ( false )
{
    //NOP
}

FilterContext
ClampFilter::push( FeatureList& features, FilterContext& cx )
{
    const Session* session = cx.getSession();
    if ( !session ) {
        OE_WARN << LC << "No session - session is required for elevation clamping" << std::endl;
        return cx;
    }

    // the map against which we'll be doing elevation clamping
    MapFrame mapf = session->createMapFrame( Map::ELEVATION_LAYERS );

    const SpatialReference* mapSRS     = mapf.getProfile()->getSRS();
    const SpatialReference* featureSRS = cx.profile()->getSRS();
    bool isGeocentric = session->getMapInfo().isGeocentric();

    // establish an elevation query interface based on the features' SRS.
    ElevationQuery eq( mapf );

    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* feature = i->get();
        double maxZ = -DBL_MAX;
        
        GeometryIterator gi( feature->getGeometry() );
        while( gi.hasMore() )
        {
            Geometry* geom = gi.next();

            if ( isGeocentric )
            {
                // convert to map coords:
                cx.toWorld( geom );
                mapSRS->transformFromECEF( geom );

                // populate the elevations:
                eq.getElevations( geom, mapSRS );

                // find the maximum Z value
                if ( !_maxZAttrName.empty() )
                {
                    for( Geometry::const_iterator i = geom->begin(); i != geom->end(); ++i )
                    {
                        if ( i->z() > maxZ )
                            maxZ = i->z();
                    }
                }

                // convert back to geocentric:
                mapSRS->transformToECEF( geom );
                cx.toLocal( geom );
            }

            else
            {
                // clamps the entire array to the highest available resolution.
                eq.getElevations( geom, featureSRS );
            }
        }

        feature->set( _maxZAttrName, maxZ );
    }

    return cx;
}
