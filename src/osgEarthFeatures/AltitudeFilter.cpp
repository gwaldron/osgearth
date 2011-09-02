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
#include <osgEarthFeatures/AltitudeFilter>
#include <osgEarth/ElevationQuery>
#include <osgEarth/GeoData>

#define LC "[AltitudeFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

AltitudeFilter::AltitudeFilter() :
_ignoreZ( false ),
_maxRes ( 0.0f )
{
    //NOP
}

void
AltitudeFilter::setPropertiesFromStyle( const Style& style )
{
    _altitude = style.get<AltitudeSymbol>();

    if ( _altitude )
    {
        setIgnoreZ( _altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
        setMaxResolution( *_altitude->clampingResolution() );

        if ( _altitude->clamping() == AltitudeSymbol::CLAMP_ABSOLUTE )
        {
            _minZAttr = "__min_z";
            _maxZAttr = "__max_z";
        }
    }
}

FilterContext
AltitudeFilter::push( FeatureList& features, FilterContext& cx )
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

    // establish an elevation query interface based on the features' SRS.
    ElevationQuery eq( mapf );

    NumericExpression scaleExpr;
    if ( _altitude.valid() && _altitude->verticalScale().isSet() )
        scaleExpr = *_altitude->verticalScale();

    NumericExpression offsetExpr;
    if ( _altitude.valid() && _altitude->verticalOffset().isSet() )
        offsetExpr = *_altitude->verticalOffset();

    bool clamp =
        _altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN ||
        _altitude->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;

    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* feature = i->get();
        double maxZ = -DBL_MAX;
        double minZ = DBL_MAX;

        double scaleZ = 1.0;
        if ( _altitude.valid() && _altitude->verticalScale().isSet() )
            scaleZ = feature->eval( scaleExpr );

        double offsetZ = 0.0;
        if ( _altitude.valid() && _altitude->verticalOffset().isSet() )
            offsetZ = feature->eval( offsetExpr );
        
        GeometryIterator gi( feature->getGeometry() );
        while( gi.hasMore() )
        {
            Geometry* geom = gi.next();

            // clamps the entire array to the terrain using the specified resolution.
            if ( clamp )
            {
                eq.getElevations( geom->asVector(), featureSRS, _ignoreZ, _maxRes );
            }

            if ( scaleZ != 1.0 || offsetZ != 0.0 || !_maxZAttr.empty() )
            {
                for( Geometry::iterator i = geom->begin(); i != geom->end(); ++i )
                {
                    i->z() *= scaleZ;
                    i->z() += offsetZ;

                    if ( i->z() > maxZ )
                        maxZ = i->z();
                    if ( i->z() < minZ )
                        minZ = i->z();
                }
            }
        }

        if ( !_minZAttr.empty() )
            feature->set( _minZAttr, minZ );
        if ( !_maxZAttr.empty() )
            feature->set( _maxZAttr, maxZ );
    }

    return cx;
}
