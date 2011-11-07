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
        setMaxResolution( *_altitude->clampingResolution() );
    }
}

FilterContext
AltitudeFilter::push( FeatureList& features, FilterContext& cx )
{
    bool clamp = 
        _altitude.valid() && 
        _altitude->clamping() != AltitudeSymbol::CLAMP_NONE &&
        cx.getSession()       != 0L &&
        cx.profile()          != 0L;

    if ( clamp )
        pushAndClamp( features, cx );
    else
        pushAndDontClamp( features, cx );

    return cx;
}

void
AltitudeFilter::pushAndDontClamp( FeatureList& features, FilterContext& cx )
{
    NumericExpression scaleExpr;
    if ( _altitude.valid() && _altitude->verticalScale().isSet() )
        scaleExpr = *_altitude->verticalScale();

    NumericExpression offsetExpr;
    if ( _altitude.valid() && _altitude->verticalOffset().isSet() )
        offsetExpr = *_altitude->verticalOffset();

    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* feature = i->get();
        double maxGeomZ     = -DBL_MAX;
        double minGeomZ     =  DBL_MAX;

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
            for( Geometry::iterator i = geom->begin(); i != geom->end(); ++i )
            {
                i->z() *= scaleZ;
                i->z() += offsetZ;

                if ( i->z() > maxGeomZ )
                    maxGeomZ = i->z();
                if ( i->z() < minGeomZ )
                    minGeomZ = i->z();
            }
        }

        if ( minGeomZ != DBL_MAX )
        {
            feature->set( "__min_geom_z", minGeomZ );
            feature->set( "__max_geom_z", maxGeomZ );
        }
    }
}

void
AltitudeFilter::pushAndClamp( FeatureList& features, FilterContext& cx )
{
    const Session* session = cx.getSession();

    // the map against which we'll be doing elevation clamping
    MapFrame mapf = session->createMapFrame( Map::ELEVATION_LAYERS );

    const SpatialReference* mapSRS     = mapf.getProfile()->getSRS();
    const SpatialReference* featureSRS = cx.profile()->getSRS();

    // establish an elevation query interface based on the features' SRS.
    ElevationQuery eq( mapf );

    NumericExpression scaleExpr;
    if ( _altitude->verticalScale().isSet() )
        scaleExpr = *_altitude->verticalScale();

    NumericExpression offsetExpr;
    if ( _altitude->verticalOffset().isSet() )
        offsetExpr = *_altitude->verticalOffset();

    // whether to record a "minimum terrain" value
    bool collectHATs =
        _altitude->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN ||
        _altitude->clamping() == AltitudeSymbol::CLAMP_ABSOLUTE;

    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* feature = i->get();
        double maxGeomZ     = -DBL_MAX;
        double minGeomZ     =  DBL_MAX;
        double maxTerrainZ  = -DBL_MAX;
        double minTerrainZ  =  DBL_MAX;
        double minHAT       =  DBL_MAX;
        double maxHAT       = -DBL_MAX;

        double scaleZ = 1.0;
        if ( _altitude.valid() && _altitude->verticalScale().isSet() )
            scaleZ = feature->eval( scaleExpr, &cx );

        double offsetZ = 0.0;
        if ( _altitude.valid() && _altitude->verticalOffset().isSet() )
            offsetZ = feature->eval( offsetExpr, &cx );
        
        GeometryIterator gi( feature->getGeometry() );
        while( gi.hasMore() )
        {
            Geometry* geom = gi.next();

            // clamps the entire array to the terrain using the specified resolution.
            if ( collectHATs )
            {
                std::vector<double> elevations;
                elevations.reserve( geom->size() );
                eq.getElevations( geom->asVector(), featureSRS, elevations, _maxRes );
                for( unsigned i=0; i<geom->size(); ++i )
                {
                    double z = (*geom)[i].z() * scaleZ + offsetZ;
                    double hat =
                        _altitude->clamping() == AltitudeSymbol::CLAMP_ABSOLUTE ? z - elevations[i] :
                        z;

                    if ( hat > maxHAT )
                        maxHAT = hat;
                    if ( hat < minHAT )
                        minHAT = hat;

                    double elev = elevations[i];
                    if ( elev > maxTerrainZ )
                        maxTerrainZ = elev;
                    if ( elev < minTerrainZ )
                        minTerrainZ = elev;

                    (*geom)[i].z() =
                        _altitude->clamping() == AltitudeSymbol::CLAMP_ABSOLUTE ? z :
                        z + elevations[i];
                }
            }
            else
            {
                eq.getElevations( geom->asVector(), featureSRS, true, _maxRes );
            }

            for( Geometry::iterator i = geom->begin(); i != geom->end(); ++i )
            {
                if ( !collectHATs )
                {
                    i->z() *= scaleZ;
                    i->z() += offsetZ;
                }

                if ( i->z() > maxGeomZ )
                    maxGeomZ = i->z();
                if ( i->z() < minGeomZ )
                    minGeomZ = i->z();
            }
        }

        if ( minHAT != DBL_MAX )
        {
            feature->set( "__min_hat", minHAT );
            feature->set( "__max_hat", maxHAT );
        }

        if ( minGeomZ != DBL_MAX )
        {
            feature->set( "__min_geom_z", minGeomZ );
            feature->set( "__max_geom_z", maxGeomZ );
        }

        if ( minTerrainZ != DBL_MAX )
        {
            feature->set( "__min_terrain_z", minTerrainZ );
            feature->set( "__max_terrain_z", maxTerrainZ );
        }
    }
}
