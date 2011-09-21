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
#include "KML_Geometry"
#include "KML_Point"
#include "KML_LineString"
#include "KML_LinearRing"
#include "KML_Polygon"
#include "KML_MultiGeometry"
#include "KML_Model"
#include <osgEarth/StringUtils>

void
KML_Geometry::build( const Config& conf, KMLContext& cx, Style& style)
{
    if ( conf.hasChild("point") ) {
        KML_Point g;
        g.parseStyle(conf.child("point"), cx, style);
        g.parseCoords(conf.child("point"), cx);
        _geom = g._geom.get();
    }
    else if ( conf.hasChild("linestring") ) {
        KML_LineString g;
        g.parseStyle(conf.child("linestring"), cx, style);
        g.parseCoords(conf.child("linestring"), cx);
        _geom = g._geom.get();
    }
    else if ( conf.hasChild("linearring") ) {
        KML_LinearRing g;
        g.parseStyle(conf.child("linearring"), cx, style);
        g.parseCoords(conf.child("linearring"), cx);
        _geom = g._geom.get();
    }
    else if ( conf.hasChild("polygon") ) {
        KML_Polygon g;
        g.parseStyle(conf.child("polygon"), cx, style);
        g.parseCoords(conf.child("polygon"), cx);
        _geom = g._geom.get();
    }
    else if ( conf.hasChild("multigeometry") ) {
        KML_MultiGeometry g;
        g.parseStyle(conf.child("multigeometry"), cx, style);
        g.parseCoords(conf.child("multigeometry"), cx);
        _geom = g._geom.get();
    }
    else if ( conf.hasChild("model") ) {
        KML_Model g;
        g.parseStyle(conf.child("model"), cx, style);
        g.parseCoords(conf.child("model"), cx);
        _geom = g._geom.get();
    }
}

void
KML_Geometry::parseCoords( const Config& conf, KMLContext& cx )
{
    const Config& coords = conf.child("coordinates");
    StringVector tuples;
    StringTokenizer( coords.value(), tuples, " ", "", false, true );
    for( StringVector::const_iterator s=tuples.begin(); s != tuples.end(); ++s )
    {
        StringVector parts;
        StringTokenizer( *s, parts, ",", "", false, true );
        if ( parts.size() >= 2 )
        {
            osg::Vec3d point;
            point.x() = as<double>( parts[0], 0.0 );
            point.y() = as<double>( parts[1], 0.0 );
            if ( parts.size() >= 3 ) {
                point.z() = as<double>( parts[2], 0.0 );
            }
            _geom->push_back(point);
        }
    }
}

void
KML_Geometry::parseStyle( const Config& conf, KMLContext& cs, Style& style )
{
    _extrude = conf.value("extrude") == "1";
    _tessellate = conf.value("tessellate") == "1";

    std::string am = conf.value("altitudemode");
    if ( am.empty() || am == "clampToGround" )
    {
        AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
        af->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        _extrude = false;
    }
    else if ( am == "relativeToGround" )
    {
        AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
        af->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
    }
    else if ( am == "absolute" )
    {
        AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
        af->clamping() = AltitudeSymbol::CLAMP_ABSOLUTE;
    }

    if ( _extrude )
    {
        ExtrusionSymbol* es = style.getOrCreate<ExtrusionSymbol>();
        es->flatten() = false;
    }
}
