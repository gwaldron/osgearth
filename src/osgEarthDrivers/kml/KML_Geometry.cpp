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
#include "KML_Geometry"
#include "KML_Point"
#include "KML_LineString"
#include "KML_LinearRing"
#include "KML_Polygon"
#include "KML_MultiGeometry"
#include "KML_Model"
#include <osgEarth/StringUtils>

using namespace osgEarth_kml;

void 
KML_Geometry::build( const Config& parentConf, KMLContext& cx, Style& style)
{
    const ConfigSet& children = parentConf.children();
    for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
    {
        buildChild( *i, cx, style );
    }
}

void
KML_Geometry::buildChild( const Config& conf, KMLContext& cx, Style& style)
{
    if ( conf.key() == "point" )
    {
        KML_Point g;
        g.parseStyle(conf, cx, style);
        g.parseCoords(conf, cx);
        _geom = g._geom.get();
    }
    else if ( conf.key() == "linestring" )
    {
        KML_LineString g;
        g.parseStyle(conf, cx, style);
        g.parseCoords(conf, cx);
        _geom = g._geom.get();
        // remove polystyle since it doesn't apply
        style.remove<PolygonSymbol>();
    }
    else if ( conf.key() == "linearring" || conf.key() == "gx:latlonquad" )
    {
        KML_LinearRing g;
        g.parseStyle(conf, cx, style);
        g.parseCoords(conf, cx);
        _geom = g._geom.get();
        // remove polystyle since it doesn't apply
        style.remove<PolygonSymbol>();
    }
    else if ( conf.key() == "polygon" )
    {
        KML_Polygon g;
        g.parseStyle(conf, cx, style);
        g.parseCoords(conf, cx);
        _geom = g._geom.get();
    }
    else if ( conf.key() == "multigeometry" )
    {
        KML_MultiGeometry g;
        g.parseStyle(conf, cx, style);
        g.parseCoords(conf, cx);
        const ConfigSet& mgChildren = conf.children();
        
        for( ConfigSet::const_iterator i = mgChildren.begin(); i != mgChildren.end(); ++i )
        {
            const Config& mgChild = *i;
            Style subStyle = style;
            KML_Geometry subGeom;
            subGeom.parseStyle( mgChild, cx, subStyle );
            subGeom.buildChild( mgChild, cx, style );
            if ( subGeom._geom.valid() )
                dynamic_cast<MultiGeometry*>(g._geom.get())->getComponents().push_back( subGeom._geom.get() );
        }
        //g.parseCoords(conf.child("multigeometry"), cx);
        _geom = g._geom.get();
    }
    else if ( conf.key() == "model" )
    {
        KML_Model g;
        g.parseStyle(conf, cx, style);
        g.parseCoords(conf, cx);
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
KML_Geometry::parseStyle( const Config& conf, KMLContext& cx, Style& style )
{
    _extrude = conf.value("extrude") == "1";
    _tessellate = conf.value("tessellate") == "1";

    std::string am = conf.value("altitudemode");

    // clampToGround is the default. We will be draping the geometry UNLESS tessellate is
    // set to true.
    if ( (am.empty() || am == "clampToGround") && _tessellate )
    {
        AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
        af->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        af->technique() = AltitudeSymbol::TECHNIQUE_SCENE;
        _extrude = false;
    }

    // "relativeToGround" means the coordinates' Z values are relative to the Z of the
    // terrain at that point. NOTE: GE flattens rooftops in this mode when extrude=1,
    // which seems wrong..
    else if ( am == "relativeToGround" )
    {
        AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
        af->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
        af->technique() = AltitudeSymbol::TECHNIQUE_SCENE;
    }

    // "absolute" means to treat the Z values as-is
    else if ( am == "absolute" )
    {
        AltitudeSymbol* af = style.getOrCreate<AltitudeSymbol>();
        //af->clamping() = AltitudeSymbol::CLAMP_ABSOLUTE;
        //TODO: evaluate this..
        af->clamping().unset();
    }

    if ( _extrude )
    {
        ExtrusionSymbol* es = style.getOrCreate<ExtrusionSymbol>();
        es->flatten() = false;
    }
}
