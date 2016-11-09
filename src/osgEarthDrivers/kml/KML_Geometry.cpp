/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
KML_Geometry::build( xml_node<>* parent, KMLContext& cx, Style& style)
{
	for (xml_node<>* node = parent->first_node(); node; node = node->next_sibling())
	{
		buildChild(node, cx, style);
	}
}

void
KML_Geometry::buildChild( xml_node<>* node, KMLContext& cx, Style& style)
{
	std::string name = toLower(node->name());
    if ( name == "point" )
    {
        KML_Point g;
        g.parseCoords(node, cx);
        _geom = g._geom.get();
        g.parseStyle(node, cx, style);
    }
    else if (name == "linestring" )
    {
        KML_LineString g;
        g.parseCoords(node, cx);
        _geom = g._geom.get();
        g.parseStyle(node, cx, style);
    }
    else if ( name == "linearring" || name == "gx:latlonquad" )
    {
        KML_LinearRing g;
        g.parseCoords(node, cx);
        _geom = g._geom.get();
        g.parseStyle(node, cx, style);
    }
    else if ( name == "polygon" )
    {
        KML_Polygon g;
        g.parseCoords(node, cx);
        _geom = g._geom.get();
        g.parseStyle(node, cx, style);
    }
    else if ( name == "multigeometry" )
    {
        KML_MultiGeometry g;
        g.parseCoords(node, cx);
        _geom = g._geom.get();
        
        for( xml_node<>* n = node->first_node(); n; n = n->next_sibling())
        {
            KML_Geometry subGeom;
            subGeom.buildChild( n, cx, style ); //use single style for all subgeometries
            if ( subGeom._geom.valid() )
                dynamic_cast<MultiGeometry*>(g._geom.get())->getComponents().push_back( subGeom._geom.get() );
        }
    }
    else if ( name == "model" )
    {
        KML_Model g;
        g.parseCoords(node, cx);
        _geom = g._geom.get();
        g.parseStyle(node, cx, style);
    }
}

void
KML_Geometry::parseCoords( xml_node<>* node, KMLContext& cx )
{
    xml_node<>* coords = node->first_node("coordinates", 0, false);
    if ( coords )
    {
        StringVector tuples;
        StringTokenizer( coords->value(), tuples, " \n", "", false, true );
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
}

void
KML_Geometry::parseStyle( xml_node<>* node, KMLContext& cx, Style& style )
{
    _extrude = getValue(node, "extrude") == "1";
    _tessellate = getValue(node, "tessellate") == "1";

    std::string am = getValue(node, "altitudemode");
    if ( am.empty() )
        am = "clampToGround"; // default.

    bool isPoly = _geom.valid() && _geom->getComponentType() == Geometry::TYPE_POLYGON;
    bool isLine = _geom.valid() && _geom->getComponentType() == Geometry::TYPE_LINESTRING;

    // Resolve the correct altitude symbol. CLAMP_TO_TERRAIN is the default, but the
    // technique will depend on the geometry's type and setup.
    AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
    alt->clamping() = alt->CLAMP_TO_TERRAIN;


    // Compute some info about the geometry

    // Are all of the elevations zero?
    bool zeroElev = true;
    // Are all of the the elevations the same?
    bool sameElev = true;

    double maxElevation = -DBL_MAX;

    //if ( isPoly ) //compute maxElevation also for line strings for extrusion height
    {
        bool first = true;
        double e = 0.0;
        ConstGeometryIterator gi( _geom.get(), false );
        while(gi.hasMore() )
        {
            const Geometry* g = gi.next();
            for( Geometry::const_iterator ji = g->begin(); ji != g->end(); ++ji )
            {
                if ( !osg::equivalent(ji->z(), 0.0) )
                    zeroElev = false;

                if (first)
                {
                    first = false;
                    e = ji->z();
                }
                else
                {
                    if (!osg::equivalent(e, ji->z()))
                    {
                        sameElev = false;
                    }
                }

                if (ji->z() > maxElevation) maxElevation = ji->z();
            }
        }
    }

    // clamp to ground mode:
    if ( am == "clampToGround" )
    {
        if ( _extrude )
        {
            alt->technique() = alt->TECHNIQUE_MAP;
        }
        else if ( isPoly )
        {
            alt->technique() = alt->TECHNIQUE_DRAPE;
        }
        else if ( isLine)
        {
            alt->technique() = alt->TECHNIQUE_DRAPE; // or could be GPU.
        }
        else // line or point
        {
            alt->technique() = alt->TECHNIQUE_SCENE;
        }

        // extrusion is not compatible with clampToGround.
        _extrude = false;
    }

    // "relativeToGround" means the coordinates' Z values are relative to the Z of the
    // terrain at that point. NOTE: GE flattens rooftops in this mode when extrude=1,
    // which seems wrong..
    else if ( am == "relativeToGround" )
    {
        alt->clamping() = alt->CLAMP_RELATIVE_TO_TERRAIN;

        if (isPoly)
        {
            // If all of the verts have the same elevation then assume that it should be clamped at the centroid and not per vertex.
            if (sameElev)
            {
                alt->binding() = AltitudeSymbol::BINDING_CENTROID;
            }

            if ( _extrude )
            {
                alt->technique() = alt->TECHNIQUE_MAP;
            }
            else
            {
                alt->technique() = alt->TECHNIQUE_SCENE;

                if ( zeroElev )
                {
                    alt->clamping()  = alt->CLAMP_TO_TERRAIN;
                    alt->technique() = alt->TECHNIQUE_DRAPE;
                }
            }
        }
    }

    // "absolute" means to treat the Z values as-is
    else if ( am == "absolute" )
    {
        alt->clamping() = AltitudeSymbol::CLAMP_NONE;
    }

    if ( _extrude )
    {
        ExtrusionSymbol* es = style.getOrCreate<ExtrusionSymbol>();
        es->flatten() = false;
        if (*alt->clamping() == AltitudeSymbol::CLAMP_NONE)
        {
            // Set the height to the max elevation + the approx depth of the mariana trench so that it will extend low enough to be always go to the surface of the earth.
            // This lets us avoid clamping absolute absolute extruded polygons completely.
            es->height() = -(maxElevation + 11100.0);
        }
    }
    else
    {
        // remove polystyle since it doesn't apply to non-extruded lines and points
        if ( !isPoly )
        {
            style.remove<PolygonSymbol>();
        }
    }
}
