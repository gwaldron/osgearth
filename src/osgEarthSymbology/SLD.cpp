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
#include <osgEarthSymbology/SLD>
#include <osgEarthSymbology/CssUtils>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/Expression>
#include <osgEarth/XmlUtils>
#include <stack>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

#define CSS_STROKE          "stroke"
#define CSS_STROKE_WIDTH    "stroke-width"
#define CSS_STROKE_OPACITY  "stroke-opacity"
#define CSS_STROKE_LINECAP  "stroke-linecap"

#define CSS_FILL           "fill"
#define CSS_FILL_OPACITY   "fill-opacity"

#define CSS_POINT_SIZE     "point-size"

#define CSS_TEXT_FONT             "text-font"
#define CSS_TEXT_SIZE             "text-size"
#define CSS_TEXT_HALO             "text-halo"
#define CSS_TEXT_ATTRIBUTE        "text-attribute"
#define CSS_TEXT_ROTATE_TO_SCREEN "text-rotate-to-screen"
#define CSS_TEXT_SIZE_MODE        "text-size-mode"
#define CSS_TEXT_REMOVE_DUPLICATE_LABELS "text-remove-duplicate-labels"
#define CSS_TEXT_LINE_ORIENTATION "text-line-orientation"
#define CSS_TEXT_LINE_PLACEMENT   "text-line-placement"
#define CSS_TEXT_CONTENT          "text-content"
#define CSS_TEXT_CONTENT_ATTRIBUTE_DELIMITER "text-content-attribute-delimiter"


static void
parseLineCap( const std::string& value, optional<Stroke::LineCapStyle>& cap )
{
    if ( value == "butt" ) cap = Stroke::LINECAP_BUTT;
    if ( value == "round" ) cap = Stroke::LINECAP_ROUND;
    if ( value == "square" ) cap = Stroke::LINECAP_SQUARE;
}

bool
SLDReader::readStyleFromCSSParams( const Config& conf, Style& sc )
{
    sc.setName( conf.key() );

    LineSymbol*      line      = 0L;
    PolygonSymbol*   polygon   = 0L;
    PointSymbol*     point     = 0L;
    TextSymbol*      text      = 0L;
    ExtrusionSymbol* extrusion = 0L;
    MarkerSymbol*    marker     = 0L;
    AltitudeSymbol*  altitude  = 0L;

    for(Properties::const_iterator p = conf.attrs().begin(); p != conf.attrs().end(); p++ )
    {
        if ( p->first == CSS_STROKE )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->color() = htmlColorToVec4f( p->second );
        }
        else if ( p->first == CSS_STROKE_OPACITY )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->color().a() = as<float>( p->second, 1.0f );
        }
        else if ( p->first == CSS_STROKE_WIDTH )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->width() = as<float>( p->second, 1.0f );
        }
        else if ( p->first == CSS_STROKE_LINECAP )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            parseLineCap( p->second, line->stroke()->lineCap() );
        }
        else if ( p->first == CSS_FILL )
        {
            if (!polygon) polygon = sc.getOrCreateSymbol<PolygonSymbol>();
            polygon->fill()->color() = htmlColorToVec4f( p->second );

            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->fill()->color() = htmlColorToVec4f( p->second );

            if ( !text ) text = new TextSymbol();
            text->fill()->color() = htmlColorToVec4f( p->second );
        }
        else if ( p->first == CSS_FILL_OPACITY )
        {
            if (!polygon) polygon = sc.getOrCreateSymbol<PolygonSymbol>();
            polygon->fill()->color().a() = as<float>( p->second, 1.0f );

            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->fill()->color().a() = as<float>( p->second, 1.0f );

            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->fill()->color().a() = as<float>( p->second, 1.0f );
        }
        else if (p->first == CSS_POINT_SIZE)
        {
            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->size() = as<float>(p->second, 1.0f);
        }
        else if (p->first == CSS_TEXT_SIZE)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->size() = as<float>(p->second, 32.0f);
        }
        else if (p->first == CSS_TEXT_FONT)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->font() = p->second;
        }
        else if (p->first == CSS_TEXT_HALO)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->halo()->color() = htmlColorToVec4f( p->second );
        }
        //else if (p->first == CSS_TEXT_ATTRIBUTE)
        //{
        //    if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
        //    text->attribute() = p->second;
        //}
        else if (p->first == CSS_TEXT_ROTATE_TO_SCREEN)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p->second == "true") text->rotateToScreen() = true;
            else if (p->second == "false") text->rotateToScreen() = false;
        }
        else if (p->first == CSS_TEXT_SIZE_MODE)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p->second == "screen") text->sizeMode() = TextSymbol::SIZEMODE_SCREEN;
            else if (p->second == "object") text->sizeMode() = TextSymbol::SIZEMODE_OBJECT;
        }
        else if (p->first == CSS_TEXT_REMOVE_DUPLICATE_LABELS)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p->second == "true") text->removeDuplicateLabels() = true;
            else if (p->second == "false") text->removeDuplicateLabels() = false;
        } 
        else if (p->first == CSS_TEXT_LINE_ORIENTATION)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p->second == "parallel") text->lineOrientation() = TextSymbol::LINEORIENTATION_PARALLEL;
            else if (p->second == "horizontal") text->lineOrientation() = TextSymbol::LINEORIENTATION_HORIZONTAL;
            else if (p->second == "perpendicular") text->lineOrientation() = TextSymbol::LINEORIENTATION_PERPENDICULAR;
        }
        else if (p->first == CSS_TEXT_LINE_PLACEMENT)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p->second == "centroid") text->linePlacement() = TextSymbol::LINEPLACEMENT_CENTROID;
            else if (p->second == "along-line") text->linePlacement() = TextSymbol::LINEPLACEMENT_ALONG_LINE;
        }
        else if (p->first == "text-content")
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            text->content() = StringExpression( p->second );
        }
        else if (p->first == "text-priority")
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->priority() = NumericExpression( p->second );
        }
        else if (p->first == "text-provider")
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            text->provider() = p->second;
        }

        //else if (p->first == CSS_TEXT_CONTENT)
        //{
        //    if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
        //    text->content() = p->second;
        //}
        //else if (p->first == CSS_TEXT_CONTENT_ATTRIBUTE_DELIMITER)
        //{
        //    if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
        //    text->contentAttributeDelimiter() = p->second;
        //}

        else if (p->first == "marker")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->url() = p->second;
        }
        else if (p->first == "marker-placement")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            if      (p->second == "centroid") marker->placement() = MarkerSymbol::PLACEMENT_CENTROID;
            else if (p->second == "interval") marker->placement() = MarkerSymbol::PLACEMENT_INTERVAL;
            else if (p->second == "random"  ) marker->placement() = MarkerSymbol::PLACEMENT_RANDOM;
        }
        else if (p->first == "marker-density")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->density() = as<float>(p->second, 1.0f);
        }
        else if (p->first == "marker-random-seed")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->randomSeed() = as<unsigned>(p->second, 0);
        }
        else if (p->first == "marker-scale")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->scale() = stringToVec3f(p->second, osg::Vec3f(1,1,1));
        }

        else if (p->first == "extrusion-height")
        {
            if (!extrusion) extrusion = sc.getOrCreateSymbol<ExtrusionSymbol>();
            extrusion->heightExpression() = NumericExpression(p->second);
        }
        else if ( p->first == "extrusion-reference")
        {
            if (!extrusion) extrusion = sc.getOrCreateSymbol<ExtrusionSymbol>();
            if      ( p->second == "z"   ) extrusion->heightReference() = ExtrusionSymbol::HEIGHT_REFERENCE_Z;
            else if ( p->second == "msl" ) extrusion->heightReference() = ExtrusionSymbol::HEIGHT_REFERENCE_MSL;
        }
        else if (p->first == "extrusion-flatten")
        {
            if (!extrusion) extrusion = sc.getOrCreateSymbol<ExtrusionSymbol>();
            extrusion->flatten() = as<bool>(p->second, true);
        }
                
        else if (p->first == "altitude-clamping")
        {
            if (!altitude) altitude = sc.getOrCreateSymbol<AltitudeSymbol>();
            if      (p->second == "none"    ) altitude->clamping() = AltitudeSymbol::CLAMP_NONE;
            else if (p->second == "terrain" ) altitude->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
            else if (p->second == "relative") altitude->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
        }
        else if (p->first == "altitude-offset")
        {
            if (!altitude) altitude = sc.getOrCreateSymbol<AltitudeSymbol>();
            altitude->verticalOffset() = as<float>( p->second, 0.0f );
        }
    }

#if 0
    if (line)
        sc.addSymbol(line);
    if (polygon)
        sc.addSymbol(polygon);
    if (point)
        sc.addSymbol(point);
    if (text)
        sc.addSymbol(text);
    if (extrusion)
        sc.addSymbol(extrusion);
    if (marker)
        sc.addSymbol(marker);
    if (altitude)
        sc.addSymbol(altitude);
#endif

    return true;
}


