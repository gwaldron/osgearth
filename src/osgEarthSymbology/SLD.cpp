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
#define CSS_TEXT_ENCODING         "text-encoding"
#define CSS_TEXT_ROTATE_TO_SCREEN "text-rotate-to-screen"
#define CSS_TEXT_SIZE_MODE        "text-size-mode"
#define CSS_TEXT_REMOVE_DUPLICATE_LABELS "text-remove-duplicate-labels"
#define CSS_TEXT_LINE_ORIENTATION "text-line-orientation"
#define CSS_TEXT_LINE_PLACEMENT   "text-line-placement"
#define CSS_TEXT_CONTENT          "text-content"
#define CSS_TEXT_ALIGN            "text-align"
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
    MarkerSymbol*    marker    = 0L;
    AltitudeSymbol*  altitude  = 0L;
    SkinSymbol*      skin      = 0L;

    for( ConfigSet::const_iterator kids = conf.children().begin(); kids != conf.children().end(); ++kids )
    {
        const Config& p = *kids;

        // ..... LineSymbol .....

        if ( p.key() == CSS_STROKE )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->color() = htmlColorToVec4f( p.value() );
        }
        else if ( p.key() == CSS_STROKE_OPACITY )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->color().a() = as<float>( p.value(), 1.0f );
        }
        else if ( p.key() == CSS_STROKE_WIDTH )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->width() = as<float>( p.value(), 1.0f );
        }
        else if ( p.key() == CSS_STROKE_LINECAP )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            parseLineCap( p.value(), line->stroke()->lineCap() );
        }

        // ..... PolygonSymbol .....

        else if ( p.key() == CSS_FILL )
        {
            if (!polygon) polygon = sc.getOrCreateSymbol<PolygonSymbol>();
            polygon->fill()->color() = htmlColorToVec4f( p.value() );

            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->fill()->color() = htmlColorToVec4f( p.value() );

            if ( !text ) text = new TextSymbol();
            text->fill()->color() = htmlColorToVec4f( p.value() );
        }
        else if ( p.key() == CSS_FILL_OPACITY )
        {
            if (!polygon) polygon = sc.getOrCreateSymbol<PolygonSymbol>();
            polygon->fill()->color().a() = as<float>( p.value(), 1.0f );

            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->fill()->color().a() = as<float>( p.value(), 1.0f );

            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->fill()->color().a() = as<float>( p.value(), 1.0f );
        }

        // ..... PointSymbol .....

        else if (p.key() == CSS_POINT_SIZE)
        {
            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->size() = as<float>(p.value(), 1.0f);
        }

        // ..... TextSymbol .....

        else if (p.key() == CSS_TEXT_SIZE)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->size() = as<float>(p.value(), 32.0f);
        }
        else if (p.key() == CSS_TEXT_FONT)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->font() = p.value();
        }
        else if (p.key() == CSS_TEXT_HALO)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->halo()->color() = htmlColorToVec4f( p.value() );
        }
        else if (p.key() == CSS_TEXT_REMOVE_DUPLICATE_LABELS)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p.value() == "true") text->removeDuplicateLabels() = true;
            else if (p.value() == "false") text->removeDuplicateLabels() = false;
        } 
        else if (p.key() == CSS_TEXT_ALIGN)
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            if      ( p.value() == "left_top" ) text->alignment() = TextSymbol::ALIGN_LEFT_TOP;
            else if ( p.value() == "left_center" ) text->alignment() = TextSymbol::ALIGN_LEFT_CENTER;
            else if ( p.value() == "left_bottom" ) text->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM;
            else if ( p.value() == "center_top"  ) text->alignment() = TextSymbol::ALIGN_CENTER_TOP;
            else if ( p.value() == "center_center" ) text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
            else if ( p.value() == "center_bottom" ) text->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
            else if ( p.value() == "right_top" ) text->alignment() = TextSymbol::ALIGN_RIGHT_TOP;
            else if ( p.value() == "right_center" ) text->alignment() = TextSymbol::ALIGN_RIGHT_CENTER;
            else if ( p.value() == "right_bottom" ) text->alignment() = TextSymbol::ALIGN_RIGHT_BOTTOM;
            else if ( p.value() == "left_base_line" ) text->alignment() = TextSymbol::ALIGN_LEFT_BASE_LINE;
            else if ( p.value() == "center_base_line" ) text->alignment() = TextSymbol::ALIGN_CENTER_BASE_LINE;
            else if ( p.value() == "right_base_line" ) text->alignment() = TextSymbol::ALIGN_RIGHT_BASE_LINE;
            else if ( p.value() == "left_bottom_base_line" ) text->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM_BASE_LINE;
            else if ( p.value() == "center_bottom_base_line" ) text->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM_BASE_LINE;
            else if ( p.value() == "right_bottom_base_line" ) text->alignment() = TextSymbol::ALIGN_RIGHT_BOTTOM_BASE_LINE;
            else if ( p.value() == "base_line" ) text->alignment() = TextSymbol::ALIGN_BASE_LINE;
        }

#if 0
        else if (p.key() == CSS_TEXT_ATTRIBUTE)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->attribute() = p.value();
        }
        else if (p.key() == CSS_TEXT_ROTATE_TO_SCREEN)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p.value() == "true") text->rotateToScreen() = true;
            else if (p.value() == "false") text->rotateToScreen() = false;
        }
        else if (p.key() == CSS_TEXT_SIZE_MODE)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p.value() == "screen") text->sizeMode() = TextSymbol::SIZEMODE_SCREEN;
            else if (p.value() == "object") text->sizeMode() = TextSymbol::SIZEMODE_OBJECT;
        }
        else if (p.key() == CSS_TEXT_REMOVE_DUPLICATE_LABELS)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p.value() == "true") text->removeDuplicateLabels() = true;
            else if (p.value() == "false") text->removeDuplicateLabels() = false;
        } 
        else if (p.key() == CSS_TEXT_LINE_ORIENTATION)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p.value() == "parallel") text->lineOrientation() = TextSymbol::LINEORIENTATION_PARALLEL;
            else if (p.value() == "horizontal") text->lineOrientation() = TextSymbol::LINEORIENTATION_HORIZONTAL;
            else if (p.value() == "perpendicular") text->lineOrientation() = TextSymbol::LINEORIENTATION_PERPENDICULAR;
        }
        else if (p.key() == CSS_TEXT_LINE_PLACEMENT)
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (p.value() == "centroid") text->linePlacement() = TextSymbol::LINEPLACEMENT_CENTROID;
            else if (p.value() == "along-line") text->linePlacement() = TextSymbol::LINEPLACEMENT_ALONG_LINE;
        }
#endif
        else if (p.key() == "text-content")
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            text->content() = StringExpression( p.value() );
        }
        else if (p.key() == "text-priority")
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->priority() = NumericExpression( p.value() );
        }
        else if (p.key() == "text-provider")
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            text->provider() = p.value();
        }
		else if (p.key() == CSS_TEXT_ENCODING)
		{
			if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
			if (p.value() == "utf-8") text->encoding() = TextSymbol::ENCODING_UTF8;
			else if (p.value() == "utf-16") text->encoding() = TextSymbol::ENCODING_UTF16;
			else if (p.value() == "utf-32") text->encoding() = TextSymbol::ENCODING_UTF32;
			else if (p.value() == "ascii") text->encoding() = TextSymbol::ENCODING_ASCII;
			else text->encoding() = TextSymbol::ENCODING_ASCII;
		}

        // ..... MarkerSymbol .....

        else if (p.key() == "marker")
        {
            if (!marker) marker = sc.getOrCreate<MarkerSymbol>();            
            marker->url() = p.value();
            marker->url()->setURIContext( conf.referrer() );
        }
        else if (p.key() == "marker-library")
        {
            if (!marker) marker = sc.getOrCreate<MarkerSymbol>();
            marker->libraryName() = StringExpression(p.value());
        }
        else if (p.key() == "marker-placement")
        {
            if (!marker) marker = sc.getOrCreate<MarkerSymbol>();
            if      (p.value() == "vertex")   marker->placement() = MarkerSymbol::PLACEMENT_VERTEX;
            else if (p.value() == "interval") marker->placement() = MarkerSymbol::PLACEMENT_INTERVAL;
            else if (p.value() == "random"  ) marker->placement() = MarkerSymbol::PLACEMENT_RANDOM;
            else if (p.value() == "centroid") marker->placement() = MarkerSymbol::PLACEMENT_CENTROID;
        }
        else if (p.key() == "marker-density")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->density() = as<float>(p.value(), 1.0f);
        }
        else if (p.key() == "marker-random-seed")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->randomSeed() = as<unsigned>(p.value(), 0);
        }
        else if (p.key() == "marker-scale")
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->scale() = NumericExpression(p.value());
        }

        // ..... ExtrusionSymbol .....
                
        else if (p.key() == "extrusion-height")
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->heightExpression() = NumericExpression(p.value());
        }
        else if (p.key() == "extrusion-flatten")
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->flatten() = as<bool>(p.value(), true);
        }
        else if (p.key() == "extrusion-wall-style")
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->wallStyleName() = p.value();
        }
        else if (p.key() == "extrusion-roof-style")
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->roofStyleName() = p.value();
        }

        // ..... AltitideSymbol .....
                
        else if (p.key() == "altitude-clamping")
        {
            if (!altitude) altitude = sc.getOrCreateSymbol<AltitudeSymbol>();
            if      (p.value() == "none"    ) altitude->clamping() = AltitudeSymbol::CLAMP_NONE;
            else if (p.value() == "terrain" ) altitude->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
            else if (p.value() == "absolute") altitude->clamping() = AltitudeSymbol::CLAMP_ABSOLUTE;
            else if (p.value() == "relative") altitude->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
        }
        else if (p.key() == "altitude-resolution")
        {
            if (!altitude) altitude = sc.getOrCreate<AltitudeSymbol>();
            altitude->clampingResolution() = as<float>( p.value(), 0.0f );
        }
        else if (p.key() == "altitude-offset")
        {
            if (!altitude) altitude = sc.getOrCreate<AltitudeSymbol>();
            altitude->verticalOffset() = NumericExpression( p.value() );
        }
        else if (p.key() == "altitude-scale")
        {
            if (!altitude) altitude = sc.getOrCreate<AltitudeSymbol>();
            altitude->verticalScale() = NumericExpression( p.value() );
        }

        // ..... SkinSymbol .....

        else if (p.key() == "skin-library")
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            if ( !p.value().empty() ) skin->libraryName() = p.value();
        }
        else if (p.key() == "skin-tags")
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->addTags( p.value() );
        }
        else if (p.key() == "skin-tiled")
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->isTiled() = as<bool>( p.value(), false );
        }
        else if (p.key() == "skin-object-height")
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->objectHeight() = as<float>( p.value(), 0.0f );
        }
        else if (p.key() == "skin-min-object-height")
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->minObjectHeight() = as<float>( p.value(), 0.0f );
        }
        else if (p.key() == "skin-max-object-height")
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->maxObjectHeight() = as<float>( p.value(), 0.0f );
        }
        else if (p.key() == "skin-random-seed")
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->randomSeed() = as<unsigned>( p.value(), 0u );
        }

    }

    return true;
}


