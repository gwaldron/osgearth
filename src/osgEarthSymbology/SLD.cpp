/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

namespace
{
    void parseLineCap( const std::string& value, optional<Stroke::LineCapStyle>& cap )
    {
        if ( value == "butt" ) cap = Stroke::LINECAP_BUTT;
        if ( value == "round" ) cap = Stroke::LINECAP_ROUND;
        if ( value == "square" ) cap = Stroke::LINECAP_SQUARE;
    }

    bool match(const std::string& s, const char* reservedWord )
    {
        if ( s == reservedWord ) return true;
        std::string temp1 = toLower(s), temp2 = toLower(reservedWord);
        replaceIn(temp1, "_", "-");
        replaceIn(temp2, "_", "-");
        return temp1 == temp2;
    }
}


bool
SLDReader::readStyleFromCSSParams( const Config& conf, Style& sc )
{
    sc.setName( conf.key() );

    IconSymbol*      icon      = 0L;
    LineSymbol*      line      = 0L;
    PolygonSymbol*   polygon   = 0L;
    PointSymbol*     point     = 0L;
    TextSymbol*      text      = 0L;
    ExtrusionSymbol* extrusion = 0L;
    MarkerSymbol*    marker    = 0L;
    ModelSymbol*     model     = 0L;
    AltitudeSymbol*  altitude  = 0L;
    SkinSymbol*      skin      = 0L;

    for( ConfigSet::const_iterator kids = conf.children().begin(); kids != conf.children().end(); ++kids )
    {
        const Config& p = *kids;

        const std::string& key   = p.key();
        const std::string& value = p.value();

        // ..... LineSymbol .....

        if ( match(key, "stroke") )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->color() = Color(value); //htmlColorToVec4f( value );
        }
        else if ( match(key, "stroke-opacity") )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->color().a() = as<float>( value, 1.0f );
        }
        else if ( match(key, "stroke-width") )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            line->stroke()->width() = as<float>( value, 1.0f );
        }
        else if ( match(key, "stroke-linecap") )
        {
            if (!line) line = sc.getOrCreateSymbol<LineSymbol>();
            parseLineCap( value, line->stroke()->lineCap() );
        }
        else if ( match(key, "stroke-tessellation") )
        {
            if (!line) line = sc.getOrCreate<LineSymbol>();
            line->tessellation() = as<unsigned>( value, 0 );
        }

        // ..... PolygonSymbol .....

        else if ( match(key, "fill") )
        {
            if (!polygon) polygon = sc.getOrCreateSymbol<PolygonSymbol>();
            polygon->fill()->color() = htmlColorToVec4f( value );

            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->fill()->color() = htmlColorToVec4f( value );

            if ( !text ) text = new TextSymbol();
            text->fill()->color() = htmlColorToVec4f( value );
        }
        else if ( match(key, "fill-opacity") )
        {
            if (!polygon) polygon = sc.getOrCreateSymbol<PolygonSymbol>();
            polygon->fill()->color().a() = as<float>( value, 1.0f );

            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->fill()->color().a() = as<float>( value, 1.0f );

            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->fill()->color().a() = as<float>( value, 1.0f );
        }

        // ..... PointSymbol .....

        else if ( match(key, "point-size") )
        {
            if ( !point ) point = sc.getOrCreateSymbol<PointSymbol>();
            point->size() = as<float>(value, 1.0f);
        }

        // ..... TextSymbol .....

        else if ( match(key, "text-size") )
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->size() = as<float>(value, 32.0f);
        }
        else if ( match(key, "text-font") )
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->font() = value;
        }
        else if ( match(key, "text-halo") )
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->halo()->color() = htmlColorToVec4f( value );
        }
        else if ( match(key, "text-halo-offset") )
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->haloOffset() = as<float>(value, 0.07f);
        }
        else if ( match(key, "text-remove-duplicate-labels") )
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if (value == "true") text->removeDuplicateLabels() = true;
            else if (value == "false") text->removeDuplicateLabels() = false;
        } 
        else if ( match(key, "text-align") )
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            if      ( match(value, "left-top") ) text->alignment() = TextSymbol::ALIGN_LEFT_TOP;
            else if ( match(value, "left-center") ) text->alignment() = TextSymbol::ALIGN_LEFT_CENTER;
            else if ( match(value, "left-bottom") ) text->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM;
            else if ( match(value, "center-top")  ) text->alignment() = TextSymbol::ALIGN_CENTER_TOP;
            else if ( match(value, "center-center") ) text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
            else if ( match(value, "center-bottom") ) text->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
            else if ( match(value, "right-top") ) text->alignment() = TextSymbol::ALIGN_RIGHT_TOP;
            else if ( match(value, "right-center") ) text->alignment() = TextSymbol::ALIGN_RIGHT_CENTER;
            else if ( match(value, "right-bottom") ) text->alignment() = TextSymbol::ALIGN_RIGHT_BOTTOM;
            else if ( match(value, "left-base-line") ) text->alignment() = TextSymbol::ALIGN_LEFT_BASE_LINE;
            else if ( match(value, "center-base-line") ) text->alignment() = TextSymbol::ALIGN_CENTER_BASE_LINE;
            else if ( match(value, "right-base-line") ) text->alignment() = TextSymbol::ALIGN_RIGHT_BASE_LINE;
            else if ( match(value, "left-bottom-base-line") ) text->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM_BASE_LINE;
            else if ( match(value, "center-bottom-base-line") ) text->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM_BASE_LINE;
            else if ( match(value, "right-bottom-base-line") ) text->alignment() = TextSymbol::ALIGN_RIGHT_BOTTOM_BASE_LINE;
            else if ( match(value, "base-line" ) ) text->alignment() = TextSymbol::ALIGN_BASE_LINE;
        }
        else if ( match(key, "text-content") )
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            text->content() = StringExpression( value );
        }
        else if ( match(key, "text-priority") )
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            text->priority() = NumericExpression( value );
        }
        else if ( match(key, "text-provider") )
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            text->provider() = value;
        }
        else if ( match(key, "text-encoding") )
        {
            if (!text) text = sc.getOrCreateSymbol<TextSymbol>();
            if      (match(value, "utf-8"))  text->encoding() = TextSymbol::ENCODING_UTF8;
            else if (match(value, "utf-16")) text->encoding() = TextSymbol::ENCODING_UTF16;
            else if (match(value, "utf-32")) text->encoding() = TextSymbol::ENCODING_UTF32;
            else if (match(value, "ascii"))  text->encoding() = TextSymbol::ENCODING_ASCII;
            else text->encoding() = TextSymbol::ENCODING_ASCII;
        }
        else if ( match(key, "text-declutter") )
        {
            if (!text) text = sc.getOrCreate<TextSymbol>();
            text->declutter() = as<bool>(value, true);
        }

        // ..... MarkerSymbol .....

        else if ( match(key, "marker") )
        {
            if (!marker) marker = sc.getOrCreate<MarkerSymbol>();            
            marker->url() = value;
            marker->url()->setURIContext( conf.referrer() );
        }
        else if ( match(key,"marker-library") )
        {
            if (!marker) marker = sc.getOrCreate<MarkerSymbol>();
            marker->libraryName() = StringExpression(value);
        }
        else if ( match(key, "marker-placement") )
        {
            if (!marker) marker = sc.getOrCreate<MarkerSymbol>();
            if      ( match(value, "vertex") )   marker->placement() = MarkerSymbol::PLACEMENT_VERTEX;
            else if ( match(value, "interval") ) marker->placement() = MarkerSymbol::PLACEMENT_INTERVAL;
            else if ( match(value, "random") )   marker->placement() = MarkerSymbol::PLACEMENT_RANDOM;
            else if ( match(value, "centroid") ) marker->placement() = MarkerSymbol::PLACEMENT_CENTROID;
        }
        else if ( match(key, "marker-density") )
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->density() = as<float>(value, 1.0f);
        }
        else if ( match(key, "marker-random-seed") )
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->randomSeed() = as<unsigned>(value, 0);
        }
        else if ( match(key, "marker-scale") )
        {
            if (!marker) marker = sc.getOrCreateSymbol<MarkerSymbol>();
            marker->scale() = NumericExpression(value);
        }
        else if ( match(key, "marker-icon-align") )
        {
            if (!marker) marker = sc.getOrCreate<MarkerSymbol>();
            if      ( match(value, "left-top") ) marker->alignment() = MarkerSymbol::ALIGN_LEFT_TOP;
            else if ( match(value, "left-center") ) marker->alignment() = MarkerSymbol::ALIGN_LEFT_CENTER;
            else if ( match(value, "left-bottom") ) marker->alignment() = MarkerSymbol::ALIGN_LEFT_BOTTOM;
            else if ( match(value, "center-top")  ) marker->alignment() = MarkerSymbol::ALIGN_CENTER_TOP;
            else if ( match(value, "center-center") ) marker->alignment() = MarkerSymbol::ALIGN_CENTER_CENTER;
            else if ( match(value, "center-bottom") ) marker->alignment() = MarkerSymbol::ALIGN_CENTER_BOTTOM;
            else if ( match(value, "right-top") ) marker->alignment() = MarkerSymbol::ALIGN_RIGHT_TOP;
            else if ( match(value, "right-center") ) marker->alignment() = MarkerSymbol::ALIGN_RIGHT_CENTER;
            else if ( match(value, "right-bottom") ) marker->alignment() = MarkerSymbol::ALIGN_RIGHT_BOTTOM;
        }

        // ..... ModelSymbol .....

        else if ( match(key, "model") )
        {
            if (!model) model = sc.getOrCreate<ModelSymbol>();
            model->url() = value;
            model->url()->setURIContext( conf.referrer() );
        }
        else if ( match(key, "model-placement") )
        {
            if (!model) model = sc.getOrCreate<ModelSymbol>();
            if      ( match(value, "vertex") )   model->placement() = ModelSymbol::PLACEMENT_VERTEX;
            else if ( match(value, "interval") ) model->placement() = ModelSymbol::PLACEMENT_INTERVAL;
            else if ( match(value, "random") )   model->placement() = ModelSymbol::PLACEMENT_RANDOM;
            else if ( match(value, "centroid") ) model->placement() = ModelSymbol::PLACEMENT_CENTROID;
        }
        else if ( match(key, "model-density") )
        {
            if (!model) model = sc.getOrCreate<ModelSymbol>();
            model->density() = as<float>(value, 1.0f);
        }
        else if ( match(key, "model-random-seed") )
        {
            if (!model) model = sc.getOrCreate<ModelSymbol>();
            model->randomSeed() = as<unsigned>(value, 0);
        }
        else if ( match(key, "model-scale") )
        {
            if (!model) model = sc.getOrCreate<ModelSymbol>();
            model->scale() = NumericExpression(value);
        }
        else if ( match(key, "model-heading") )
        {
            if (!model) model = sc.getOrCreate<ModelSymbol>();
            model->heading() = NumericExpression(value);
        }

        // ..... IconSymbol .....

        else if ( match(key, "icon") )
        {
            if (!icon) icon = sc.getOrCreate<IconSymbol>();
            icon->url() = value;
            icon->url()->setURIContext( conf.referrer() );
        }
        else if ( match(key,"icon-library") )
        {
            if (!icon) icon = sc.getOrCreate<IconSymbol>();
            icon->libraryName() = StringExpression(value);
        }
        else if ( match(key, "icon-placement") )
        {
            if (!icon) icon = sc.getOrCreate<IconSymbol>();
            if      ( match(value, "vertex") )   icon->placement() = ModelSymbol::PLACEMENT_VERTEX;
            else if ( match(value, "interval") ) icon->placement() = ModelSymbol::PLACEMENT_INTERVAL;
            else if ( match(value, "random") )   icon->placement() = ModelSymbol::PLACEMENT_RANDOM;
            else if ( match(value, "centroid") ) icon->placement() = ModelSymbol::PLACEMENT_CENTROID;
        }
        else if ( match(key, "icon-density") )
        {
            if (!icon) icon = sc.getOrCreate<IconSymbol>();
            icon->density() = as<float>(value, 1.0f);
        }
        else if ( match(key, "icon-random-seed") )
        {
            if (!icon) icon = sc.getOrCreate<IconSymbol>();
            icon->randomSeed() = as<unsigned>(value, 0);
        }
        else if ( match(key, "icon-scale") )
        {
            if (!icon) icon = sc.getOrCreate<IconSymbol>();
            icon->scale() = NumericExpression(value);
        }
        else if ( match(key, "icon-align") )
        {
            if (!icon) icon = sc.getOrCreate<IconSymbol>();
            if      ( match(value, "left-top") )      icon->alignment() = IconSymbol::ALIGN_LEFT_TOP;
            else if ( match(value, "left-center") )   icon->alignment() = IconSymbol::ALIGN_LEFT_CENTER;
            else if ( match(value, "left-bottom") )   icon->alignment() = IconSymbol::ALIGN_LEFT_BOTTOM;
            else if ( match(value, "center-top")  )   icon->alignment() = IconSymbol::ALIGN_CENTER_TOP;
            else if ( match(value, "center-center") ) icon->alignment() = IconSymbol::ALIGN_CENTER_CENTER;
            else if ( match(value, "center-bottom") ) icon->alignment() = IconSymbol::ALIGN_CENTER_BOTTOM;
            else if ( match(value, "right-top") )     icon->alignment() = IconSymbol::ALIGN_RIGHT_TOP;
            else if ( match(value, "right-center") )  icon->alignment() = IconSymbol::ALIGN_RIGHT_CENTER;
            else if ( match(value, "right-bottom") )  icon->alignment() = IconSymbol::ALIGN_RIGHT_BOTTOM;
        }

        // ..... ExtrusionSymbol .....
                
        else if ( match(key, "extrusion-height") )
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->heightExpression() = NumericExpression(value);
        }
        else if ( match(key, "extrusion-flatten") )
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->flatten() = as<bool>(value, true);
        }
        else if ( match(key, "extrusion-wall-style") )
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->wallStyleName() = value;
        }
        else if ( match(key, "extrusion-roof-style") )
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->roofStyleName() = value;
        }
        else if ( match(key, "extrusion-wall-gradient") )
        {
            if (!extrusion) extrusion = sc.getOrCreate<ExtrusionSymbol>();
            extrusion->wallGradientPercentage() = as<float>(value, 0.0f);
        }

        // ..... AltitideSymbol .....
                
        else if ( match(key, "altitude-clamping") )
        {
            if (!altitude) altitude = sc.getOrCreateSymbol<AltitudeSymbol>();
            if      ( match(value, "none") )     altitude->clamping() = AltitudeSymbol::CLAMP_NONE;
            else if ( match(value, "terrain") )  altitude->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
            else if ( match(value, "absolute") ) altitude->clamping() = AltitudeSymbol::CLAMP_ABSOLUTE;
            else if ( match(value, "relative") ) altitude->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
        }
        else if ( match(key, "altitude-resolution") )
        {
            if (!altitude) altitude = sc.getOrCreate<AltitudeSymbol>();
            altitude->clampingResolution() = as<float>( value, 0.0f );
        }
        else if ( match(key, "altitude-offset") )
        {
            if (!altitude) altitude = sc.getOrCreate<AltitudeSymbol>();
            altitude->verticalOffset() = NumericExpression( value );
        }
        else if ( match(key, "altitude-scale") )
        {
            if (!altitude) altitude = sc.getOrCreate<AltitudeSymbol>();
            altitude->verticalScale() = NumericExpression( value );
        }

        // ..... SkinSymbol .....

        else if ( match(key, "skin-library") )
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            if ( !value.empty() ) skin->libraryName() = value;
        }
        else if ( match(key, "skin-tags") )
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->addTags( value );
        }
        else if ( match(key, "skin-tiled") )
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->isTiled() = as<bool>( value, false );
        }
        else if ( match(key, "skin-object-height") )
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->objectHeight() = as<float>( value, 0.0f );
        }
        else if (match(key, "skin-min-object-height") )
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->minObjectHeight() = as<float>( value, 0.0f );
        }
        else if (match(key, "skin-max-object-height") )
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->maxObjectHeight() = as<float>( value, 0.0f );
        }
        else if (match(key, "skin-random-seed") )
        {
            if (!skin) skin = sc.getOrCreate<SkinSymbol>();
            skin->randomSeed() = as<unsigned>( value, 0u );
        }

    }

    return true;
}


