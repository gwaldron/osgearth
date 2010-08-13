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
#include <osgEarthSymbology/Text>
#include <osgEarth/XmlUtils>
#include <stack>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

static osg::Vec4f
htmlColorToVec4f( const std::string& html )
{
    std::string t = html;
    std::transform( t.begin(), t.end(), t.begin(), ::tolower );
    osg::Vec4ub c(0,0,0,255);
    if ( t.length() == 7 ) {
        c.r() |= t[1]<='9' ? (t[1]-'0')<<4 : (10+(t[1]-'a'))<<4;
        c.r() |= t[2]<='9' ? (t[2]-'0')    : (10+(t[2]-'a'));
        c.g() |= t[3]<='9' ? (t[3]-'0')<<4 : (10+(t[3]-'a'))<<4;
        c.g() |= t[4]<='9' ? (t[4]-'0')    : (10+(t[4]-'a'));
        c.b() |= t[5]<='9' ? (t[5]-'0')<<4 : (10+(t[5]-'a'))<<4;
        c.b() |= t[6]<='9' ? (t[6]-'0')    : (10+(t[6]-'a'));
    }
    return osg::Vec4f( ((float)c.r())/255.0f, ((float)c.g())/255.0f, ((float)c.b())/255.0f, 1.0f );
}

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

    LineSymbol* line = 0;
    PolygonSymbol* polygon = 0;
    PointSymbol* point = 0;
    TextSymbol* text = 0;
    for(Properties::const_iterator p = conf.attrs().begin(); p != conf.attrs().end(); p++ )
    {
        if ( p->first == CSS_STROKE ) {
            if (!line)
                line = new LineSymbol;
            line->stroke()->color() = htmlColorToVec4f( p->second );
        }
        else if ( p->first == CSS_STROKE_OPACITY ) {
            if (!line)
                line = new LineSymbol;
            line->stroke()->color().a() = as<float>( p->second, 1.0f );
        }
        else if ( p->first == CSS_STROKE_WIDTH ) {
            if (!line)
                line = new LineSymbol;
            line->stroke()->width() = as<float>( p->second, 1.0f );
        }
        else if ( p->first == CSS_STROKE_LINECAP ) {
            parseLineCap( p->second, line->stroke()->lineCap() );
        }
        else if ( p->first == CSS_FILL ) {
            if (!polygon)
                polygon = new PolygonSymbol;

            if (!point)
                point = new PointSymbol;

            if (!text)
                text = new TextSymbol;

            polygon->fill()->color() = htmlColorToVec4f( p->second );
            point->fill()->color() = htmlColorToVec4f( p->second );
            text->fill()->color() = htmlColorToVec4f( p->second );
        }
        else if ( p->first == CSS_FILL_OPACITY ) {
            if (!polygon)
                polygon = new PolygonSymbol;
            polygon->fill()->color().a() = as<float>( p->second, 1.0f );
        }
        else if (p->first == CSS_POINT_SIZE) {
            if (!point)
                point = new PointSymbol;
            point->size() = as<float>(p->second, 1.0f);
        }
        else if (p->first == CSS_TEXT_SIZE) {
            if (!text)
                text = new TextSymbol;
            text->size() = as<float>(p->second, 32.0f);
        }
        else if (p->first == CSS_TEXT_FONT) {
            if (!text)
                text = new TextSymbol;
            text->font() = p->second;
        }
        else if (p->first == CSS_TEXT_HALO) {
            if (!text)
                text = new TextSymbol;
            text->halo()->color() = htmlColorToVec4f( p->second );
        }
        else if (p->first == CSS_TEXT_ATTRIBUTE) {
            if (!text)
                text = new TextSymbol;
            text->attribute() = p->second;
        }
        else if (p->first == CSS_TEXT_ROTATE_TO_SCREEN) {
            if (!text)
                text = new TextSymbol;
            if (p->second == "true") text->rotateToScreen() = true;
            else if (p->second == "false") text->rotateToScreen() = false;
        }
        else if (p->first == CSS_TEXT_SIZE_MODE) {
            if (!text)
                text = new TextSymbol;
            if (p->second == "screen") text->sizeMode() = TextSymbol::SIZEMODE_SCREEN;
            else if (p->second == "object") text->sizeMode() = TextSymbol::SIZEMODE_OBJECT;
        }
        else if (p->first == CSS_TEXT_REMOVE_DUPLICATE_LABELS) {
            if (!text)
                text = new TextSymbol;
            if (p->second == "true") text->removeDuplicateLabels() = true;
            else if (p->second == "false") text->removeDuplicateLabels() = false;
        } 
        else if (p->first == CSS_TEXT_LINE_ORIENTATION) {
            if (!text)
                text = new TextSymbol;
            if (p->second == "parallel") text->lineOrientation() = TextSymbol::LINEORIENTATION_PARALLEL;
            else if (p->second == "horizontal") text->lineOrientation() = TextSymbol::LINEORIENTATION_HORIZONTAL;
            else if (p->second == "perpendicular") text->lineOrientation() = TextSymbol::LINEORIENTATION_PERPENDICULAR;
        }
        else if (p->first == CSS_TEXT_LINE_PLACEMENT) {
            if (!text)
                text = new TextSymbol;
            if (p->second == "centroid") text->linePlacement() = TextSymbol::LINEPLACEMENT_CENTROID;
            else if (p->second == "along-line") text->linePlacement() = TextSymbol::LINEPLACEMENT_ALONG_LINE;
        }
        else if (p->first == CSS_TEXT_CONTENT) {
            if (!text)
                text = new TextSymbol;
            text->content() = p->second;
        }
        else if (p->first == CSS_TEXT_CONTENT_ATTRIBUTE_DELIMITER) {
             if (!text)
                 text = new TextSymbol;
             text->contentAttributeDelimiter() = p->second;
        }
    }
    if (line)
        sc.addSymbol(line);
    if (polygon)
        sc.addSymbol(polygon);
    if (point)
        sc.addSymbol(point);
    if (text)
        sc.addSymbol(text);
    return true;
}


