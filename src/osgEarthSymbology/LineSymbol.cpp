/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

namespace
{
    std::string stripQuotes(const std::string& s) {
        bool q0 = (s.length() > 0 && (s[0] == '\"' || s[0] == '\''));
        bool q1 = (s.length() > 1 && (s[s.length()-1] == '\"' || s[s.length()-1] == '\''));
        if (q0 && q1) 
            return s.substr(1, s.length()-2);
        else if (q0)
            return s.substr(1);
        else if (q1)
            return s.substr(0, s.length()-1);
        else
            return s;
    }
}

OSGEARTH_REGISTER_SIMPLE_SYMBOL(line, LineSymbol);

LineSymbol::LineSymbol( const Config& conf ) :
Symbol       ( conf ),
_stroke      ( Stroke() ),
_tessellation( 0 ),
_creaseAngle ( 0.0f )
{
    mergeConfig(conf);
}

LineSymbol::LineSymbol(const LineSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_stroke          (rhs._stroke),
_tessellation    (rhs._tessellation),
_creaseAngle     (rhs._creaseAngle),
_tessellationSize(rhs._tessellationSize),
_imageURI        (rhs._imageURI)
{
    //nop
}

Config 
LineSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "line";
    conf.set("stroke",       _stroke);
    conf.set("tessellation", _tessellation);
    conf.set("crease_angle", _creaseAngle);
    conf.set("tessellation_size", _tessellationSize );
    conf.set("image", _imageURI);
    return conf;
}

void 
LineSymbol::mergeConfig( const Config& conf )
{
    conf.get("stroke",       _stroke);
    conf.get("tessellation", _tessellation);
    conf.get("crease_angle", _creaseAngle);
    conf.get("tessellation_size", _tessellationSize);
    conf.get("image", _imageURI);
}

void
LineSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "stroke") ) {
        style.getOrCreate<LineSymbol>()->stroke()->color() = Color(c.value());
    }
    else if ( match(c.key(), "stroke-opacity") ) {
        style.getOrCreate<LineSymbol>()->stroke()->color().a() = as<float>( c.value(), 1.0f );
    }
    else if ( match(c.key(), "stroke-width") ) {
        float width;
        Units units;
        if ( Units::parse(c.value(), width, units, Units::PIXELS) )
        {
            style.getOrCreate<LineSymbol>()->stroke()->width() = width;
            style.getOrCreate<LineSymbol>()->stroke()->widthUnits() = units;
        }
    }
    else if ( match(c.key(), "stroke-linecap") ) {
        style.getOrCreate<LineSymbol>()->stroke()->lineCap() =
            c.value() == "flat"   ?   Stroke::LINECAP_FLAT   :
            c.value() == "square" ?   Stroke::LINECAP_SQUARE :
            /*value == "round"   ?*/  Stroke::LINECAP_ROUND;
    }
    else if (match(c.key(), "stroke-linejoin") ) {
        style.getOrCreate<LineSymbol>()->stroke()->lineJoin() =
            c.value() == "mitre" ?      Stroke::LINEJOIN_MITRE :
            c.value() == "miter" ?      Stroke::LINEJOIN_MITRE : // alternate spelling
            /*c.value() == "round"  ?*/ Stroke::LINEJOIN_ROUND;
    }
    else if ( match(c.key(), "stroke-rounding-ratio") ) {
        style.getOrCreate<LineSymbol>()->stroke()->roundingRatio() = as<float>(c.value(), 0.4f);
    }
    else if ( match(c.key(), "stroke-tessellation-segments") ) {
        style.getOrCreate<LineSymbol>()->tessellation() = as<unsigned>( c.value(), 0 );
    }
    else if ( match(c.key(), "stroke-tessellation-size") ) {
        float value;
        Units units;
        if ( Units::parse(c.value(), value, units, Units::METERS) ) {
            style.getOrCreate<LineSymbol>()->tessellationSize() = Distance(value, units);
        }
    }        
    else if ( match(c.key(), "stroke-min-pixels") ) {
        style.getOrCreate<LineSymbol>()->stroke()->minPixels() = as<float>(c.value(), 0.0f);
    }
    else if ( match(c.key(), "stroke-stipple-factor") ) {
        style.getOrCreate<LineSymbol>()->stroke()->stippleFactor() = as<unsigned>(c.value(), 1);
    }
    else if ( match(c.key(), "stroke-stipple-pattern") ||
              match(c.key(), "stroke-stipple") ) {
        style.getOrCreate<LineSymbol>()->stroke()->stipplePattern() = as<unsigned short>(c.value(), 0xFFFF);
    }
    else if ( match(c.key(), "stroke-crease-angle") ) {
        style.getOrCreate<LineSymbol>()->creaseAngle() = as<float>(c.value(), 0.0);
    }
    else if ( match(c.key(), "stroke-script") ) {
        style.getOrCreate<LineSymbol>()->script() = StringExpression(c.value());
    }
    else if (match(c.key(), "stroke-image")) {
        style.getOrCreate<LineSymbol>()->imageURI() = StringExpression(stripQuotes(c.value()), c.referrer());
    }
    else if (match(c.key(), "stroke-smooth")) {
        style.getOrCreate<LineSymbol>()->stroke()->smooth() = as<bool>(c.value(), false);
    }
}
