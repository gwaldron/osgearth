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
#include <osgEarthSymbology/BarSymbol>
#include <osgEarthSymbology/Style>
#include <cctype>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(bar, BarSymbol)

BarSymbol::Value::Value()
: _value(0),
_minimumValue(0),
_maximumValue(100),
_valueScale(1.0),
_minimumColor(Fill(1, 0, 0, 1)),
_maximumColor(Fill(1, 0, 0, 1))
{


}

BarSymbol::BarSymbol(const BarSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_width(rhs._width),
_values(rhs._values)
{
}

BarSymbol::BarSymbol( const Config& conf ) :
    Symbol( conf ),
    _width(4.0)
{
    if ( !conf.empty() )
        mergeConfig(conf);
}

Config 
BarSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "bar";
    conf.set("width", _width);
    unsigned index = 0;
    for(ValueList::const_iterator it = _values.begin(); it != _values.end(); ++it, ++index)
    {
        const Value& value = *it;
        Config subconf;
        subconf.set("value", value._value);
        subconf.set("min_value", value._minimumValue);
        subconf.set("max_value", value._maximumValue);
        subconf.set("value_scale", value._valueScale);
        subconf.set("min_color", value._minimumColor);
        subconf.set("max_color", value._maximumColor);
        conf.add(toString(index), subconf);
    }
    
    return conf;
}

void 
BarSymbol::mergeConfig( const Config& conf )
{
    conf.get("width", _width);

    unsigned index = 0;
    bool ok = true;
    do 
    {
        Config subconf = conf.child(toString(index));
        ok = !subconf.empty();
        if(ok)
        {
            Value value;
            Config subconf;
            subconf.get("value", value._value);
            subconf.get("min_value", value._minimumValue);
            subconf.get("max_value", value._maximumValue);
            subconf.get("value_scale", value._valueScale);
            subconf.get("min_color", value._minimumColor);
            subconf.get("max_color", value._maximumColor);
            _values.push_back(value);
        }
    } while (ok);
}

void
BarSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "bar-width")) {
        style.getOrCreate<BarSymbol>()->width() = NumericExpression(c.value());
    }
    else {
        std::string lkey = toLower(c.key());
        if(lkey.compare(0, 4, "bar-") == 0)
        {
            // currently only support up to ten values
            if(lkey.size() >= 6 && std::isdigit(lkey.at(4)) && lkey.at(5) == '-')
            {
                int value_index = lkey.at(4) - '0';
                std::string subkey = lkey.substr(6);
                ValueList& values = style.getOrCreate<BarSymbol>()->_values;
                values.resize(value_index + 1);
                Value& value = values[value_index];
                if(subkey == "value")
                    value.value() = NumericExpression(c.value());
                else if(subkey == "min-value")
                    value.minimumValue() = NumericExpression(c.value());
                else if (subkey == "max-value")
                    value.maximumValue() = NumericExpression(c.value());
                else if (subkey == "value-scale")
                    value.valueScale() = NumericExpression(c.value());
                else if (subkey == "min-color")
                    value.minimumColor() = Color(c.value());
                else if (subkey == "max-color")
                    value.maximumColor() = Color(c.value());
            }
        }
    }
}
