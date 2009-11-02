/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/CssUtils>
#include <iostream>
#include <sstream>
#include <iterator>

using namespace osgEarth;
using namespace osgEarthFeatures;
using namespace osgEarthFeatures::Styling;

static std::string
trim( const std::string& in )
{
    std::string whitespace (" \t\f\v\n\r");
    // by Rodrigo C F Dias
    // http://www.codeproject.com/KB/stl/stdstringtrim.aspx
    std::string str = in;
    std::string::size_type pos = str.find_last_not_of( whitespace );
    if(pos != std::string::npos) {
        str.erase(pos + 1);
        pos = str.find_first_not_of( whitespace );
        if(pos != std::string::npos) str.erase(0, pos);
    }
    else str.erase(str.begin(), str.end());
    return str;
}

// http://stackoverflow.com/questions/53849/how-do-i-tokenize-a-string-in-c
class Tokenizer 
{
public:
    Tokenizer(const std::string& s)
        : _string(s), _offset(0), _delimiters(" \t\n\r") { }
    Tokenizer(const std::string& s, const std::string& delimiters)
        : _string(s), _offset(0), _delimiters(delimiters) { }
    bool nextToken() {
        return nextToken( _delimiters ); }
    bool nextToken(const std::string& delimiters) {
        size_t i = _string.find_first_not_of(delimiters, _offset);
        if (std::string::npos == i) {
            _offset = _string.length();
            return false;
        }
        size_t j = _string.find_first_of(delimiters, i);
        if (std::string::npos == j) {
            _token = _string.substr(i);
            _offset = _string.length();
            return true;
        }
        _token = _string.substr(i, j - i);
        _offset = j;
        return true;
    }
    const std::string& token() const { 
        return trim( _token );
    }
protected:
    size_t _offset;
    const std::string _string;
    std::string _token;
    std::string _delimiters;
};


Config
CssUtils::readConfig( std::istream& in )
{
    // read the entire stream into a string:
    std::stringstream buf;
    std::copy( std::istream_iterator<char>(in),
        std::istream_iterator<char>(),
        std::ostream_iterator<char>( buf ) );
    std::string css = buf.str();

    // tokenize the CSS into a config object..
    Config conf( "css" );

    Tokenizer tok( css, "{}" );
    while( tok.nextToken() ) {
        std::string name = tok.token();
        if ( tok.nextToken() ) {
            Config elementConf( name );
            std::string props = tok.token();
            Tokenizer propsTok( props, ":;" );
            while( tok.nextToken() ) {
                std::string key = tok.token();
                if ( tok.nextToken() ) {
                    std::string value = tok.token();
                    elementConf.attr(key) = value;
                }
            }    
            conf.add( elementConf );
        }
    }

    return conf;
}
