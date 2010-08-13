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
#include <osgEarthSymbology/CssUtils>
#include <osgEarth/StringUtils>
#include <iostream>
#include <sstream>
#include <iterator>

using namespace osgEarth;
using namespace osgEarth::Symbology;

Config
CssUtils::readConfig( std::istream& in )
{
    // read the entire stream into a string:
    std::stringstream buf;
    std::copy( std::istream_iterator<char>(in),
        std::istream_iterator<char>(),
        std::ostream_iterator<char>( buf ) );
    std::string css;
	css = buf.str();

    // tokenize the CSS into a config object..
    Config conf( "css" );

    StringTokenizer tok( css, "{}" );
    while( tok.nextToken() ) {
        std::string name = tok.token();
        if ( tok.nextToken() ) {
            Config elementConf( name );
            std::string props = tok.token();
            StringTokenizer propsTok( props, ":;" );
            while( propsTok.nextToken() ) {
                std::string key = propsTok.token();
                if ( propsTok.nextToken() ) {
                    std::string value = propsTok.token();
                    elementConf.attr(key) = value;
                }
            }    
            conf.add( elementConf );
        }
    }

    return conf;
}
