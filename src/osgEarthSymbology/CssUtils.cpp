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
    //std::copy( std::istreambuf_iterator<char>(in), //::istream_iterator<char>(in),
    //    std::istreambuf_iterator<char>(),
    //    std::ostreambuf_iterator<char>( buf ) );

    buf << in.rdbuf();

    std::string css;
	css = buf.str();

    // tokenize the CSS into a config object..
    Config conf( "css" );

    StringVector tok;
    osgEarth::tokenize( css, tok, "{}", "" );

    for( unsigned i=0; i<tok.size(); )
    {
        const std::string& name = tok[i++];
        if ( i < tok.size() )
        {
            Config elementConf( name );
            const std::string& props = tok[i++];

            StringVector propsTok;
            osgEarth::tokenize( props, propsTok, ":;" );
            
            for( unsigned j=0; j<propsTok.size(); )
            {
                const std::string& key = propsTok[j++];
                if ( j < propsTok.size() )
                {
                    const std::string& value = propsTok[j++];
                    elementConf.attr(key) = value;
                }
            }
            conf.add( elementConf );
        }
    }

    return conf;
}
