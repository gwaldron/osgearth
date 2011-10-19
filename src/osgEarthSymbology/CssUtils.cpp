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

    StringTokenizer blockIzer( "{}", "" );
    blockIzer.addQuotes( "'\"", true );

    StringTokenizer propSetIzer( ";", "" );
    propSetIzer.addQuotes( "'\"", true );

    StringTokenizer propIzer( ":", "'\"" );
    propIzer.addQuotes( "()", true );

    StringVector blocks;
    blockIzer.tokenize( css, blocks );

    for( unsigned i=0; i<blocks.size(); )
    {
        const std::string& name = blocks[i++];
        if ( i < blocks.size() )
        {
            Config elementConf( name );

            StringVector propSet;
            propSetIzer.tokenize( blocks[i++], propSet );
            
            for( unsigned j=0; j<propSet.size(); ++j )
            {
                StringVector prop;
                propIzer.tokenize( propSet[j], prop );

                if ( prop.size() == 2 )
                {
                    elementConf.attr( prop[0] ) = prop[1];
                }
            }
            conf.add( elementConf );
        }
    }

    return conf;
}
