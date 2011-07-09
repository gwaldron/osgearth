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

#include <osgEarth/StringUtils>

using namespace osgEarth;

StringTokenizer::StringTokenizer( const std::string& delims, const std::string& quotes ) :
_allowEmpties( true ),
_trimTokens  ( true )
{
    addDelims( delims );
    addQuotes( quotes );
}

StringTokenizer::StringTokenizer(const std::string& input, 
                                 StringVector&      output,
                                 const std::string& delims, 
                                 const std::string& quotes ) :
_allowEmpties( true ),
_trimTokens  ( true )
{
    addDelims( delims );
    addQuotes( quotes );
    tokenize( input, output );
}

void
StringTokenizer::addDelim( char delim, bool keep )
{
    _delims[delim] = keep;
}

void
StringTokenizer::addDelims( const std::string& delims, bool keep )
{
    for( unsigned i=0; i<delims.size(); ++i )
        addDelim( delims.at(i), keep );
}

void
StringTokenizer::addQuote( char quote, bool keep )
{
    _quotes[quote] = keep;
}

void
StringTokenizer::addQuotes( const std::string& quotes, bool keep )
{
    for( unsigned i=0; i<quotes.size(); ++i )
        addQuote( quotes.at(i), keep );
}

void
StringTokenizer::tokenize( const std::string& input, StringVector& output ) const
{
    output.clear();

    std::stringstream buf;
    bool quoted = false;
    for( std::string::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        char c = *i;    

        TokenMap::const_iterator q = _quotes.find( c );

        if ( quoted )
        {
            if ( q != _quotes.end() )
            {
                quoted = false;
                if ( q->second )
                    buf << c;
            }
            else
            {
                buf << c;
            }
        }
        else
        {
            if ( q != _quotes.end() )
            {
                quoted = true;
                if ( q->second )
                    buf << c;
            }
            else
            {
                TokenMap::const_iterator d = _delims.find( c );
                if ( d == _delims.end() )
                {
                    buf << c;
                }
                else
                {
                    std::string token = _trimTokens ? trim(buf.str()) : buf.str();

                    if ( _allowEmpties || !token.empty() )
                        output.push_back( token );

                    if ( d->second == true )
                    {
                        output.push_back( std::string(1, c) );
                    }

                    buf.str("");
                }
            }
        }       
    }

    std::string last = _trimTokens ? trim(buf.str()) : buf.str();
    if ( !last.empty() )
        output.push_back( last );
}

