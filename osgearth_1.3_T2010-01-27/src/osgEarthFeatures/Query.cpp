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
#include <osgEarthFeatures/Query>

using namespace osgEarth::Features;

Query::Query() :
_bounds( Bounds() ),
_expression( "" )
{
    //nop
}

Query::Query( const Config& conf ) :
_bounds( Bounds() ),
_expression( "" )
{
    if ( conf.hasValue( "expr" ) )
        _expression = conf.value( "expr" );
    else if ( conf.hasValue( "where" ) )
        _expression = conf.value( "where" );
    else if ( conf.hasValue( "sql" ) )
        _expression = conf.value( "sql" );
    else if ( conf.hasValue( "expression" ) )
        _expression = conf.value( "expression" );

    Config b = conf.child( "extent" );
    if( !b.empty() )
    {
        _bounds = Bounds(
            b.value<double>( "xmin", 0.0 ),
            b.value<double>( "ymin", 0.0 ),
            b.value<double>( "xmax", 0.0 ),
            b.value<double>( "ymax", 0.0 ) );
    }
}

