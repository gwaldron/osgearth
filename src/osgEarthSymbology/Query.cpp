/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthSymbology/Query>

using namespace osgEarth;
using namespace osgEarth::Symbology;

Query::Query( const Config& conf ):
_map(0)
{
    mergeConfig( conf );
}

Query::Query(const Query& rhs) :
_bounds(rhs._bounds),
_expression(rhs._expression),
_orderby(rhs._orderby),
_tileKey(rhs._tileKey),
_map(rhs._map)
{
    //nop
}

void
Query::mergeConfig( const Config& conf )
{
    if ( !conf.getIfSet( "expr", _expression ) )
        if ( !conf.getIfSet( "where", _expression ) )
            if ( !conf.getIfSet( "sql", _expression ) )
                conf.getIfSet( "expression", _expression );

    conf.getIfSet("orderby", _orderby);

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

Config
Query::getConfig() const
{
    Config conf( "query" );
    conf.addIfSet( "expr", _expression );
    conf.addIfSet( "orderby", _orderby);
    if ( _bounds.isSet() ) {
        Config bc( "extent" );
        bc.add( "xmin", toString(_bounds->xMin()) );
        bc.add( "ymin", toString(_bounds->yMin()) );
        bc.add( "xmax", toString(_bounds->xMax()) );
        bc.add( "ymax", toString(_bounds->yMax()) );
        conf.add( bc );
    }
    return conf;
}

Query
Query::combineWith( const Query& rhs ) const
{
    Query merged;

    // merge the expressions:

    bool lhsEmptyExpr = !_expression.isSet() || _expression->empty();
    bool rhsEmptyExpr = !rhs.expression().isSet() || rhs.expression()->empty();

    if ( !lhsEmptyExpr && !rhsEmptyExpr )
    {
        std::stringstream buf;
        buf << "( " << *_expression << " ) AND ( " << *rhs.expression() << " )";
        std::string str;
        str = buf.str();
        merged.expression() = str;
    }
    else if ( lhsEmptyExpr && !rhsEmptyExpr )
    {
        merged.expression() = *rhs.expression();
    }
    else if ( !lhsEmptyExpr && rhsEmptyExpr )
    {
        merged.expression() = *_expression;
    }

    // tilekey overrides bounds:
    if ( _tileKey.isSet() )
    {
        merged.tileKey() = *_tileKey;
    }
    else if ( rhs._tileKey.isSet() )
    {
        merged.tileKey() = *rhs._tileKey;
    }

    // merge the bounds:
    if ( bounds().isSet() && rhs.bounds().isSet() )
    {
        merged.bounds() = bounds()->intersectionWith( *rhs.bounds() );
    }
    else if ( bounds().isSet() )
    {
        merged.bounds() = *bounds();
    }
    else if ( rhs.bounds().isSet() )
    {
        merged.bounds() = *rhs.bounds();
    }

    return merged;
}

const Map* Query::getMap() const
{
    return _map;
}

void Query::setMap(const Map* map)
{
    _map = map;
}
