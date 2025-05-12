/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Query>

using namespace osgEarth;

Query::Query(const Config& conf)
{
    mergeConfig(conf);
}

Query::Query(const TileKey& key_)
{
    _tileKey = key_;
}

void
Query::mergeConfig( const Config& conf )
{
    if ( !conf.get( "expr", _expression ) )
        if ( !conf.get( "where", _expression ) )
            if ( !conf.get( "sql", _expression ) )
                conf.get( "expression", _expression );

    conf.get("orderby", _orderby);

    Config b = conf.child( "extent" );
    if( !b.empty() )
    {
        _bounds = Bounds(
            b.value<double>( "xmin", 0.0 ),
            b.value<double>( "ymin", 0.0 ),
            0.0,
            b.value<double>( "xmax", 0.0 ),
            b.value<double>( "ymax", 0.0 ),
            0.0);
    }

    conf.get("limit", _limit);
}

Config
Query::getConfig() const
{
    Config conf( "query" );
    conf.set( "expr", _expression );
    conf.set( "orderby", _orderby);
    conf.set( "limit", _limit);
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
        merged.bounds() = intersectionOf(*bounds(), *rhs.bounds());
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
