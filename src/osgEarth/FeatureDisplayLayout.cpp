/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/FeatureDisplayLayout>
#include <limits>

using namespace osgEarth;

//------------------------------------------------------------------------

FeatureLevel::FeatureLevel( const Config& conf ) :
_minRange( 0.0f ),
_maxRange( FLT_MAX )
{
    fromConfig( conf );
}

FeatureLevel::FeatureLevel( float minRange, float maxRange )
{
    _minRange = minRange;
    _maxRange = maxRange;
}

FeatureLevel::FeatureLevel( float minRange, float maxRange, const std::string& styleName )
{
    _minRange = minRange;
    _maxRange = maxRange;
    _styleName = styleName;
}

void
FeatureLevel::fromConfig( const Config& conf )
{
    conf.get( "min_range", _minRange );
    conf.get( "max_range", _maxRange );
    conf.get( "style",     _styleName ); 
    conf.get( "class",     _styleName ); // alias
    conf.get( "style_expr", _styleExpr );
    conf.get( "class_expr", _styleExpr ); // alias
}

Config
FeatureLevel::getConfig() const
{
    Config conf( "level" );
    conf.set( "min_range", _minRange );
    conf.set( "max_range", _maxRange );
    conf.set( "style",     _styleName );
    conf.set( "style_expr", _styleExpr);
    return conf;
}

//------------------------------------------------------------------------

FeatureDisplayLayout::FeatureDisplayLayout( const Config& conf ) :
_tileSizeFactor( 3.5f ),
_minRange      ( 0.0f ),
_maxRange      ( 0.0f ),
_cropFeatures  ( false ),
_priorityOffset( 0.0f ),
_priorityScale ( 1.0f ),
_minExpiryTime ( 0.0f ),
_paged(true)
{
    fromConfig( conf );
}

void
FeatureDisplayLayout::fromConfig( const Config& conf )
{
    conf.get( "tile_size",        _tileSize );
    conf.get( "tile_size_factor", _tileSizeFactor );
    conf.get( "crop_features",    _cropFeatures );
    conf.get( "priority_offset",  _priorityOffset );
    conf.get( "priority_scale",   _priorityScale );
    conf.get( "min_expiry_time",  _minExpiryTime );
    conf.get( "min_range",        _minRange );
    conf.get( "max_range",        _maxRange );
    conf.get("paged", _paged);
    ConfigSet children = conf.children( "level" );
    for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
        addLevel( FeatureLevel( *i ) );
}

Config
FeatureDisplayLayout::getConfig() const
{
    Config conf( "layout" );
    conf.set( "tile_size",        _tileSize );
    conf.set( "tile_size_factor", _tileSizeFactor );
    conf.set( "crop_features",    _cropFeatures );
    conf.set( "priority_offset",  _priorityOffset );
    conf.set( "priority_scale",   _priorityScale );
    conf.set( "min_expiry_time",  _minExpiryTime );
    conf.set( "min_range",        _minRange );
    conf.set( "max_range",        _maxRange );
    conf.set("paged", _paged);
    for( Levels::const_iterator i = _levels.begin(); i != _levels.end(); ++i )
        conf.add( i->second.getConfig() );
    return conf;
}

void 
FeatureDisplayLayout::addLevel( const FeatureLevel& level )
{
    _levels.insert( std::make_pair( -level.maxRange().get(), level ) );
}

unsigned 
FeatureDisplayLayout::getNumLevels() const
{
    return _levels.size();
}

const FeatureLevel*
FeatureDisplayLayout::getLevel( unsigned n ) const
{
    unsigned i = 0;
    for( Levels::const_iterator k = _levels.begin(); k != _levels.end(); ++k )
    {
        if ( n == i++ )
            return &(k->second);
    }
    return 0L;
}

unsigned
FeatureDisplayLayout::chooseLOD( const FeatureLevel& level, double fullExtentRadius ) const
{
    double radius = fullExtentRadius;    
    unsigned lod = 1;
    for( ; lod < 20; ++lod )
    {
        radius *= 0.5;
        float lodMaxRange = radius * _tileSizeFactor.value();
        
        if ( level.maxRange() >= lodMaxRange )
            break;
    }
    return lod-1;
}
