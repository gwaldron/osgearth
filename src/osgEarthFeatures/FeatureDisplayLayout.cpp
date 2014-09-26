/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthFeatures/FeatureDisplayLayout>
#include <limits>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

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
    conf.getIfSet( "min_range", _minRange );
    conf.getIfSet( "max_range", _maxRange );
    conf.getIfSet( "style",     _styleName ); 
    conf.getIfSet( "class",     _styleName ); // alias
}

Config
FeatureLevel::getConfig() const
{
    Config conf( "level" );
    conf.addIfSet( "min_range", _minRange );
    conf.addIfSet( "max_range", _maxRange );
    conf.addIfSet( "style",     _styleName );
    return conf;
}

//------------------------------------------------------------------------

FeatureDisplayLayout::FeatureDisplayLayout( const Config& conf ) :
_tileSizeFactor( 15.0f ),
_minRange      ( 0.0f ),
_maxRange      ( 0.0f ),
_cropFeatures  ( false ),
_priorityOffset( 0.0f ),
_priorityScale ( 1.0f )
{
    fromConfig( conf );
}

void
FeatureDisplayLayout::fromConfig( const Config& conf )
{
    conf.getIfSet( "tile_size_factor", _tileSizeFactor );
    conf.getIfSet( "crop_features",    _cropFeatures );
    conf.getIfSet( "priority_offset",  _priorityOffset );
    conf.getIfSet( "priority_scale",   _priorityScale );
    conf.getIfSet( "min_range",        _minRange );
    conf.getIfSet( "max_range",        _maxRange );
    ConfigSet children = conf.children( "level" );
    for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
        addLevel( FeatureLevel( *i ) );
}

Config
FeatureDisplayLayout::getConfig() const
{
    Config conf( "layout" );
    conf.addIfSet( "tile_size_factor", _tileSizeFactor );
    conf.addIfSet( "crop_features",    _cropFeatures );
    conf.addIfSet( "priority_offset",  _priorityOffset );
    conf.addIfSet( "priority_scale",   _priorityScale );
    conf.addIfSet( "min_range",        _minRange );
    conf.addIfSet( "max_range",        _maxRange );
    for( Levels::const_iterator i = _levels.begin(); i != _levels.end(); ++i )
        conf.add( i->second.getConfig() );
    return conf;
}

void 
FeatureDisplayLayout::addLevel( const FeatureLevel& level )
{
    _levels.insert( std::make_pair( -level.maxRange(), level ) );
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

//float
//FeatureDisplayLayout::getMaxRange() const
//{
//    return _levels.size() > 0 ? _levels.begin()->second.maxRange() : 0.0f;
//}

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
