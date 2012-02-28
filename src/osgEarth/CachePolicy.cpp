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
#include <osgEarth/CachePolicy>

using namespace osgEarth;

//------------------------------------------------------------------------

//statics
CachePolicy CachePolicy::INHERIT;
CachePolicy CachePolicy::NO_CACHE( CachePolicy::USAGE_NO_CACHE );
CachePolicy CachePolicy::CACHE_ONLY( CachePolicy::USAGE_CACHE_ONLY );

//------------------------------------------------------------------------

CachePolicy::CachePolicy( const Usage& usage, double maxAge ) :
_usage( usage ),
_maxAge( maxAge )
{
    _usage  = usage;
    _maxAge = maxAge;
}

CachePolicy::CachePolicy( const Config& conf ) :
_usage ( USAGE_DEFAULT ),
_maxAge( DBL_MAX )
{
    fromConfig( conf );
}

void
CachePolicy::fromConfig( const Config& conf )
{
    conf.getIfSet( "usage", "read_write", _usage, USAGE_READ_WRITE );
    conf.getIfSet( "usage", "read_only",  _usage, USAGE_READ_ONLY );
    conf.getIfSet( "usage", "cache_only", _usage, USAGE_CACHE_ONLY );
    conf.getIfSet( "usage", "no_cache",   _usage, USAGE_NO_CACHE );
    conf.getIfSet( "max_age", _maxAge );
}

Config
CachePolicy::getConfig() const
{
    Config conf( "cache_policy" );
    conf.addIfSet( "usage", "read_write", _usage, USAGE_READ_WRITE );
    conf.addIfSet( "usage", "read_only",  _usage, USAGE_READ_ONLY );
    conf.addIfSet( "usage", "cache_only", _usage, USAGE_CACHE_ONLY );
    conf.addIfSet( "usage", "no_cache",   _usage, USAGE_NO_CACHE );
    conf.addIfSet( "max_age", _maxAge );
    return conf;
}
