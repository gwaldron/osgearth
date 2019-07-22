/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#include <limits.h> 

using namespace osgEarth;

//------------------------------------------------------------------------

//statics
CachePolicy CachePolicy::DEFAULT;
CachePolicy CachePolicy::NO_CACHE( CachePolicy::USAGE_NO_CACHE );
CachePolicy CachePolicy::CACHE_ONLY( CachePolicy::USAGE_CACHE_ONLY );

//------------------------------------------------------------------------

CachePolicy::CachePolicy() :
_usage  ( USAGE_READ_WRITE ),
_maxAge ( INT_MAX ),
_minTime( 0 )
{
    //nop
}

CachePolicy::CachePolicy( const Usage& usage ) :
_usage  ( usage ),
_maxAge ( INT_MAX ),
_minTime( 0 )
{
    _usage = usage; // explicity set the optional<>
}

CachePolicy::CachePolicy( const Config& conf ) :
_usage  ( USAGE_READ_WRITE ),
_maxAge ( INT_MAX ),
_minTime( 0 )
{
    fromConfig( conf );
}

CachePolicy::CachePolicy(const CachePolicy& rhs) :
_usage  ( rhs._usage ),
_maxAge ( rhs._maxAge ),
_minTime( rhs._minTime )
{
    //nop
}

void
CachePolicy::mergeAndOverride(const CachePolicy& rhs)
{
    if ( rhs.usage().isSet() )
        usage() = rhs.usage().get();

    if ( rhs.minTime().isSet() )
        minTime() = rhs.minTime().get();

    if ( rhs.maxAge().isSet() )
        maxAge() = rhs.maxAge().get();
}

void
CachePolicy::mergeAndOverride(const optional<CachePolicy>& rhs)
{
    if ( rhs.isSet() )
    {
        mergeAndOverride( rhs.get() );
    }
}

TimeStamp
CachePolicy::getMinAcceptTime() const
{
    return
        _minTime.isSet() ? _minTime.value() :
        _maxAge.isSet()  ? DateTime().asTimeStamp() - _maxAge.value() :
        0;
}

bool
CachePolicy::isExpired(TimeStamp lastModified) const
{
    return lastModified < getMinAcceptTime();    
}

bool
CachePolicy::operator == (const CachePolicy& rhs) const
{
    return 
        (_usage.get() == rhs._usage.get()) &&
        (_maxAge.get() == rhs._maxAge.get()) &&
        (_minTime.get() == rhs._minTime.get());
}

CachePolicy&
CachePolicy::operator = ( const CachePolicy& rhs )
{
    _usage  = optional<Usage>(rhs._usage);
    _maxAge = optional<TimeSpan>(rhs._maxAge);
    _minTime = optional<TimeStamp>(rhs._minTime);

    return *this;
}

std::string
CachePolicy::usageString() const
{
    if ( _usage == USAGE_READ_WRITE ) return "read-write";
    if ( _usage == USAGE_READ_ONLY )  return "read-only";
    if ( _usage == USAGE_CACHE_ONLY)  return "cache-only";
    if ( _usage == USAGE_NO_CACHE)    return "no-cache";
    return "unknown";
}

bool
CachePolicy::empty() const
{
    bool isSet = _usage.isSet() || _maxAge.isSet() || _minTime.isSet();
    return !isSet;
}

void
CachePolicy::fromConfig( const Config& conf )
{
    conf.get( "usage", "read_write",   _usage, USAGE_READ_WRITE );
    conf.get( "usage", "read_only",    _usage, USAGE_READ_ONLY );
    conf.get( "usage", "cache_only",   _usage, USAGE_CACHE_ONLY );
    conf.get( "usage", "no_cache",     _usage, USAGE_NO_CACHE );
    conf.get( "usage", "none",         _usage, USAGE_NO_CACHE );
    conf.get( "max_age", _maxAge );
    conf.get( "min_time", _minTime );
}

Config
CachePolicy::getConfig() const
{
    Config conf( "cache_policy" );
    conf.set( "usage", "read_write",   _usage, USAGE_READ_WRITE );
    conf.set( "usage", "read_only",    _usage, USAGE_READ_ONLY );
    conf.set( "usage", "cache_only",   _usage, USAGE_CACHE_ONLY );
    conf.set( "usage", "no_cache",     _usage, USAGE_NO_CACHE );
    conf.set( "max_age", _maxAge );
    conf.set( "min_time", _minTime );
    return conf;
}
