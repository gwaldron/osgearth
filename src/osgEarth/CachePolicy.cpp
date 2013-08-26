/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
CachePolicy CachePolicy::DEFAULT( CachePolicy::USAGE_READ_WRITE );
CachePolicy CachePolicy::NO_CACHE( CachePolicy::USAGE_NO_CACHE );
CachePolicy CachePolicy::CACHE_ONLY( CachePolicy::USAGE_CACHE_ONLY );

//------------------------------------------------------------------------

CachePolicy::CachePolicy() :
_expire ( EXPIRE_NEVER ),
_maxAge ( INT_MAX ),
_minTime( 0 )
{
    _usage = USAGE_READ_WRITE;
}

CachePolicy::CachePolicy( const Usage& usage ) :
_usage  ( usage ),
_expire ( EXPIRE_NEVER ),
_maxAge ( INT_MAX ),
_minTime( 0 )
{
    _usage = usage; // explicity init the optional<>
}

CachePolicy::CachePolicy( const Config& conf ) :
_usage  ( USAGE_READ_WRITE ),
_expire ( EXPIRE_NEVER ),
_maxAge ( INT_MAX ),
_minTime( 0 )
{
    fromConfig( conf );
}

CachePolicy::CachePolicy(const CachePolicy& rhs) :
_usage  ( rhs._usage ),
_expire ( rhs._expire ),
_maxAge ( rhs._maxAge ),
_minTime( rhs._minTime )
{
    //nop
}

bool
CachePolicy::fromOptions( const osgDB::Options* dbOptions, optional<CachePolicy>& out )
{
    if ( dbOptions )
    {
        std::string jsonString = dbOptions->getPluginStringData( "osgEarth::CachePolicy" );
        if ( !jsonString.empty() )
        {
            Config conf;
            conf.fromJSON( jsonString );
            out = CachePolicy( conf );
            return true;
        }
    }
    return false;
}

void
CachePolicy::apply( osgDB::Options* dbOptions )
{
    if ( dbOptions )
    {
        Config conf = getConfig();
        dbOptions->setPluginStringData( "osgEarth::CachePolicy", conf.toJSON() );
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
CachePolicy::operator == (const CachePolicy& rhs) const
{
    return 
        (_usage.get() == rhs._usage.get()) &&
        (_expire.get() == rhs._expire.get()) &&
        (_maxAge.get() == rhs._maxAge.get()) &&
        (_minTime.get() == rhs._minTime.get());
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

std::string
CachePolicy::expireString() const
{
    if ( _expire == EXPIRE_NEVER )       return "never";
    if ( _expire == EXPIRE_PER_SESSION ) return "per-session";
    //if ( _expire == EXPIRE_PER_ENTRY )   return "per-entry";
    return "unknown";
}

void
CachePolicy::fromConfig( const Config& conf )
{
    conf.getIfSet( "usage", "read_write",   _usage, USAGE_READ_WRITE );
    conf.getIfSet( "usage", "read_only",    _usage, USAGE_READ_ONLY );
    conf.getIfSet( "usage", "cache_only",   _usage, USAGE_CACHE_ONLY );
    conf.getIfSet( "usage", "no_cache",     _usage, USAGE_NO_CACHE );
    conf.getIfSet( "usage", "none",         _usage, USAGE_NO_CACHE );
    //conf.getIfSet( "expire", "never",       _expire, EXPIRE_NEVER );
    //conf.getIfSet( "expire", "per_session", _expire, EXPIRE_PER_SESSION );
    //conf.getIfSet( "expire", "per_entry",   _expire, EXPIRE_PER_ENTRY );
    conf.getIfSet( "max_age", _maxAge );
    conf.getIfSet( "min_time", _minTime );
}

Config
CachePolicy::getConfig() const
{
    Config conf( "cache_policy" );
    conf.addIfSet( "usage", "read_write",   _usage, USAGE_READ_WRITE );
    conf.addIfSet( "usage", "read_only",    _usage, USAGE_READ_ONLY );
    conf.addIfSet( "usage", "cache_only",   _usage, USAGE_CACHE_ONLY );
    conf.addIfSet( "usage", "no_cache",     _usage, USAGE_NO_CACHE );
    //conf.addIfSet( "expire", "never",       _expire, EXPIRE_NEVER );
    //conf.addIfSet( "expire", "per_session", _expire, EXPIRE_PER_SESSION );
    //conf.addIfSet( "expire", "per_entry",   _expire, EXPIRE_PER_ENTRY );
    conf.addIfSet( "max_age", _maxAge );
    conf.addIfSet( "min_time", _minTime );
    return conf;
}
