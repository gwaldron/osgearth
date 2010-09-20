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
#include <osgEarth/MapOptions>

using namespace osgEarth;

void
MapOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "name", _name );
    conf.getObjIfSet( "profile", _profileOptions );
    conf.getObjIfSet( "cache", _cacheOptions );

    // all variations:
    conf.getIfSet( "type", "geocentric", _cstype, CSTYPE_GEOCENTRIC );
    conf.getIfSet( "type", "globe",      _cstype, CSTYPE_GEOCENTRIC );
    conf.getIfSet( "type", "round",      _cstype, CSTYPE_GEOCENTRIC );
    conf.getIfSet( "type", "projected",  _cstype, CSTYPE_PROJECTED );
    conf.getIfSet( "type", "flat",       _cstype, CSTYPE_PROJECTED );
    conf.getIfSet( "type", "cube",       _cstype, CSTYPE_GEOCENTRIC_CUBE );
}

Config
MapOptions::getConfig() const
{
    Config conf; // get a fresh one since this is a final object // = ConfigOptions::getConfig();

    conf.updateIfSet( "name", _name );
    conf.updateObjIfSet( "profile", _profileOptions );
    conf.updateObjIfSet( "cache", _cacheOptions );

    // all variations:
    conf.updateIfSet( "type", "geocentric", _cstype, CSTYPE_GEOCENTRIC );
    conf.updateIfSet( "type", "projected",  _cstype, CSTYPE_PROJECTED );
    conf.updateIfSet( "type", "cube",       _cstype, CSTYPE_GEOCENTRIC_CUBE );

    return conf;
}
