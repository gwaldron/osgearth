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
#include <osgEarth/MapLayer>

using namespace osgEarth;

MapLayer::MapLayer(const std::string& name, Type type, const std::string& driver, const Properties& driverProps,
                   const CacheConfig& caching, const ProfileConfig& profile) :
_name( name ),
_type( type ),
_driver( driver ),
_driverProps( driverProps ),
_cacheConf( caching ),
_profileConf( profile ),
_minLevel(0),
_maxLevel(25)
{
    //NOP
}

MapLayer::MapLayer(const std::string& name, Type type, TileSource* source,
                   const CacheConfig& caching, const ProfileConfig& profile ) :
_name( name ),
_type( type ),
_tileSource( source ),
_cacheConf( caching ),
_profileConf( profile ),
_minLevel(0),
_maxLevel(25)
{
    //NOP
}

unsigned int
MapLayer::getMinLevel() const {
    return _minLevel;
}

unsigned int
MapLayer::getMaxLevel() const {
    return _maxLevel;
}

void
MapLayer::setMinLevel( unsigned int value ) {
    _minLevel = value; 
}

void MapLayer::setMaxLevel( unsigned int value ) {
    _maxLevel = value;
}

const std::string&
MapLayer::getName() const {
    return _name; 
}

const MapLayer::Type&
MapLayer::getType() const {
    return _type;
}

const std::string& 
MapLayer::getDriver() const {
    return _driver;
}

const Properties& 
MapLayer::getDriverProperties() const {
    return _driverProps;
}

TileSource* 
MapLayer::getTileSource() const {
    return _tileSource.get();
}

const CacheConfig&
MapLayer::getCacheConfig() const {
    return _cacheConf;
}

const ProfileConfig&
MapLayer::getProfileConfig() const {
    return _profileConf;
}

void
MapLayer::setTileSource( TileSource* tileSource )
{
    if ( !_tileSource.valid() )
        _tileSource = tileSource;
    else
        osg::notify(osg::WARN) << "[osgEarth::MapLayer] TileSource already set; ignoring attempt to set it again" << std::endl;
}
