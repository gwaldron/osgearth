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

MapLayer::MapLayer(const std::string& name, Type type, const std::string& driver, const Properties& driverProps) :
_name( name ),
_type( type ),
_driver( driver ),
_driverProps( driverProps ),
_opacity(1.0f),
_enabled(true)
{
    //NOP
}

MapLayer::MapLayer(const std::string& name, Type type, TileSource* source ) :
_name( name ),
_type( type ),
_tileSource( source ),
_opacity(1.0f),
_enabled(true)
{
    //NOP
}

optional<int>&
MapLayer::minLevel() {
    return _minLevel;
}
const optional<int>&
MapLayer::minLevel() const {
    return _minLevel;
}

optional<int>&
MapLayer::maxLevel() {
    return _maxLevel;
}
const optional<int>&
MapLayer::maxLevel() const {
    return _maxLevel;
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

optional<CacheConfig>&
MapLayer::cacheConfig() {
    return _cacheConf;
}
const optional<CacheConfig>&
MapLayer::cacheConfig() const {
    return _cacheConf;
}

optional<ProfileConfig>&
MapLayer::profileConfig() {
    return _profileConf;
}
const optional<ProfileConfig>&
MapLayer::profileConfig() const {
    return _profileConf;
}

optional<std::string>& 
MapLayer::noDataImage()
{
	return _nodata_image;
}
const optional<std::string>&
MapLayer::noDataImage() const
{
	return _nodata_image;
}

void
MapLayer::setTileSource( TileSource* tileSource )
{
    //if ( !_tileSource.valid() )
    //{
        _tileSource = tileSource;
        if ( _tileSource.valid() )
        {
            if ( minLevel().isSet() )
                _tileSource->setMinLevel( _minLevel.get() );
            if ( _maxLevel.isSet() )
                _tileSource->setMaxLevel( _maxLevel.get() );

			if (_transparentColor.isSet() )
				_tileSource->transparentColor() = _transparentColor.get();

			if ( _nodata_image.isSet() && _nodata_image.get().length() > 0 )
			{
				osg::notify(osg::NOTICE) << "Setting nodata image to " << _nodata_image.get() << std::endl;
				osg::Image* image = osgDB::readImageFile( _nodata_image.get());
				if (image)
				{
					_tileSource->setNoDataImage( image );
				}
				else
				{
					osg::notify(osg::NOTICE) << "Warning:  Could not read nodata image from " << _nodata_image.get() << std::endl;
				}
			}
        }
    //}
    //else
    //{
    //    osg::notify(osg::WARN) << "[osgEarth::MapLayer] TileSource already set; ignoring attempt to set it again" << std::endl;
    //}
}

float
MapLayer::getOpacity() const
{
	return _opacity;
}

void
MapLayer::setOpacity(float opacity)
{
	_opacity = osg::clampBetween(opacity, 0.0f, 1.0f);
}

bool
MapLayer::getEnabled() const
{
	return _enabled;
}

void
MapLayer::setEnabled(bool enabled)
{
	_enabled = enabled;
}

const optional<osg::Vec4ub>& 
MapLayer::transparentColor() const
{
	return _transparentColor;
}

optional<osg::Vec4ub>&
MapLayer::transparentColor()
{
	return _transparentColor;
}

