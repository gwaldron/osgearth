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

#include <osgEarth/MapEngineProperties>

using namespace osgEarth;

MapEngineProperties::MapEngineProperties()
{
    _vertical_scale = 1.0f;
    _skirt_ratio = 0.02f;
    _sample_ratio = 1.0f;
    _proxy_port = 8080;
    _min_tile_range_factor = 5;
    _cache_only = false;
    _normalize_edges = false;
    _combine_layers = true;
    _filename = "";
    _use_mercator_locator = true;
}

MapEngineProperties::MapEngineProperties( const MapEngineProperties& rhs )
{
    (*this) = rhs;
}

MapEngineProperties&
MapEngineProperties::operator = ( const MapEngineProperties& rhs )
{
    _vertical_scale = rhs._vertical_scale;
    _skirt_ratio = rhs._skirt_ratio;
    _sample_ratio = rhs._sample_ratio;
    _proxy_host = rhs._proxy_host;
    _proxy_port = rhs._proxy_port;
    _min_tile_range_factor = rhs._min_tile_range_factor;
    _cache_only = rhs._cache_only;
    _combine_layers = rhs._combine_layers;
    _normalize_edges = rhs._normalize_edges;
    _filename = rhs._filename;
    _use_mercator_locator = rhs._use_mercator_locator;
    return *this;
}

void
MapEngineProperties::setFilename(const std::string& filename)
{
    _filename = filename;
}

const std::string&
MapEngineProperties::getFilename() const
{
    return _filename;
}

void
MapEngineProperties::setVerticalScale( float value )
{
    _vertical_scale = value;
}

float
MapEngineProperties::getVerticalScale() const
{
    return _vertical_scale;
}

void
MapEngineProperties::setSampleRatio(float sample_ratio)
{
    _sample_ratio = sample_ratio;
}

float
MapEngineProperties::getSampleRatio() const
{
    return _sample_ratio;
}

void
MapEngineProperties::setSkirtRatio( float value )
{
    _skirt_ratio = value;
}

float
MapEngineProperties::getSkirtRatio() const
{
    return _skirt_ratio;
}

void
MapEngineProperties::setProxyHost( const std::string& value )
{
    _proxy_host = value;
}

const std::string&
MapEngineProperties::getProxyHost() const 
{
    return _proxy_host;
}

void
MapEngineProperties::setProxyPort( unsigned short value )
{
    _proxy_port = value;
}

unsigned short
MapEngineProperties::getProxyPort() const
{
    return _proxy_port;
}

void
MapEngineProperties::setMinTileRangeFactor( float value )
{
    _min_tile_range_factor = value;
}

float
MapEngineProperties::getMinTileRangeFactor() const
{
    return _min_tile_range_factor;
}

void
MapEngineProperties::setCacheOnly(bool cacheOnly)
{
    _cache_only = cacheOnly;
}

bool
MapEngineProperties::getCacheOnly() const
{
    return _cache_only;
}

void
MapEngineProperties::setCombineLayers(bool combineLayers)
{
    _combine_layers = combineLayers;
}

bool
MapEngineProperties::getCombineLayers() const
{
    return _combine_layers;
}

bool
MapEngineProperties::getNormalizeEdges() const
{
    return _normalize_edges;
}

void
MapEngineProperties::setNormalizeEdges(bool normalize_edges)
{
    _normalize_edges = normalize_edges;
}

void
MapEngineProperties::setUseMercatorLocator(bool value)
{
    _use_mercator_locator = value;
}

bool
MapEngineProperties::getUseMercatorLocator() const
{
    return _use_mercator_locator;
}
