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

#include <osgEarth/Layer>
#include <osgEarth/Map>

using namespace osgEarth;


static int s_nextLayerId = 0;

Layer::Layer():
_enabled(true),
_opacity(1.0f)
{
    _id = s_nextLayerId++;
}

Layer::~Layer()
{
}

int
Layer::getId() const
{
    return _id;
}

const std::string&
Layer::getName() const
{
    return _name;
}

void
Layer::setName(const std::string& name)
{
    _name = name;
}

bool
Layer::getEnabled() const
{
    return _enabled;
}

void
Layer::setEnabled(bool enabled)
{
    if (_enabled != enabled)
    {
        _enabled = enabled;
        if (_map.valid()) _map->updateUniforms();
    }
}

float
Layer::getOpacity() const
{
    return _opacity;
}

void
Layer::setOpacity(float opacity)
{
    if (_opacity != opacity)
    {
        _opacity = osg::clampBetween(opacity, 0.0f, 1.0f);
        if (_map.valid()) _map->updateUniforms();
    }
}

void
Layer::setMap( Map *map)
{
    _map = map;
}



/*****************************************************************************************/
ImageLayer::ImageLayer(TileSource *tileSource):
Layer(),
_tileSource(tileSource)
{
}

TileSource*
ImageLayer::getTileSource() const
{
    return _tileSource.get();
}

/*****************************************************************************************/
ElevationLayer::ElevationLayer(osgEarth::TileSource *tileSource):
_tileSource(tileSource)
{
}

TileSource*
ElevationLayer::getTileSource() const
{
    return _tileSource.get();
}


/*****************************************************************************************/

osgEarthImageLayer::osgEarthImageLayer(int layerId, osg::Image* image):
ImageLayer(image),
_layerId(layerId),
_visible(true),
_opacity(1.0f)
{
}

int 
osgEarthImageLayer::getLayerId() const
{
    return _layerId;
}

bool
osgEarthImageLayer::getVisible() const
{
    return _visible;
}

void osgEarthImageLayer::setVisible(bool visible)
{
    if (_visible != visible)
    {
        _visible = visible;
    }
}

float
osgEarthImageLayer::getOpacity() const
{
    return _opacity;
}

void osgEarthImageLayer::setOpacity(float opacity)
{
    _opacity = opacity;
}


