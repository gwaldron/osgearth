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

#include <osgEarth/TileKey>
#include <osgEarth/Mercator>

using namespace osgEarth;

TileKey::TileKey() :
_profile( TileGridProfile::UNKNOWN )
{
    //NOP
}

TileKey::TileKey( const TileKey& rhs ) :
_x(rhs._x),
_y(rhs._y),
_lod(rhs._lod),
_profile( rhs._profile )
{
    //NOP
}

std::string
TileKey::str() const
{
    std::stringstream ss;
    ss << _lod << "_" << _x << "_" << _y;
    return ss.str();
}

const TileGridProfile&
TileKey::getProfile() const
{
    return _profile;
}

int
TileKey::getMapSizePixels(const unsigned int &tile_size) const
{
    return getMapSizePixels( tile_size, getLevelOfDetail() );
}

/*static*/ int
TileKey::getMapSizePixels(const unsigned int &tile_size, const unsigned int &lod )
{
    return tile_size << lod;
}

int
TileKey::getMapSizeTiles() const
{
    return getMapSizeTiles(getLevelOfDetail());
}

int
TileKey::getMapSizeTiles(const unsigned int level)
{
    return pow(2.0, (double)level);
}

void
TileKey::getTileXY(unsigned int& out_tile_x,
                   unsigned int& out_tile_y) const
{
    out_tile_x = _x;
    out_tile_y = _y;
}

osgTerrain::TileID
TileKey::getTileId() const
{
    return osgTerrain::TileID(_lod, _x, _y);
}

unsigned int
TileKey::getLevelOfDetail() const
{
    return _lod;
}

void
TileKey::getPixelExtents(unsigned int& xmin,
                         unsigned int& ymin,
                         unsigned int& xmax,
                         unsigned int& ymax,
                         const unsigned int &tile_size) const
{
    xmin = _x * tile_size;
    ymin = _y * tile_size;
    xmax = xmin + tile_size;
    ymax = ymin + tile_size; 
}

TileKey*
TileKey::getSubkey( unsigned int quadrant ) const
{
    if ( !_subkeys[quadrant].valid() )
    {
        unsigned int lod = _lod + 1;
        unsigned int x = _x * 2;
        unsigned int y = _y * 2;

        if (quadrant == 1)
        {
            x+=1;
        }
        else if (quadrant == 2)
        {
            y+=1;
        }
        else if (quadrant == 3)
        {
            x+=1;
            y+=1;
        }
        const_cast<TileKey*>(this)->_subkeys[quadrant] = new TileKey(x, y, lod, _profile);
    }
    return _subkeys[quadrant].get();
}


TileKey*
TileKey::createParentKey() const
{
    if (_lod == 0) return NULL;

    unsigned int lod = _lod - 1;
    unsigned int x = _x / 2;
    unsigned int y = _y / 2;
    return new TileKey(x, y, lod, _profile);
}


bool
TileKey::getGeoExtents(
            double& xmin,
            double& ymin,
            double& xmax,
            double& ymax) const
{
    getNativeExtents(xmin, ymin, xmax, ymax);
    //Convert meters to degrees if if in Mercator.
    if (isMercator())
    {
        Mercator::metersToLatLon(xmin, ymin, ymin, xmin);
        Mercator::metersToLatLon(xmax, ymax, ymax, xmax);
    }
    return true;
}

bool
TileKey::getNativeExtents(
            double& xmin,
            double& ymin,
            double& xmax,
            double& ymax) const
{
    double width, height;
    _profile.getTileDimensions(_lod, width, height);

    xmin = _profile.xMin() + (width * (double)_x);
    ymax = _profile.yMax() - (height * (double)_y);
    xmax = xmin + width;
    ymin = ymax - height;
    return true;
}


TileKey::TileKey( unsigned int tile_x, unsigned int tile_y, unsigned int lod, TileGridProfile profile)
{
    _x = tile_x;
    _y = tile_y;
    _lod = lod;
    _profile = profile;
}

bool TileKey::isGeodetic() const
{
    return _profile.getProfileType() == TileGridProfile::GLOBAL_GEODETIC;
}

bool TileKey::isMercator() const
{
    return _profile.getProfileType() == TileGridProfile::GLOBAL_MERCATOR;
}

bool TileKey::isProjected() const
{
    return _profile.getProfileType() == TileGridProfile::PROJECTED;
}