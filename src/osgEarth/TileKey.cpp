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

#include <osgEarth/TileKey>
#include <osgEarth/StringUtils>

using namespace osgEarth;

//------------------------------------------------------------------------

TileKey TileKey::INVALID( 0, 0, 0, 0L );

//------------------------------------------------------------------------

TileKey::TileKey( unsigned int lod, unsigned int tile_x, unsigned int tile_y, const Profile* profile)
{
    _x = tile_x;
    _y = tile_y;
    _lod = lod;
    _profile = profile;

    double width, height;
    if ( _profile.valid() )
    {
        _profile->getTileDimensions(lod, width, height);

        double xmin = _profile->getExtent().xMin() + (width * (double)_x);
        double ymax = _profile->getExtent().yMax() - (height * (double)_y);
        double xmax = xmin + width;
        double ymin = ymax - height;

        _extent = GeoExtent( _profile->getSRS(), xmin, ymin, xmax, ymax );

        _key = Stringify() << _lod << "/" << _x << "/" << _y;
    }
    else
    {
        _extent = GeoExtent::INVALID;
        _key = "invalid";
    }
}

TileKey::TileKey( const TileKey& rhs ) :
_key( rhs._key ),
_lod(rhs._lod),
_x(rhs._x),
_y(rhs._y),
_profile( rhs._profile.get() ),
_extent( rhs._extent )
{
    //NOP
}

const Profile*
TileKey::getProfile() const
{
    return _profile.get();
}

void
TileKey::getTileXY(unsigned int& out_tile_x,
                   unsigned int& out_tile_y) const
{
    out_tile_x = _x;
    out_tile_y = _y;
}

unsigned
TileKey::getQuadrant() const
{
    if ( _lod == 0 )
        return 0;
    bool xeven = (_x & 1) == 0;
    bool yeven = (_y & 1) == 0;
    return 
        xeven && yeven ? 0 :
        xeven          ? 2 :
        yeven          ? 1 : 3;
}

osgTerrain::TileID
TileKey::getTileId() const
{
    //TODO: will this be an issue with multi-face? perhaps not since each face will
    // exist within its own scene graph.. ?
    return osgTerrain::TileID(_lod, _x, _y);
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

TileKey
TileKey::createChildKey( unsigned int quadrant ) const
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
    return TileKey( lod, x, y, _profile.get());
}


TileKey
TileKey::createParentKey() const
{
    if (_lod == 0) return TileKey::INVALID;

    unsigned int lod = _lod - 1;
    unsigned int x = _x / 2;
    unsigned int y = _y / 2;
    return TileKey( lod, x, y, _profile.get());
}

TileKey
TileKey::createAncestorKey( int ancestorLod ) const
{
    if ( ancestorLod > (int)_lod ) return TileKey::INVALID;

    unsigned int x = _x, y = _y;
    for( int i=_lod; i > ancestorLod; i-- )
    {
        x /= 2;
        y /= 2;
    }
    return TileKey( ancestorLod, x, y, _profile.get() );
}

TileKey
TileKey::createNeighborKey( int xoffset, int yoffset ) const
{
    unsigned tx, ty;
    getProfile()->getNumTiles( _lod, tx, ty );

    int sx = (int)_x + xoffset;
    unsigned x =
        sx < 0        ? (unsigned)((int)tx + sx) :
        sx >= (int)tx ? (unsigned)sx - tx :
        (unsigned)sx;

    int sy = (int)_y + yoffset;
    unsigned y =
        sy < 0        ? (unsigned)((int)ty + sy) :
        sy >= (int)ty ? (unsigned)sy - ty :
        (unsigned)sy;

    //OE_NOTICE << "Returning neighbor " << x << ", " << y << " for tile " << str() << " offset=" << xoffset << ", " << yoffset << std::endl;

    return TileKey( _lod, x, y, _profile.get() );
}
