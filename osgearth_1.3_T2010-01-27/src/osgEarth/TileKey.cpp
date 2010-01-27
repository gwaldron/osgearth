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

using namespace osgEarth;

TileKey::TileKey() :
osg::Referenced( true )
{
    //NOP
}

TileKey::TileKey( const TileKey& rhs ) :
osg::Referenced( rhs ),
_face(rhs._face),
_lod(rhs._lod),
_x(rhs._x),
_y(rhs._y),
_profile( rhs._profile.get() ),
_extent( rhs._extent)
{
    //NOP
}

std::string
TileKey::str() const
{
    if ( _key.empty() )
    {
        std::stringstream ss;
        ss << _face << "_" << _lod << "_" << _x << "_" << _y;
		std::string ssStr;
		ssStr = ss.str();
        const_cast<TileKey*>(this)->_key = ssStr;
    }
    return _key;
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

osgTerrain::TileID
TileKey::getTileId() const
{
    //TODO: will this be an issue with multi-face? perhaps not since each face will
    // exist within its own scene graph.. ?
    return osgTerrain::TileID(_lod, _x, _y);
}

unsigned int
TileKey::getFace() const
{
    return _face;
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
TileKey::createSubkey( unsigned int quadrant ) const
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
    return new TileKey( _face, lod, x, y, _profile.get());
}


TileKey*
TileKey::createParentKey() const
{
    if (_lod == 0) return NULL;

    unsigned int lod = _lod - 1;
    unsigned int x = _x / 2;
    unsigned int y = _y / 2;
    return new TileKey( _face, lod, x, y, _profile.get());
}

TileKey*
TileKey::createAncestorKey( int ancestorLod ) const
{
    if ( ancestorLod > _lod ) return NULL;

    unsigned int x = _x, y = _y;
    for( int i=_lod; i > ancestorLod; i-- )
    {
        x /= 2;
        y /= 2;
    }
    return new TileKey( _face, ancestorLod, x, y, _profile.get() );
}

const GeoExtent&
TileKey::getGeoExtent() const
{
    return _extent;
}

TileKey::TileKey( unsigned int face, unsigned int lod, unsigned int tile_x, unsigned int tile_y, const Profile* profile)
{
    _face = face;
    _x = tile_x;
    _y = tile_y;
    _lod = lod;
    _profile = profile;

    //Compute the extent
    double width, height;
    _profile->getTileDimensions(_lod, width, height);

    double xmin = _profile->getExtent().xMin() + (width * (double)_x);
    double ymax = _profile->getExtent().yMax() - (height * (double)_y);
    double xmax = xmin + width;
    double ymin = ymax - height;

    _extent = GeoExtent( _profile->getSRS(), xmin, ymin, xmax, ymax );
}

bool TileKey::isGeodetic() const
{
    return _profile->getProfileType() == Profile::TYPE_GEODETIC; //GLOBAL_GEODETIC;
}

bool TileKey::isMercator() const
{
    return _profile->getProfileType() == Profile::TYPE_MERCATOR; //GLOBAL_MERCATOR;
}

bool TileKey::isProjected() const
{
    return _profile->getProfileType() == Profile::TYPE_LOCAL; //PROJECTED;
}