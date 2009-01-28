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
profile( TileGridProfile::UNKNOWN )
{
    //NOP
}

TileKey::TileKey( const TileKey& rhs ) :
key( rhs.key ),
profile( rhs.profile )
{
    //NOP
}

TileKey::TileKey( const std::string& _key, const TileGridProfile& _profile ) :
key( _key ),
profile( _profile )
{
    //NOP
}

const std::string&
TileKey::str() const
{
    return key;
}

const TileGridProfile&
TileKey::getProfile() const
{
    return profile;
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
    int x = 0;
    int y = 0;

    unsigned int lod = getLevelOfDetail();
    for( unsigned int i=0; i<lod; i++ )
    {
        x*=2;
        y*=2;
        switch( key[i] ) {
            case '1': x += 1; break;
            case '2': y += 1; break;
            case '3': x += 1; y += 1; break;
        }
    }
    out_tile_x = x;
    out_tile_y = y;
}

osgTerrain::TileID
TileKey::getTileId() const
{
    unsigned int x, y;
    getTileXY(x, y);
    return osgTerrain::TileID(getLevelOfDetail(), x, y);
}

unsigned int
TileKey::getLevelOfDetail() const
{
    return (unsigned int)key.length();
}

void
TileKey::getPixelExtents(unsigned int& xmin,
                         unsigned int& ymin,
                         unsigned int& xmax,
                         unsigned int& ymax,
                         const unsigned int &tile_size) const
{
    unsigned int lod = getLevelOfDetail();
    unsigned int px = 0, py = 0;
    unsigned int delta = getMapSizePixels(tile_size) >> 1;
    for( unsigned int i=0; i<lod; i++ )
    {
        switch( key[i] ) {
            case '1': px += delta; break;
            case '2': py += delta; break;
            case '3': px += delta; py += delta; break;
        }
        delta >>= 1;
    }
    xmin = px;
    ymin = py;
    xmax = px + (delta << 1);
    ymax = py + (delta << 1);
}

TileKey*
TileKey::getSubkey( unsigned int quadrant ) const
{
    if ( !subkeys[quadrant].valid() )
        const_cast<TileKey*>(this)->subkeys[quadrant] = new TileKey( key + (char)('0'+quadrant), profile );
    return subkeys[quadrant].get();
}

TileKey*
TileKey::createParentKey() const
{
    if (getLevelOfDetail() == 1) return NULL;
    return new TileKey(key.substr(0, key.length()-1),profile);
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
    double width =  profile.xMax() - profile.xMin();
    double height = profile.yMax() - profile.yMin();

    ymax = profile.yMax();
    xmin = profile.xMin();

    for( unsigned int lod = 0; lod < getLevelOfDetail(); lod++ )
    {
        width /= 2.0;
        height /= 2.0;

        char digit = key[lod];
        switch( digit )
        {
        case '1': xmin += width; break;
        case '2': ymax -= height; break;
        case '3': xmin += width; ymax -= height; break;
        }
    }

    ymin = ymax - height;
    xmax = xmin + width;

    return true;
}


TileKey::TileKey( unsigned int tile_x, unsigned int tile_y, unsigned int lod, TileGridProfile profile)
{       
    std::stringstream ss;
    for( unsigned i = lod; i > 0; i-- )
    {
        char digit = '0';
        unsigned int mask = 1 << (i-1);
        if ( (tile_x & mask) != 0 )
        {
            digit++;
        }
        if ( (tile_y & mask) != 0 )
        {
            digit += 2;
        }
        ss << digit;
    }
    key = ss.str();
    this->profile = profile;
}

bool TileKey::isGeodetic() const
{
    return profile.getProfileType() == TileGridProfile::GLOBAL_GEODETIC;
}

bool TileKey::isMercator() const
{
    return profile.getProfileType() == TileGridProfile::GLOBAL_MERCATOR;
}

bool TileKey::isProjected() const
{
    return profile.getProfileType() == TileGridProfile::PROJECTED;
}