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

/************************************************************************/

HeightFieldExtractor::HeightFieldExtractor()
{
    //NOP
}

HeightFieldExtractor::HeightFieldExtractor(const TileKey *key, const osg::HeightField *heightField):
_key(key),
_heightField(heightField)
{
}


float
HeightFieldExtractor::getInterpolatedValue(const float &c, const float &r)
{
    //osg::notify(osg::INFO) << "getInterpolateValue: (" << c << ", " << r << ")" << std::endl;
    int rowMin = osg::maximum((int)floor(r), 0);
    int rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(_heightField->getNumRows()-1)), 0);
    int colMin = osg::maximum((int)floor(c), 0);
    int colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(_heightField->getNumColumns()-1)), 0);

    if (rowMin > rowMax) rowMin = rowMax;
    if (colMin > colMax) colMin = colMax;

    float urHeight = _heightField->getHeight(colMax, rowMax);
    float llHeight = _heightField->getHeight(colMin, rowMin);
    float ulHeight = _heightField->getHeight(colMin, rowMax);
    float lrHeight = _heightField->getHeight(colMax, rowMin);

    //osg::notify(osg::INFO) << "Heights (ll, lr, ul, ur) ( " << llHeight << ", " << urHeight << ", " << ulHeight << ", " << urHeight << std::endl;

    float result = 0.0f;

    //Check for exact value
    if ((colMax == colMin) && (rowMax == rowMin))
    {
        //osg::notify(osg::NOTICE) << "Exact value" << std::endl;
        result = _heightField->getHeight((int)c, (int)r);
    }
    else if (colMax == colMin)
    {
        //osg::notify(osg::NOTICE) << "Vertically" << std::endl;
        //Linear interpolate vertically
        result = ((float)rowMax - r) * llHeight + (r - (float)rowMin) * ulHeight;
    }
    else if (rowMax == rowMin)
    {
        //osg::notify(osg::NOTICE) << "Horizontally" << std::endl;
        //Linear interpolate horizontally
        result = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
    }
    else
    {
        //osg::notify(osg::NOTICE) << "Bilinear" << std::endl;
        //Bilinear interpolate
        float r1 = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
        float r2 = ((float)colMax - c) * ulHeight + (c - (float)colMin) * urHeight;

        osg::notify(osg::INFO) << "r1, r2 = " << r1 << " , " << r2 << std::endl;

        result = ((float)rowMax - r) * r1 + (r - (float)rowMin) * r2;
    }

    osg::notify(osg::INFO) << "Result : " << result << std::endl << std::endl;

    return result;
}

osg::HeightField*
HeightFieldExtractor::extractChild(const osgEarth::TileKey* childKey, const unsigned int &width, const unsigned int &height)
{
    //Check to make sure the child key is a subkey of the parent
    if ((_key->getProfile().getProfileType() == childKey->getProfile().getProfileType()) && 
        (childKey->str().length() > _key->str().length()) &&
        (childKey->str().find(_key->str()) != 0))
    {
        osg::notify(osg::NOTICE) << childKey->str() << " is not a child of " << _key->str() << std::endl;
    }

    //Determine the coordinates of the child heightfield
    std::string remainingLod = childKey->str().substr(_key->str().length());

    float x = 0;
    float y = 0;
    float deltaX = (float)_heightField->getNumColumns();
    float deltaY = (float)_heightField->getNumRows();
    for (unsigned int i = 0; i < remainingLod.length(); ++i)
    {
        deltaX /= 2.0f;
        deltaY /= 2.0f;
        switch (remainingLod[i])
        {
        case '0': y += deltaY; break;
        case '1': x += deltaX; y += deltaY; break;
        case '3': x += deltaX; break;
        }
    }

    float minX = x;
    float minY = y;
    float maxX = x + deltaX;
    float maxY = y + deltaY;

    osg::HeightField *hf = new osg::HeightField;
    hf->allocate(width, height);

    deltaX = deltaX / (float)width;
    deltaY = deltaY / (float)height;

    for (unsigned int c = 0; c < width; ++c)
    {
        x = minX + (float)c * deltaX;
        for (unsigned int r = 0; r < height; ++r)
        {
            y = minY + (float)r * deltaY;
            float height = getInterpolatedValue(x, y);
            //osg::notify(osg::NOTICE) << "Interpolated height ( " << x << ", " << y << ") = " << height << std::endl;
            hf->setHeight(c, r, height);
        }
    }
    return hf;
}