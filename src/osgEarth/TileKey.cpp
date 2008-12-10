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
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>

using namespace osgEarth;

TileKey::TileKey() :
profile( TileGridProfile( 0, 0, 256, 256, 256 ) )
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

std::string
TileKey::getName() const
{
    return getTypeCode() + str();
}

const TileGridProfile&
TileKey::getProfile() const
{
    return profile;
}

int
TileKey::getMapSizePixels() const
{
    return getMapSizePixels( profile.pixelsPerTile(), getLevelOfDetail() );
}

/*static*/ int
TileKey::getMapSizePixels( int tile_size, int lod )
{
    return tile_size << lod;
}

int
TileKey::getMapSizeTiles() const
{
    return getMapSizePixels() / profile.pixelsPerTile();
}

/************************************************************************/

TileKey*
TileKeyFactory::createFromName( const std::string& input )
{
    TileKey* result = NULL;
    if ( input.length() >= 1 )
    {
        if ( input.substr( 0, 1 ) == PlateCarreTileKey::TYPE_CODE )
            result = new PlateCarreTileKey( input.substr( 1 ) );
        else if ( input.substr( 0, 1 ) == MercatorTileKey::TYPE_CODE )
            result = new MercatorTileKey( input.substr( 1 ) );
    }
    return result;
}

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
    if ((_key->getTypeCode() == childKey->getTypeCode()) && 
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