/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include <osgEarth/ImageMosaic>
#include <osgEarth/ImageUtils>
#include <osgEarth/HeightFieldUtils>
#include <osg/Notify>
#include <osg/Timer>
#include <osg/io_utils>

#define LC "[ImageMosaic] "

using namespace osgEarth;


/***************************************************************************/

TileImage::TileImage(osg::Image* image, const TileKey& key)
{
    _image = image;
    key.getExtent().getBounds(_minX, _minY, _maxX, _maxY);
    key.getTileXY(_tileX, _tileY);
}


/***************************************************************************/

ImageMosaic::ImageMosaic()
{
}

ImageMosaic::~ImageMosaic()
{
}

void ImageMosaic::getExtents(double &minX, double &minY, double &maxX, double &maxY)
{
    minX = DBL_MAX;
    maxX = -DBL_MAX;
    minY = DBL_MAX;
    maxY = -DBL_MAX;

    for (TileImageList::iterator i = _images.begin(); i != _images.end(); ++i)
    {
        minX = osg::minimum(i->_minX, minX);
        minY = osg::minimum(i->_minY, minY);
        maxX = osg::maximum(i->_maxX, maxX);
        maxY = osg::maximum(i->_maxY, maxY);
    }
}

osg::Image*
ImageMosaic::createImage()
{
    if (_images.size() == 0)
    {
        OE_NOTICE << "ImageMosaic has no images..." << std::endl;
        return 0;
    }

    unsigned int tileWidth = _images[0]._image->s();
    unsigned int tileHeight = _images[0]._image->t();

    //OE_NOTICE << "TileDim " << tileWidth << ", " << tileHeight << std::endl;

    unsigned int minTileX = _images[0]._tileX;
    unsigned int minTileY = _images[0]._tileY;
    unsigned int maxTileX = _images[0]._tileX;
    unsigned int maxTileY = _images[0]._tileY;

    //Compute the tile size.
    for (TileImageList::iterator i = _images.begin(); i != _images.end(); ++i)
    {
        if (i->_tileX < minTileX) minTileX = i->_tileX;
        if (i->_tileY < minTileY) minTileY = i->_tileY;

        if (i->_tileX > maxTileX) maxTileX = i->_tileX;
        if (i->_tileY > maxTileY) maxTileY = i->_tileY;
    }

    unsigned int tilesWide = maxTileX - minTileX + 1;
    unsigned int tilesHigh = maxTileY - minTileY + 1;

    unsigned int pixelsWide = tilesWide * tileWidth;
    unsigned int pixelsHigh = tilesHigh * tileHeight;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage(pixelsWide, pixelsHigh, 1, _images[0]._image->getPixelFormat(), _images[0]._image->getDataType());
    image->setInternalTextureFormat(_images[0]._image->getInternalTextureFormat()); 

    //Initialize the image to be completely transparent/black
    //memset(image->data(), 0, image->getImageSizeInBytes());

    //Composite the incoming images into the master image
    for (TileImageList::iterator i = _images.begin(); i != _images.end(); ++i)
    {
        osg::Image* sourceTile = i->getImage();

        //Determine the indices in the master image for this image
        int dstX = (i->_tileX - minTileX) * tileWidth;
        int dstY = (maxTileY - i->_tileY) * tileHeight;
        ImageUtils::copyAsSubImage(sourceTile, image.get(), dstX, dstY);
    }

    return image.release();
}

/***************************************************************************/
