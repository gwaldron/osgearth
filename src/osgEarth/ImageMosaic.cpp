/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/ImageMosaic>

#define LC "[ImageMosaic] "

using namespace osgEarth;
using namespace osgEarth::Util;


/***************************************************************************/

TileImage::TileImage(const osg::Image* image, const TileKey& key)
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
        return nullptr;
    }

    // find the first valid tile and use its size as the mosaic tile size
    TileImage* tile = 0L;
    for (int i = 0; i<_images.size() && !tile; ++i)
        if (_images[i]._image.valid())
            tile = &_images[i];

    if ( !tile )
        return 0L;

    unsigned int tileWidth = tile->_image->s();
    unsigned int tileHeight = tile->_image->t();

    //OE_NOTICE << "TileDim " << tileWidth << ", " << tileHeight << std::endl;

    unsigned int minTileX = tile->_tileX;
    unsigned int minTileY = tile->_tileY;
    unsigned int maxTileX = tile->_tileX;
    unsigned int maxTileY = tile->_tileY;

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
	unsigned int tileDepth = tile->_image->r();

    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage(pixelsWide, pixelsHigh, tileDepth, tile->_image->getPixelFormat(), tile->_image->getDataType());
    image->setInternalTextureFormat(tile->_image->getInternalTextureFormat());

    //Initialize the image to be completely white!
    ImageUtils::PixelWriter write(image.get());
    write.assign(osg::Vec4(1,1,1,0));

    //Composite the incoming images into the master image
    for (TileImageList::iterator i = _images.begin(); i != _images.end(); ++i)
    {
        //Determine the indices in the master image for this image
        int dstX = (i->_tileX - minTileX) * tileWidth;
        int dstY = (maxTileY - i->_tileY) * tileHeight;

        const osg::Image* sourceTile = i->getImage();
        if ( sourceTile )
        {
            ImageUtils::copyAsSubImage(sourceTile, image.get(), dstX, dstY);
        }
    }

    return image.release();
}

/***************************************************************************/
