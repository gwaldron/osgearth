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

#include <osgEarth/Compositing>
#include <osgEarth/ImageUtils>
#include <osgEarth/HeightFieldUtils>
#include <osg/Notify>
#include <osg/Timer>
#include <osg/io_utils>

using namespace osgEarth;


TileImage::TileImage(osg::Image* image, const TileKey *key)
{
    _image = image;
    key->getGeoExtents(_minX, _minY, _maxX, _maxY);
    key->getTileXY(_tileX, _tileY);
}


/***************************************************************************/

MultiImage::MultiImage()
{
}

MultiImage::~MultiImage()
{
}

void MultiImage::getExtents(double &minX, double &minY, double &maxX, double &maxY)
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


osg::Image* MultiImage::createImage(double minx, double miny, double maxx, double maxy)
{
    if (_images.size() == 0)
    {
        osg::notify(osg::NOTICE) << "MultiImage has no images..." << std::endl;
        return 0;
    }

    unsigned int tileWidth = _images[0]._image->s();
    unsigned int tileHeight = _images[0]._image->t();

    //osg::notify(osg::NOTICE) << "TileDim " << tileWidth << ", " << tileHeight << std::endl;

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

    //osg::notify(osg::NOTICE) << "Creating image that is " << tilesWide << " x " << tilesHigh << " tiles and " << pixelsWide << ", " << pixelsHigh << " pixels " << std::endl;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage(pixelsWide, pixelsHigh, 1, GL_RGB, GL_UNSIGNED_BYTE);

    {
      //osg::Timer_t start = osg::Timer::instance()->tick();
      //Composite the incoming images into the master image
      for (TileImageList::iterator i = _images.begin(); i != _images.end(); ++i)
      {
        //Determine the indices in the master image for this image
        int dstX = (i->_tileX - minTileX) * tileWidth;
        int dstY = (maxTileY - i->_tileY) * tileHeight;
        //osg::notify(osg::NOTICE) << "Copying image to " << dstX << ", " << dstY << std::endl;
        ImageUtils::copyAsSubImage(i->getImage(), image.get(), dstX, dstY);
      }
      //osg::Timer_t end = osg::Timer::instance()->tick();
      //osg::notify(osg::NOTICE) << "Mosaic for " << _images.size() << " images took " << osg::Timer::instance()->delta_m(start, end) << std::endl;
    }
   
    double src_minx, src_miny, src_maxx, src_maxy;
    getExtents(src_minx, src_miny, src_maxx, src_maxy);

    {
      //osg::Timer_t start = osg::Timer::instance()->tick();
      image = ImageUtils::cropImage(image.get(), src_minx, src_miny, src_maxx, src_maxy, minx, miny, maxx, maxy);
      //osg::Timer_t end = osg::Timer::instance()->tick();
      //osg::notify(osg::NOTICE) << "Crop took " << osg::Timer::instance()->delta_m(start, end) << std::endl;
    }
    //osg::notify(osg::NOTICE) << "Cropped image is " << image->s() << " x " << image->t() << std::endl;

    //We shouldn't need to manually resize the image if it isn't a power of two.  If the card supports NPOT textures
    //it should work fine, and if it doesn't, OSG will resize the image for us.
    /*unsigned int new_s = osg::Image::computeNearestPowerOfTwo(image->s());
    unsigned int new_t = osg::Image::computeNearestPowerOfTwo(image->t());

    if (new_s != image->s() || new_t != image->t())
    {
      //osg::Timer_t start = osg::Timer::instance()->tick();
      osg::notify(osg::INFO) << "Resizing image from " << image->s() << ", " << image->t() << " to " << new_s << ", " << new_t << std::endl;
      image = ImageUtils::resizeImage(image.get(), new_s, new_t);
      //osg::Timer_t end = osg::Timer::instance()->tick();
      //osg::notify(osg::NOTICE) << "Resize took " << osg::Timer::instance()->delta_m(start, end) << std::endl;
    }*/

    return image.release();
}


/***************************************************************************/

Compositor::Compositor()
{
    //NOP
}


osg::Image*
Compositor::mosaicImages( const TileKey* key, TileSource* source ) const
{
    osg::Image* result = NULL;
    
    //Determine the intersecting keys and create and extract an appropriate image from the tiles
    std::vector< osg::ref_ptr<const TileKey> > intersectingTiles;
    source->getProfile()->getIntersectingTiles(key, intersectingTiles);
    if (intersectingTiles.size() > 0)
    {
        double dst_minx, dst_miny, dst_maxx, dst_maxy;
        key->getGeoExtents(dst_minx, dst_miny, dst_maxx, dst_maxy);

        osg::ref_ptr<MultiImage> mi = new MultiImage;
        for (unsigned int j = 0; j < intersectingTiles.size(); ++j)
        {
            double minX, minY, maxX, maxY;
            intersectingTiles[j]->getGeoExtents(minX, minY, maxX, maxY);

            //osg::notify(osg::NOTICE) << "\t Intersecting Tile " << j << ": " << minX << ", " << minY << ", " << maxX << ", " << maxY << std::endl;

            osg::ref_ptr<osg::Image> img = source->createImage(intersectingTiles[j].get());
            if (img.valid())
            {
                mi->getImages().push_back(TileImage(img.get(), intersectingTiles[j].get()));
            }
            else
            {
                //If we couldn't create an image that is needed to composite, return NULL
                osg::notify(osg::INFO) << "Couldn't create image for MultiImage " << std::endl;
                return 0;
            }
        }
        result = mi->createImage(dst_minx, dst_miny, dst_maxx, dst_maxy);
    }

    return result;
}


osg::HeightField*
Compositor::compositeHeightFields( const TileKey* key, TileSource* source ) const
{        
    osg::HeightField* result = NULL;

    //Determine the intersecting keys and create and extract an appropriate image from the tiles

    std::vector< osg::ref_ptr<const TileKey> > intersectingTiles;
    source->getProfile()->getIntersectingTiles(key, intersectingTiles);
    if (intersectingTiles.size() > 0)
    {
        double dst_minx, dst_miny, dst_maxx, dst_maxy;
        key->getGeoExtents(dst_minx, dst_miny, dst_maxx, dst_maxy);

        std::vector<osg::ref_ptr<osg::HeightField> > heightFields;
        for (unsigned int j = 0; j < intersectingTiles.size(); ++j)
        {
            double minX, minY, maxX, maxY;
            intersectingTiles[j]->getGeoExtents(minX, minY, maxX, maxY);

            //osg::notify(osg::NOTICE) << "\t Intersecting Tile " << j << ": " << minX << ", " << minY << ", " << maxX << ", " << maxY << std::endl;

            osg::ref_ptr<osg::HeightField> hf = source->createHeightField(intersectingTiles[j].get());
            if (hf.valid())
            {
                //Need to init this before extracting the heightfield
                hf->setOrigin( osg::Vec3d( minX, minY, 0.0 ) );
                hf->setXInterval( (maxX - minX)/(double)(hf->getNumColumns()-1) );
                hf->setYInterval( (maxY - minY)/(double)(hf->getNumRows()-1) );
                heightFields.push_back(hf.get());
            }
            else
            {
                //If we couldn't create a heightfield that is needed to composite, return NULL
                osg::notify(osg::NOTICE) << "Couldn't create heightfield" << std::endl;
                return 0;
            }
        }
        if (heightFields.size() > 0)
        {
            result = HeightFieldUtils::compositeHeightField(dst_minx, dst_miny, dst_maxx, dst_maxy, heightFields[0]->getNumColumns(), heightFields[0]->getNumRows(), heightFields);
        }
    }

    return result;
}