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

#include <osgEarth/MultiImage>
#include <osg/Notify>
#include <osg/io_utils>

using namespace osgEarth;

GeoImage::GeoImage(osg::Image * image, double minX, double minY, double maxX, double maxY)
{
    _image = image;
    _minX = minX;
    _minY = minY;
    _maxX = maxX;
    _maxY = maxY;
}

void GeoImage::pixelToWorld(double pixelX, double pixelY,
                  double &worldX, double &worldY)
{
    worldX = _minX + ((pixelX) / (double)_image->s());
    worldX = _minY + ((pixelY) / (double)_image->t());        
}

void GeoImage::worldToPixel(double worldX, double worldY,
                  double &pixelX, double &pixelY)
{
    pixelX = (worldX - _minX) / (_maxX - _minX) * (double)_image->s();
    pixelY = (worldY - _minY) / (_maxY - _minY) * (double)_image->t();
}

bool GeoImage::contains(double x, double y)
{
    return (_minX <= x && _maxX >= x && _minY <= y && _maxY >= y);
}


/***************************************************************************/
MultiImage::MultiImage()
{
}

MultiImage::~MultiImage()
{
}

osg::Vec4 MultiImage::getColor(double x, double y)
{
    osg::Vec4 color(0,0,0,0);
    int index = 0;
    for (GeoImageList::iterator i = _images.begin(); i != _images.end(); ++i)
    {
        if (i->getImage() && i->contains(x,y))
        {
            double px, py;
            i->worldToPixel(x, y, px, py);
            px = osg::clampBetween(px, 0.0, (double)i->getImage()->s()-1);
            py = osg::clampBetween(py, 0.0, (double)i->getImage()->t()-1);

            if (px >= 0 && py >= 0)
            {
                unsigned char *data = i->getImage()->data((int)px, (int)py);
                float r = float(*(data + 0))/255.0f;
                float g = float(*(data + 1))/255.0f;
                float b = float(*(data + 2))/255.0f;
                color = osg::Vec4(r,g,b,1);
                //osg::notify(osg::NOTICE) << "Returning color " << color<< std::endl;
                break;
            }
        }
        index++;
    }
    return color;
}

osg::Image* MultiImage::createImage(unsigned int width, unsigned int height,
                                    double minX, double minY, double maxX, double maxY)
{
    if (_images.size() == 0)
    {
        osg::notify(osg::NOTICE) << "MultiImage has no images..." << std::endl;
        return 0;
    }
    osg::Image* image = new osg::Image;
    image->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);

    double dx = (maxX - minX) / (double)(width);
    double dy = (maxY - minY) / (double)(height);

    for (unsigned int c = 0; c < width; ++c)
    {
        double gx = minX + (double)c * dx;
        for (unsigned int r = 0; r < height; ++r)
        {
            double gy = minY + (double)r * dy;
            osg::Vec4 color = getColor(gx, gy);
            //osg::notify(osg::NOTICE) << "Got color " << color << std::endl;
            *(image->data(c,r) + 0) = (unsigned char) (color.r()*255.0f);
            *(image->data(c,r) + 1) = (unsigned char) (color.g()*255.0f);
            *(image->data(c,r) + 2) = (unsigned char) (color.b()*255.0f);
            *(image->data(c,r) + 3) = (unsigned char) (color.a()*255.0f);
        }
    }
    return image;
}