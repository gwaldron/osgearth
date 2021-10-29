/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include "TimeSeriesImage"
#include <osg/NodeVisitor>

using namespace osgEarth;

#undef LC
#define LC "[TimeSeriesImage] "

TimeSeriesImage::TimeSeriesImage() :
    osg::Image(),
    _ptr(_images.end())
{
    // nop
}


void
TimeSeriesImage::insert(const DateTime& dt, const osg::Image* image)
{
    _images.insert(std::make_pair(dt.asTimeStamp(), image));

    if (_images.size() == 1)
    {
        setDateTime(dt);
    }

    _extent.expandBy(dt);
}

void
TimeSeriesImage::insert(const DateTime& dt, osg::ref_ptr<osg::Image> image)
{
    insert(dt, image.get());
}

void
TimeSeriesImage::update(osg::NodeVisitor* nv)
{
    DateTime dt(nv->getFrameStamp()->getSimulationTime());
    setDateTime(dt);
}

void
TimeSeriesImage::setDateTime(const DateTime& dt)
{
    // find the closest image frame:
    Table::iterator ptr = _images.lower_bound(dt.asTimeStamp());

    // if it's different, apply it to the underlying image
    if (ptr != _images.end() && ptr != _ptr)
    {
        _ptr = ptr;

        const osg::Image* image = _ptr->second.get();

        setImage(
            image->s(), image->t(), image->r(),
            image->getInternalTextureFormat(),
            image->getPixelFormat(), image->getDataType(),
            const_cast<unsigned char*>(image->data()),
            osg::Image::NO_DELETE,
            image->getPacking());

        setMipmapLevels(
            image->getMipmapLevels());
    }
}

const DateTimeExtent&
TimeSeriesImage::getDateTimeExtent() const
{
    return _extent;
}
