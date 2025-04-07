/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
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
