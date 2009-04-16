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

#include <osgEarth/GeoData>
#include <osgEarth/ImageUtils>
#include <osgEarth/HeightFieldUtils>
#include <osg/Notify>

using namespace osgEarth;


GeoExtent GeoExtent::INVALID = GeoExtent();


GeoExtent::GeoExtent()
{
    //NOP - invalid
}

GeoExtent::GeoExtent(const SpatialReference* srs,
                     double xmin, double ymin, double xmax, double ymax) :
_srs( srs ),
_xmin(xmin),_ymin(ymin),_xmax(xmax),_ymax(ymax)
{
    //NOP
}

GeoExtent::GeoExtent( const GeoExtent& rhs ) :
_srs( rhs._srs ),
_xmin( rhs._xmin ), _ymin( rhs._ymin ), _xmax( rhs._xmax ), _ymax( rhs._ymax )
{
    //NOP
}

bool
GeoExtent::operator == ( const GeoExtent& rhs ) const
{
    if ( !isValid() && !rhs.isValid() )
        return true;

    else return
        isValid() && rhs.isValid() &&
        _xmin == rhs._xmin &&
        _ymin == rhs._ymin &&
        _xmax == rhs._xmax &&
        _ymax == rhs._ymax &&
        _srs.valid() && rhs._srs.valid() &&
        _srs->isEquivalentTo( rhs._srs.get() );
}

bool
GeoExtent::operator != ( const GeoExtent& rhs ) const
{
    return !( *this == rhs );
}

bool
GeoExtent::isValid() const {
    return _srs.valid();
}

const SpatialReference*
GeoExtent::getSRS() const {
    return _srs.get(); 
}

double
GeoExtent::xMin() const {
    return _xmin;
}

double
GeoExtent::yMin() const {
    return _ymin;
}

double
GeoExtent::xMax() const {
    return _xmax; 
}

double
GeoExtent::yMax() const {
    return _ymax; 
}

GeoExtent
GeoExtent::transform( const SpatialReference* to_srs ) const 
{       
    if ( isValid() && to_srs )
    {
        double to_xmin, to_ymin, to_xmax, to_ymax;
        int err = 0;
        err += _srs->transform( _xmin, _ymin, to_srs, to_xmin, to_ymin )? 0 : 1;
        err += _srs->transform( _xmax, _ymax, to_srs, to_xmax, to_ymax )? 0 : 1;
        if ( err > 0 )
        {
            osg::notify(osg::WARN)
                << "[osgEarth] Warning, failed to transform an extent from "
                << _srs->getName() << " to "
                << to_srs->getName() << std::endl;
        }
        else
        {
            return GeoExtent( to_srs, to_xmin, to_ymin, to_xmax, to_ymax );
        }
    }
    return GeoExtent(); // invalid
}

void
GeoExtent::getBounds(double &xmin, double &ymin, double &xmax, double &ymax) const
{
    xmin = _xmin;
    ymin = _ymin;
    xmax = _xmax;
    ymax = _ymax;
}

/***************************************************************************/


GeoImage::GeoImage(osg::Image* image, const GeoExtent& extent ) :
_image(image),
_extent(extent)
{
    //NOP
}

osg::Image*
GeoImage::getImage() const {
    return _image.get();
}

const SpatialReference*
GeoImage::getSRS() const {
    return _extent.getSRS();
}

const GeoExtent&
GeoImage::getExtent() const {
    return _extent;
}


GeoImage*
GeoImage::crop( double xmin, double ymin, double xmax, double ymax ) const
{
    osg::Image* new_image = ImageUtils::cropImage(
        _image.get(),
        _extent.xMin(), _extent.yMin(), _extent.xMax(), _extent.yMax(),
        xmin, ymin, xmax, ymax );

    return new_image?
        new GeoImage( new_image, GeoExtent( _extent.getSRS(), xmin, ymin, xmax, ymax ) ) :
        NULL;
}
