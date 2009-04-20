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
#include <osgEarth/Mercator>
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

double
GeoExtent::width() const {
    return _xmax - _xmin;
}

double
GeoExtent::height() const {
    return _ymax - _ymin;
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

bool
GeoExtent::contains(const SpatialReference* srs, double x, double y)
{
    double local_x, local_y;
    if (srs->isEquivalentTo(_srs.get()))
    {
        //No need to transform
        local_x = x;
        local_y = y;
    }
    else
    {
        srs->transform(x, y, _srs.get(), local_x, local_y);
        //osgEarth::Mercator::latLongToMeters(y, x, local_x, local_y);        
    }

    return (local_x >= _xmin && local_x <= _xmax && local_y >= _ymin && local_y <= _ymax);
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
    double destXMin = xmin;
    double destYMin = ymin;
    double destXMax = xmax;
    double destYMax = ymax;

    osg::Image* new_image = ImageUtils::cropImage(
        _image.get(),
        _extent.xMin(), _extent.yMin(), _extent.xMax(), _extent.yMax(),
        destXMin, destYMin, destXMax, destYMax );

    //The destination extents may be different than the input extents due to not being able to crop along pixel boundaries.
    return new_image?
        new GeoImage( new_image, GeoExtent( _extent.getSRS(), destXMin, destYMin, destXMax, destYMax ) ) :
        NULL;
}


/***************************************************************************/
GeoHeightField::GeoHeightField(osg::HeightField* heightField, const GeoExtent& extent)
{
    _extent = extent;
    _heightField = heightField;

    double minx, miny, maxx, maxy;
    _extent.getBounds(minx, miny, maxx, maxy);

    _heightField->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
    _heightField->setXInterval( (maxx - minx)/(double)(_heightField->getNumColumns()-1) );
    _heightField->setYInterval( (maxy - miny)/(double)(_heightField->getNumRows()-1) );
    _heightField->setBorderWidth( 0 );
}

bool GeoHeightField::getElevation(const osgEarth::SpatialReference *srs, double x, double y, ElevationInterpolation interp, float &elevation)
{
    double local_x, local_y;
    if (_extent.getSRS()->isEquivalentTo(srs))
    {
        //No need to transform
        local_x = x;
        local_y = y;
    }
    else
    {
        if (!srs->transform(x, y, _extent.getSRS(), local_x, local_y)) return false;
    }

    if (_extent.contains(_extent.getSRS(), local_x, local_y))
    {
        elevation = HeightFieldUtils::getHeightAtLocation(_heightField.get(), local_x, local_y, interp);
        return true;
    }
    else
    {
        elevation = 0.0f;
        return false;
    }
}

const GeoExtent&
GeoHeightField::getGeoExtent() const
{
    return _extent;
}

const osg::HeightField*
GeoHeightField::getHeightField() const
{
    return _heightField.get();
}