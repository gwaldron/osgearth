/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/Cube>

#include <osg/Notify>
#include <osg/Timer>

#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>
#include <memory.h>

#include <sstream>
#include <iomanip>

#define LC "[GeoData] "

using namespace osgEarth;


Bounds::Bounds() :
osg::BoundingBoxImpl<osg::Vec3d>( DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX )
{
    //nop
}

Bounds::Bounds(double xmin, double ymin, double xmax, double ymax ) :
osg::BoundingBoxImpl<osg::Vec3d>( xmin, ymin, -DBL_MAX, xmax, ymax, DBL_MAX )
{
    //nop
}

bool
Bounds::isValid() const
{
    return xMin() <= xMax() && yMin() <= yMax();
}

bool 
Bounds::contains(double x, double y ) const
{
    return 
        isValid() &&
        x >= xMin() && x <= xMax() && y >= yMin() && y <= yMax();
}

bool
Bounds::contains(const Bounds& rhs) const
{
    return 
        isValid() && rhs.isValid() && 
        xMin() <= rhs.xMin() && xMax() >= rhs.xMax() &&
        yMin() <= rhs.yMin() && yMax() >= rhs.yMax();
}

void
Bounds::expandBy( double x, double y )
{
    osg::BoundingBoxImpl<osg::Vec3d>::expandBy( x, y, 0 );
}

void
Bounds::expandBy( double x, double y, double z )
{
    osg::BoundingBoxImpl<osg::Vec3d>::expandBy( x, y, z );
}

void
Bounds::expandBy( const Bounds& rhs )
{
    osg::BoundingBoxImpl<osg::Vec3d>::expandBy( rhs );
}

Bounds 
Bounds::unionWith(const Bounds& rhs) const
{
    if ( valid() && !rhs.valid() ) return *this;
    if ( !valid() && rhs.valid() ) return rhs;

    Bounds u;
    if ( intersects(rhs) ) {
        u.xMin() = xMin() >= rhs.xMin() && xMin() <= rhs.xMax() ? xMin() : rhs.xMin();
        u.xMax() = xMax() >= rhs.xMin() && xMax() <= rhs.xMax() ? xMax() : rhs.xMax();
        u.yMin() = yMin() >= rhs.yMin() && yMin() <= rhs.yMax() ? yMin() : rhs.yMin();
        u.yMax() = yMax() >= rhs.yMin() && yMax() <= rhs.yMax() ? yMax() : rhs.yMax();
        u.zMin() = zMin() >= rhs.zMin() && zMin() <= rhs.zMax() ? zMin() : rhs.zMin();
        u.zMax() = zMax() >= rhs.zMin() && zMax() <= rhs.zMax() ? zMax() : rhs.zMax();
    }
    return u;
}

Bounds
Bounds::intersectionWith(const Bounds& rhs) const 
{
    if ( valid() && !rhs.valid() ) return *this;
    if ( !valid() && rhs.valid() ) return rhs;

    if ( this->contains(rhs) ) return rhs;
    if ( rhs.contains(*this) ) return *this;

    if ( !intersects(rhs) ) return Bounds();

    double xmin, xmax, ymin, ymax;

    xmin = ( xMin() > rhs.xMin() && xMin() < rhs.xMax() ) ? xMin() : rhs.xMin();
    xmax = ( xMax() > rhs.xMin() && xMax() < rhs.xMax() ) ? xMax() : rhs.xMax();
    ymin = ( yMin() > rhs.yMin() && yMin() < rhs.yMax() ) ? yMin() : rhs.yMin();
    ymax = ( yMax() > rhs.yMin() && yMax() < rhs.yMax() ) ? yMax() : rhs.yMax();

    return Bounds(xmin, ymin, xmax, ymax);
}

double
Bounds::width() const {
    return xMax()-xMin();
}

double
Bounds::height() const {
    return yMax()-yMin();
}

double
Bounds::depth() const {
    return zMax()-zMin();
}

osg::Vec2d
Bounds::center2d() const {
    osg::Vec3d c = center();
    return osg::Vec2d( c.x(), c.y() );
}

double
Bounds::radius2d() const {
    return (center2d() - osg::Vec2d(xMin(),yMin())).length();
}

std::string
Bounds::toString() const {
    std::stringstream buf;
    buf << "(" << xMin() << "," << yMin() << " => " << xMax() << "," << yMax() << ")";
    std::string result = buf.str();
    return result;
}

/*************************************************************/

GeoExtent GeoExtent::INVALID = GeoExtent();


GeoExtent::GeoExtent():
_xmin(FLT_MAX),
_ymin(FLT_MAX),
_xmax(-FLT_MAX),
_ymax(-FLT_MAX)
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


GeoExtent::GeoExtent( const SpatialReference* srs, const Bounds& bounds ) :
_srs( srs ),
_xmin( bounds.xMin() ),
_ymin( bounds.yMin() ),
_xmax( bounds.xMax() ),
_ymax( bounds.yMax() )
{
    //nop
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
GeoExtent::isValid() const
{
    return _srs.valid() && width() > 0 && height() > 0;
}

const SpatialReference*
GeoExtent::getSRS() const {
    return _srs.get(); 
}

double
GeoExtent::width() const
{
    return crossesDateLine()?
        (180-_xmin) + (_xmax+180) :
        _xmax - _xmin;
}

double
GeoExtent::height() const
{
    return _ymax - _ymin;
}

void
GeoExtent::getCentroid( double& out_x, double& out_y ) const
{
    out_x = _xmin+width()/2.0;
    out_y = _ymin+height()/2.0;
}

bool
GeoExtent::crossesDateLine() const
{
    return _xmax < _xmin;
    //return _srs.valid() && _srs->isGeographic() && _xmax < _xmin;
}

bool
GeoExtent::splitAcrossDateLine( GeoExtent& out_first, GeoExtent& out_second ) const
{
    bool success = false;

    if ( crossesDateLine() )
    {
        if ( _srs->isGeographic() )
        {
            out_first = GeoExtent( _srs.get(), _xmin, _ymin, 180.0, _ymax );
            out_second = GeoExtent( _srs.get(), -180.0, _ymin, _xmax, _ymax );
            success = true;
        }
        else
        {
            GeoExtent latlong_extent = transform( _srs->getGeographicSRS() );
            GeoExtent first, second;
            if ( latlong_extent.splitAcrossDateLine( first, second ) )
            {
                out_first = first.transform( _srs.get() );
                out_second = second.transform( _srs.get() );
                success = out_first.isValid() && out_second.isValid();
            }
        }
    }
    return success;
}

GeoExtent
GeoExtent::transform( const SpatialReference* to_srs ) const 
{       
    if ( isValid() && to_srs )
    {
        double xmin = _xmin, ymin = _ymin;
        double xmax = _xmax, ymax = _ymax;
        
        if ( _srs->transformExtent( to_srs, xmin, ymin, xmax, ymax ) )
        {
            return GeoExtent( to_srs, xmin, ymin, xmax, ymax );
        }

    }
    return GeoExtent::INVALID;
}

void
GeoExtent::getBounds(double &xmin, double &ymin, double &xmax, double &ymax) const
{
    xmin = _xmin;
    ymin = _ymin;
    xmax = _xmax;
    ymax = _ymax;
}

Bounds
GeoExtent::bounds() const
{
    return Bounds( _xmin, _ymin, _xmax, _ymax );
}

//TODO:: support crossesDateLine!
bool
GeoExtent::contains(double x, double y, const SpatialReference* srs) const
{
    double local_x = x, local_y = y;
    if (srs &&
        !srs->isEquivalentTo( _srs.get() ) &&
        !srs->transform(x, y, _srs.get(), local_x, local_y) )
    {
        return false;
    }
    else
    {
        //Account for small rounding errors along the edges of the extent
        if (osg::equivalent(_xmin, local_x)) local_x = _xmin;
        if (osg::equivalent(_xmax, local_x)) local_x = _xmax;
        if (osg::equivalent(_ymin, local_y)) local_y = _ymin;
        if (osg::equivalent(_ymax, local_y)) local_y = _ymax;
        return local_x >= _xmin && local_x <= _xmax && local_y >= _ymin && local_y <= _ymax;
    }
}

bool
GeoExtent::contains( const Bounds& rhs ) const
{
    return 
        rhs.xMin() >= _xmin &&
        rhs.yMin() >= _ymin &&
        rhs.xMax() <= _xmax &&
        rhs.yMax() <= _ymax;
}

bool
GeoExtent::intersects( const GeoExtent& rhs ) const
{
    bool valid = isValid();
    if ( !valid ) return false;
    bool exclusive =
        _xmin > rhs.xMax() ||
        _xmax < rhs.xMin() ||
        _ymin > rhs.yMax() ||
        _ymax < rhs.yMin();
    return !exclusive;
}

void
GeoExtent::expandToInclude( double x, double y )
{
    if ( x < _xmin ) _xmin = x;
    if ( x > _xmax ) _xmax = x;
    if ( y < _ymin ) _ymin = y;
    if ( y > _ymax ) _ymax = y;
}

void GeoExtent::expandToInclude(const Bounds& rhs)
{
    expandToInclude( rhs.xMin(), rhs.yMin() );
    expandToInclude( rhs.xMax(), rhs.yMax() );
}

GeoExtent
GeoExtent::intersectionSameSRS( const Bounds& rhs ) const
{
    Bounds b(
        osg::maximum( xMin(), rhs.xMin() ),
        osg::maximum( yMin(), rhs.yMin() ),
        osg::minimum( xMax(), rhs.xMax() ),
        osg::minimum( yMax(), rhs.yMax() ) );

    return b.width() > 0 && b.height() > 0 ? GeoExtent( getSRS(), b ) : GeoExtent::INVALID;
}

void
GeoExtent::scale(double x_scale, double y_scale)
{
    double orig_width = width();
    double orig_height = height();

    double new_width  = orig_width  * x_scale;
    double new_height = orig_height * y_scale;

    double halfXDiff = (new_width - orig_width) / 2.0;
    double halfYDiff = (new_height - orig_height) /2.0;

    _xmin -= halfXDiff;
    _xmax += halfXDiff;
    _ymin -= halfYDiff;
    _ymax += halfYDiff;
}

void
GeoExtent::expand( double x, double y )
{
    _xmin -= .5*x;
    _xmax += .5*x;
    _ymin -= .5*y;
    _ymax += .5*y;
}

double
GeoExtent::area() const
{
    return width() * height();
}

std::string
GeoExtent::toString() const
{
    std::stringstream buf;
    if ( !isValid() )
        buf << "INVALID";
    else
        buf << "MIN=" << _xmin << "," << _ymin << " MAX=" << _xmax << "," << _ymax;

    buf << ", SRS=" << _srs->getName();

	std::string bufStr;
	bufStr = buf.str();
    return bufStr;
}


/***************************************************************************/

DataExtent::DataExtent(const osgEarth::GeoExtent &extent, unsigned int minLevel,  unsigned int maxLevel):
GeoExtent(extent),
_minLevel(minLevel),
_maxLevel(maxLevel)
{
}

unsigned int
DataExtent::getMinLevel() const
{
    return _minLevel;
}

unsigned int
DataExtent::getMaxLevel() const
{
    return _maxLevel;
}


/***************************************************************************/

// static
GeoImage GeoImage::INVALID( 0L, GeoExtent::INVALID );

GeoImage::GeoImage() :
_image(0L),
_extent( GeoExtent::INVALID )
{
    //nop
}


GeoImage::GeoImage( osg::Image* image, const GeoExtent& extent ) :
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

double
GeoImage::getUnitsPerPixel() const {
	double uppw = _extent.width() / (double)_image->s();
	double upph = _extent.height() / (double)_image->t();
	return (uppw + upph) / 2.0;
}

GeoImage
GeoImage::crop( const GeoExtent& extent, bool exact, unsigned int width, unsigned int height  ) const
{
    //Check for equivalence
    if ( extent.getSRS()->isEquivalentTo( getSRS() ) )
    {
        //If we want an exact crop or they want to specify the output size of the image, use GDAL
        if (exact || width != 0 || height != 0 )
        {
            OE_DEBUG << "[osgEarth::GeoImage::crop] Performing exact crop" << std::endl;

            //Suggest an output image size
            if (width == 0 || height == 0)
            {
                double xRes = (getExtent().xMax() - getExtent().xMin()) / (double)_image->s();
                double yRes = (getExtent().yMax() - getExtent().yMin()) / (double)_image->t();

                width =  osg::maximum(1u, (unsigned int)((extent.xMax() - extent.xMin()) / xRes));
                height = osg::maximum(1u, (unsigned int)((extent.yMax() - extent.yMin()) / yRes));

                OE_DEBUG << "[osgEarth::GeoImage::crop] Computed output image size " << width << "x" << height << std::endl;
            }

            //Note:  Passing in the current SRS simply forces GDAL to not do any warping
            return reproject( getSRS(), &extent, width, height);
        }
        else
        {
            OE_DEBUG << "[osgEarth::GeoImage::crop] Performing non-exact crop " << std::endl;
            //If an exact crop is not desired, we can use the faster image cropping code that does no resampling.
            double destXMin = extent.xMin();
            double destYMin = extent.yMin();
            double destXMax = extent.xMax();
            double destYMax = extent.yMax();

            osg::Image* new_image = ImageUtils::cropImage(
                _image.get(),
                _extent.xMin(), _extent.yMin(), _extent.xMax(), _extent.yMax(),
                destXMin, destYMin, destXMax, destYMax );

            //The destination extents may be different than the input extents due to not being able to crop along pixel boundaries.
            return new_image?
                GeoImage( new_image, GeoExtent( getSRS(), destXMin, destYMin, destXMax, destYMax ) ) :
                GeoImage::INVALID;
        }
    }
    else
    {
        //TODO: just reproject the image before cropping
        OE_NOTICE << "[osgEarth::GeoImage::crop] Cropping extent does not have equivalent SpatialReference" << std::endl;
        return GeoImage::INVALID;
    }
}

GeoImage
GeoImage::addTransparentBorder(bool leftBorder, bool rightBorder, bool bottomBorder, bool topBorder)
{
    unsigned int buffer = 1;

    unsigned int newS = _image->s();
    if (leftBorder) newS += buffer;
    if (rightBorder) newS += buffer;

    unsigned int newT = _image->t();
    if (topBorder)    newT += buffer;
    if (bottomBorder) newT += buffer;

    osg::Image* newImage = new osg::Image;
    newImage->allocateImage(newS, newT, _image->r(), _image->getPixelFormat(), _image->getDataType(), _image->getPacking());
    newImage->setInternalTextureFormat(_image->getInternalTextureFormat());
    memset(newImage->data(), 0, newImage->getImageSizeInBytes());
    unsigned startC = leftBorder ? buffer : 0;
    unsigned startR = bottomBorder ? buffer : 0;
    ImageUtils::copyAsSubImage(_image.get(), newImage, startC, startR );

    //double upp = getUnitsPerPixel();
    double uppw = _extent.width() / (double)_image->s();
	double upph = _extent.height() / (double)_image->t();

    double xmin = leftBorder ? _extent.xMin() - buffer * uppw : _extent.xMin();
    double ymin = bottomBorder ? _extent.yMin() - buffer * upph : _extent.yMin();
    double xmax = rightBorder ? _extent.xMax() + buffer * uppw : _extent.xMax();
    double ymax = topBorder ? _extent.yMax() + buffer * upph : _extent.yMax();

    return GeoImage(newImage, GeoExtent(getSRS(), xmin, ymin, xmax, ymax));
}

static osg::Image*
createImageFromDataset(GDALDataset* ds)
{
    // called internally -- GDAL lock not required

    //Allocate the image
    osg::Image *image = new osg::Image;
    image->allocateImage(ds->GetRasterXSize(), ds->GetRasterYSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

    ds->RasterIO(GF_Read, 0, 0, image->s(), image->t(), (void*)image->data(), image->s(), image->t(), GDT_Byte, 4, NULL, 4, 4 * image->s(), 1);
    ds->FlushCache();

    image->flipVertical();

    return image;
}

static GDALDataset*
createMemDS(int width, int height, double minX, double minY, double maxX, double maxY, const std::string &projection)
{
    //Get the MEM driver
    GDALDriver* memDriver = (GDALDriver*)GDALGetDriverByName("MEM");
    if (!memDriver)
    {
        OE_NOTICE << "[osgEarth::GeoData] Could not get MEM driver" << std::endl;
    }

    //Create the in memory dataset.
    GDALDataset* ds = memDriver->Create("", width, height, 4, GDT_Byte, 0);

    //Initialize the color interpretation
    ds->GetRasterBand(1)->SetColorInterpretation(GCI_RedBand);
    ds->GetRasterBand(2)->SetColorInterpretation(GCI_GreenBand);
    ds->GetRasterBand(3)->SetColorInterpretation(GCI_BlueBand);
    ds->GetRasterBand(4)->SetColorInterpretation(GCI_AlphaBand);

    //Initialize the geotransform
    double geotransform[6];
    double x_units_per_pixel = (maxX - minX) / (double)width;
    double y_units_per_pixel = (maxY - minY) / (double)height;
    geotransform[0] = minX;
    geotransform[1] = x_units_per_pixel;
    geotransform[2] = 0;
    geotransform[3] = maxY;
    geotransform[4] = 0;
    geotransform[5] = -y_units_per_pixel;
    ds->SetGeoTransform(geotransform);
    ds->SetProjection(projection.c_str());

    return ds;
}

static GDALDataset*
createDataSetFromImage(const osg::Image* image, double minX, double minY, double maxX, double maxY, const std::string &projection)
{
    //Clone the incoming image
    osg::ref_ptr<osg::Image> clonedImage = new osg::Image(*image);

    //Flip the image
    clonedImage->flipVertical();  

    GDALDataset* srcDS = createMemDS(image->s(), image->t(), minX, minY, maxX, maxY, projection);

    //Write the image data into the memory dataset
    //If the image is already RGBA, just read all 4 bands in one call
    if (image->getPixelFormat() == GL_RGBA)
    {
        srcDS->RasterIO(GF_Write, 0, 0, clonedImage->s(), clonedImage->t(), (void*)clonedImage->data(), clonedImage->s(), clonedImage->t(), GDT_Byte, 4, NULL, 4, 4 * image->s(), 1);
    }
    else if (image->getPixelFormat() == GL_RGB)
    {    
        //OE_NOTICE << "[osgEarth::GeoData] Reprojecting RGB " << std::endl;
        //Read the read, green and blue bands
        srcDS->RasterIO(GF_Write, 0, 0, clonedImage->s(), clonedImage->t(), (void*)clonedImage->data(), clonedImage->s(), clonedImage->t(), GDT_Byte, 3, NULL, 3, 3 * image->s(), 1);

        //Initialize the alpha values to 255.
        unsigned char *alpha = new unsigned char[clonedImage->s() * clonedImage->t()];
        memset(alpha, 255, clonedImage->s() * clonedImage->t());

        GDALRasterBand* alphaBand = srcDS->GetRasterBand(4);
        alphaBand->RasterIO(GF_Write, 0, 0, clonedImage->s(), clonedImage->t(), alpha, clonedImage->s(),clonedImage->t(), GDT_Byte, 0, 0);

        delete[] alpha;
    }
    else
    {
        OE_WARN << LC << "createDataSetFromImage: unsupported pixel format " << std::hex << image->getPixelFormat() << std::endl;
    }
    srcDS->FlushCache();

    return srcDS;
}

static osg::Image*
reprojectImage(osg::Image* srcImage, const std::string srcWKT, double srcMinX, double srcMinY, double srcMaxX, double srcMaxY,
               const std::string destWKT, double destMinX, double destMinY, double destMaxX, double destMaxY,
               int width = 0, int height = 0)
{
    //OE_NOTICE << "Reprojecting..." << std::endl;
    GDAL_SCOPED_LOCK;
	osg::Timer_t start = osg::Timer::instance()->tick();

    //Create a dataset from the source image
    GDALDataset* srcDS = createDataSetFromImage(srcImage, srcMinX, srcMinY, srcMaxX, srcMaxY, srcWKT);

	OE_DEBUG << "Source image is " << srcImage->s() << "x" << srcImage->t() << std::endl;


    if (width == 0 || height == 0)
    {
        double outgeotransform[6];
        double extents[4];
        void* transformer = GDALCreateGenImgProjTransformer(srcDS, srcWKT.c_str(), NULL, destWKT.c_str(), 1, 0, 0);
        GDALSuggestedWarpOutput2(srcDS,
            GDALGenImgProjTransform, transformer,
            outgeotransform,
            &width,
            &height,
            extents,
            0);
        GDALDestroyGenImgProjTransformer(transformer);
    }
	OE_DEBUG << "Creating warped output of " << width <<"x" << height << std::endl;
   
    GDALDataset* destDS = createMemDS(width, height, destMinX, destMinY, destMaxX, destMaxY, destWKT);

    GDALReprojectImage(srcDS, NULL,
                       destDS, NULL,
                       //GDALResampleAlg::GRA_NearestNeighbour,
                       GRA_Bilinear,
                       0,0,0,0,0);                    

    osg::Image* result = createImageFromDataset(destDS);
    
    delete srcDS;
    delete destDS;  

	osg::Timer_t end = osg::Timer::instance()->tick();

	OE_DEBUG << "Reprojected image in " << osg::Timer::instance()->delta_m(start,end) << std::endl;

    return result;
}    

static osg::Image*
manualReproject(const osg::Image* image, const GeoExtent& src_extent, const GeoExtent& dest_extent,
                unsigned int width = 0, unsigned int height = 0)
{
    //TODO:  Compute the optimal destination size
    if (width == 0 || height == 0)
    {
        //If no width and height are specified, just use the minimum dimension for the image
        width = osg::minimum(image->s(), image->t());
        height = osg::minimum(image->s(), image->t());        
    }

    // need to know this in order to choose the right interpolation algorithm
    const bool isSrcContiguous = src_extent.getSRS()->isContiguous();

    osg::Image *result = new osg::Image();
    result->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    //Initialize the image to be completely transparent
    memset(result->data(), 0, result->getImageSizeInBytes());

    ImageUtils::PixelReader ra(result);
    const double dx = dest_extent.width() / (double)width;
    const double dy = dest_extent.height() / (double)height;

    // offset the sample points by 1/2 a pixel so we are sampling "pixel center".
    // (This is especially useful in the UnifiedCubeProfile since it nullifes the chances for
    // edge ambiguity.)

    unsigned int numPixels = width * height;

    // Start by creating a sample grid over the destination
    // extent. These will be the source coordinates. Then, reproject
    // the sample grid into the source coordinate system.
    double *srcPointsX = new double[numPixels * 2];
    double *srcPointsY = srcPointsX + numPixels;
    dest_extent.getSRS()->transformExtentPoints(
        src_extent.getSRS(),
        dest_extent.xMin() + .5 * dx, dest_extent.yMin() + .5 * dy,
        dest_extent.xMax() - .5 * dx, dest_extent.yMax() - .5 * dy,
        srcPointsX, srcPointsY, width, height, 0, true);

    // Next, go through the source-SRS sample grid, read the color at each point from the source image,
    // and write it to the corresponding pixel in the destination image.
    int pixel = 0;
    ImageUtils::PixelReader ia(image);
    double xfac = (image->s() - 1) / src_extent.width();
    double yfac = (image->t() - 1) / src_extent.height();
    for (unsigned int c = 0; c < width; ++c)
    {
        for (unsigned int r = 0; r < height; ++r)
        {   
            double src_x = srcPointsX[pixel];
            double src_y = srcPointsY[pixel];

            if ( src_x < src_extent.xMin() || src_x > src_extent.xMax() || src_y < src_extent.yMin() || src_y > src_extent.yMax() )
            {
                //If the sample point is outside of the bound of the source extent, increment the pixel and keep looping through.
                //OE_WARN << LC << "ERROR: sample point out of bounds: " << src_x << ", " << src_y << std::endl;
                pixel++;
                continue;
            }

            float px = (src_x - src_extent.xMin()) * xfac;
            float py = (src_y - src_extent.yMin()) * yfac;

            int px_i = osg::clampBetween( (int)osg::round(px), 0, image->s()-1 );
            int py_i = osg::clampBetween( (int)osg::round(py), 0, image->t()-1 );

            osg::Vec4 color(0,0,0,0);

            if ( ! isSrcContiguous ) // non-contiguous space- use nearest neighbot
            {
                color = ia(px_i, py_i);
            }

            else // contiguous space - use bilinear sampling
            {
                int rowMin = osg::maximum((int)floor(py), 0);
                int rowMax = osg::maximum(osg::minimum((int)ceil(py), (int)(image->t()-1)), 0);
                int colMin = osg::maximum((int)floor(px), 0);
                int colMax = osg::maximum(osg::minimum((int)ceil(px), (int)(image->s()-1)), 0);

                if (rowMin > rowMax) rowMin = rowMax;
                if (colMin > colMax) colMin = colMax;

                osg::Vec4 urColor = ia(colMax, rowMax);
                osg::Vec4 llColor = ia(colMin, rowMin);
                osg::Vec4 ulColor = ia(colMin, rowMax);
                osg::Vec4 lrColor = ia(colMax, rowMin);

                /*Average Interpolation*/
                /*double x_rem = px - (int)px;
                double y_rem = py - (int)py;

                double w00 = (1.0 - y_rem) * (1.0 - x_rem);
                double w01 = (1.0 - y_rem) * x_rem;
                double w10 = y_rem * (1.0 - x_rem);
                double w11 = y_rem * x_rem;
                double wsum = w00 + w01 + w10 + w11;
                wsum = 1.0/wsum;

                color.r() = (w00 * llColor.r() + w01 * lrColor.r() + w10 * ulColor.r() + w11 * urColor.r()) * wsum;
                color.g() = (w00 * llColor.g() + w01 * lrColor.g() + w10 * ulColor.g() + w11 * urColor.g()) * wsum;
                color.b() = (w00 * llColor.b() + w01 * lrColor.b() + w10 * ulColor.b() + w11 * urColor.b()) * wsum;
                color.a() = (w00 * llColor.a() + w01 * lrColor.a() + w10 * ulColor.a() + w11 * urColor.a()) * wsum;*/

                /*Nearest Neighbor Interpolation*/
                /*if (px_i >= 0 && px_i < image->s() &&
                py_i >= 0 && py_i < image->t())
                {
                //OE_NOTICE << "[osgEarth::GeoData] Sampling pixel " << px << "," << py << std::endl;
                color = ImageUtils::getColor(image, px_i, py_i);
                }
                else
                {
                OE_NOTICE << "[osgEarth::GeoData] Pixel out of range " << px_i << "," << py_i << "  image is " << image->s() << "x" << image->t() << std::endl;
                }*/

                /*Bilinear interpolation*/
                //Check for exact value
                if ((colMax == colMin) && (rowMax == rowMin))
                {
                    //OE_NOTICE << "[osgEarth::GeoData] Exact value" << std::endl;
                    color = ia(px_i, py_i);
                }
                else if (colMax == colMin)
                {
                    //OE_NOTICE << "[osgEarth::GeoData] Vertically" << std::endl;
                    //Linear interpolate vertically
                    for (unsigned int i = 0; i < 4; ++i)
                    {
                        color[i] = ((float)rowMax - py) * llColor[i] + (py - (float)rowMin) * ulColor[i];
                    }
                }
                else if (rowMax == rowMin)
                {
                    //OE_NOTICE << "[osgEarth::GeoData] Horizontally" << std::endl;
                    //Linear interpolate horizontally
                    for (unsigned int i = 0; i < 4; ++i)
                    {
                        color[i] = ((float)colMax - px) * llColor[i] + (px - (float)colMin) * lrColor[i];
                    }
                }
                else
                {
                    //OE_NOTICE << "[osgEarth::GeoData] Bilinear" << std::endl;
                    //Bilinear interpolate
                    float col1 = colMax - px, col2 = px - colMin;
                    float row1 = rowMax - py, row2 = py - rowMin;
                    for (unsigned int i = 0; i < 4; ++i)
                    {
                        float r1 = col1 * llColor[i] + col2 * lrColor[i];
                        float r2 = col1 * ulColor[i] + col2 * urColor[i];

                        //OE_INFO << "r1, r2 = " << r1 << " , " << r2 << std::endl;
                        color[i] = row1 * r1 + row2 * r2;
                    }
                }
            }
               
            unsigned char* rgba = const_cast<unsigned char*>(ra.data(c,r,0));

            rgba[0] = (unsigned char)(color.r() * 255);
            rgba[1] = (unsigned char)(color.g() * 255);
            rgba[2] = (unsigned char)(color.b() * 255);
            rgba[3] = (unsigned char)(color.a() * 255);

            pixel++;            
        }
    }

    delete[] srcPointsX;

    return result;
}



GeoImage
GeoImage::reproject(const SpatialReference* to_srs, const GeoExtent* to_extent, unsigned int width, unsigned int height) const
{  
    GeoExtent destExtent;
    if (to_extent)
    {
        destExtent = *to_extent;
    }
    else
    {
         destExtent = getExtent().transform(to_srs);    
    }

    osg::Image* resultImage = 0L;

    if ( getSRS()->isUserDefined() || to_srs->isUserDefined() ||
        ( getSRS()->isMercator() && to_srs->isGeographic() ) ||
        ( getSRS()->isGeographic() && to_srs->isMercator() ) )
    {
        // if either of the SRS is a custom projection, we have to do a manual reprojection since
        // GDAL will not recognize the SRS.
        resultImage = manualReproject(getImage(), getExtent(), *to_extent, width, height);
    }
    else
    {
        // otherwise use GDAL.
        resultImage = reprojectImage(getImage(),
            getSRS()->getWKT(),
            getExtent().xMin(), getExtent().yMin(), getExtent().xMax(), getExtent().yMax(),
            to_srs->getWKT(),
            destExtent.xMin(), destExtent.yMin(), destExtent.xMax(), destExtent.yMax(),
            width, height);
    }   
    return GeoImage(resultImage, destExtent);
}

osg::Image*
GeoImage::takeImage()
{
    return _image.release();
}


/***************************************************************************/

// static
GeoHeightField GeoHeightField::INVALID( 0L, GeoExtent::INVALID, 0L );

GeoHeightField::GeoHeightField() :
_heightField( 0L ),
_extent( GeoExtent::INVALID ),
_vsrs( 0L )
{
    //nop
}

GeoHeightField::GeoHeightField(osg::HeightField* heightField,
                               const GeoExtent& extent,
                               const VerticalSpatialReference* vsrs) :
_heightField( heightField ),
_extent( extent ),
_vsrs( vsrs )
{
    if ( _heightField )
    {
        double minx, miny, maxx, maxy;
        _extent.getBounds(minx, miny, maxx, maxy);

        _heightField->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
        _heightField->setXInterval( (maxx - minx)/(double)(_heightField->getNumColumns()-1) );
        _heightField->setYInterval( (maxy - miny)/(double)(_heightField->getNumRows()-1) );
        _heightField->setBorderWidth( 0 );
    }
}

bool
GeoHeightField::getElevation(const osgEarth::SpatialReference* inputSRS, 
                             double x, double y, 
                             ElevationInterpolation interp,
                             const VerticalSpatialReference* outputVSRS,
                             float &elevation) const
{
    double local_x = x, local_y = y;

    if ( inputSRS && !inputSRS->transform(x, y, _extent.getSRS(), local_x, local_y) )
        return false;

    if ( _extent.contains(local_x, local_y) )
    {
        double xInterval = _extent.width()  / (double)(_heightField->getNumColumns()-1);
        double yInterval = _extent.height() / (double)(_heightField->getNumRows()-1);

        elevation = HeightFieldUtils::getHeightAtLocation(
            _heightField.get(), 
            local_x, local_y, 
            _extent.xMin(), _extent.yMin(), 
            xInterval, yInterval, 
            interp);

        if ( elevation != NO_DATA_VALUE )
        {
            if ( VerticalSpatialReference::canTransform( _vsrs.get(), outputVSRS ) )
            {
                // need geodetic coordinates for a VSRS transformation:
                double lat_deg, lon_deg, newElevation;

                if ( inputSRS->isGeographic() ) {
                    lat_deg = y;
                    lon_deg = x;
                }
                else if ( _extent.getSRS()->isGeographic() ) {
                    lat_deg = local_y;
                    lon_deg = local_x;
                }
                else {
                    _extent.getSRS()->transform( x, y, inputSRS->getGeographicSRS(), lon_deg, lat_deg );
                }

                if ( _vsrs->transform( outputVSRS, lat_deg, lon_deg, elevation, newElevation ) )
                    elevation = newElevation;
            }
        }

        return true;
    }
    else
    {
        elevation = 0.0f;
        return false;
    }
}

GeoHeightField
GeoHeightField::createSubSample( const GeoExtent& destEx, ElevationInterpolation interpolation) const
{
    double div = destEx.width()/_extent.width();
    if ( div >= 1.0f )
        return GeoHeightField::INVALID;

    int w = _heightField->getNumColumns();
    int h = _heightField->getNumRows();
    //double dx = _heightField->getXInterval() * div;
    //double dy = _heightField->getYInterval() * div;
    double xInterval = _extent.width() / (double)(_heightField->getNumColumns()-1);
    double yInterval = _extent.height() / (double)(_heightField->getNumRows()-1);
    double dx = xInterval * div;
    double dy = yInterval * div;

    osg::HeightField* dest = new osg::HeightField();
    dest->allocate( w, h );
    dest->setXInterval( dx );
    dest->setYInterval( dy );

    // copy over the skirt height, adjusting it for tile size.
    dest->setSkirtHeight( _heightField->getSkirtHeight() * div );

    double x, y;
    int col, row;

    for( x = destEx.xMin(), col=0; col < w; x += dx, col++ )
    {
        for( y = destEx.yMin(), row=0; row < h; y += dy, row++ )
        {
            float height = HeightFieldUtils::getHeightAtLocation( _heightField.get(), x, y, _extent.xMin(), _extent.yMin(), xInterval, yInterval, interpolation);
            dest->setHeight( col, row, height );
        }
    }

    osg::Vec3d orig( destEx.xMin(), destEx.yMin(), _heightField->getOrigin().z() );
    dest->setOrigin( orig );

    return GeoHeightField( dest, destEx, _vsrs.get() );
}

const GeoExtent&
GeoHeightField::getExtent() const
{
    return _extent;
}

const osg::HeightField*
GeoHeightField::getHeightField() const
{
    return _heightField.get();
}

osg::HeightField*
GeoHeightField::getHeightField() 
{
    return _heightField.get();
}

osg::HeightField*
GeoHeightField::takeHeightField()
{
    return _heightField.release();
}

// --------------------------------------------------------------------------

#undef  LC
#define LC "[Geoid] "

Geoid::Geoid() :
_hf( GeoHeightField::INVALID ),
_units( Units::METERS ),
_valid( false )
{
    //nop
}

void
Geoid::setName( const std::string& name )
{
    _name = name;
    validate();
}

void
Geoid::setHeightField( const GeoHeightField& hf )
{
    _hf = hf;
    validate();
}

void
Geoid::setUnits( const Units& units ) 
{
    _units = units;
    validate();
}

void
Geoid::validate()
{
    _valid = false;
    if ( !_hf.valid() ) {
        //OE_WARN << LC << "ILLEGAL GEOID: no heightfield" << std::endl;
    }
    else if ( !_hf.getExtent().getSRS() || !_hf.getExtent().getSRS()->isGeographic() ) {
        OE_WARN << LC << "ILLEGAL GEOID: heightfield must be geodetic" << std::endl;
    }
    else {
        _valid = true;
    }
}

float 
Geoid::getOffset(double lat_deg, double lon_deg, const ElevationInterpolation& interp ) const
{
    float result = 0.0f;

    if ( _valid )
    {
        // first convert the query coordinates to the geoid heightfield range if neccesary.
        if ( lat_deg < _hf.getExtent().yMin() )
            lat_deg = 90.0 - (-90.0-lat_deg);
        else if ( lat_deg > _hf.getExtent().yMax() )
            lat_deg = -90 + (lat_deg-90.0);
        if ( lon_deg < _hf.getExtent().xMin() )
            lon_deg += 360.0;
        else if ( lon_deg > _hf.getExtent().xMax() )
            lon_deg -= 360.0;

        bool ok = _hf.getElevation( 0L, lon_deg, lat_deg, interp, 0L, result );
        if ( !ok )
            result = 0.0f;
    }

    return result;
}

bool
Geoid::isEquivalentTo( const Geoid& rhs ) const
{
    // weak..
    return
        _valid &&
        _name == rhs._name &&
        _hf.getExtent() == rhs._hf.getExtent() &&
        _units == rhs._units;
}
