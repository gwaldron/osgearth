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
#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>


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
    return _srs.valid() &&
           _xmin < _xmax &&
           _ymin < _ymax;
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

osg::Image* createImageFromDataset(GDALDataset* ds)
{
    //Allocate the image
    osg::Image *image = new osg::Image;
    image->allocateImage(ds->GetRasterXSize(), ds->GetRasterYSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

    ds->RasterIO(GF_Read, 0, 0, image->s(), image->t(), (void*)image->data(), image->s(), image->t(), GDT_Byte, 4, NULL, 4, 4 * image->s(), 1);
    ds->FlushCache();

    image->flipVertical();

    return image;
}

GDALDataset* createMemDS(int width, int height, double minX, double minY, double maxX, double maxY, const std::string &projection)
{
    //Get the MEM driver
    GDALDriver* memDriver = (GDALDriver*)GDALGetDriverByName("MEM");
    if (!memDriver)
    {
        osg::notify(osg::NOTICE) << "Could not get MEM driver" << std::endl;
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

GDALDataset* createDataSetFromImage(const osg::Image* image, double minX, double minY, double maxX, double maxY, const std::string &projection)
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
        //osg::notify(osg::NOTICE) << "Reprojecting RGB " << std::endl;
        //Read the read, green and blue bands
        srcDS->RasterIO(GF_Write, 0, 0, clonedImage->s(), clonedImage->t(), (void*)clonedImage->data(), clonedImage->s(), clonedImage->t(), GDT_Byte, 3, NULL, 3, 3 * image->s(), 1);

        //Initialize the alpha values to 255.
        unsigned char *alpha = new unsigned char[clonedImage->s() * clonedImage->t()];
        memset(alpha, 255, clonedImage->s() * clonedImage->t());

        GDALRasterBand* alphaBand = srcDS->GetRasterBand(4);
        alphaBand->RasterIO(GF_Write, 0, 0, clonedImage->s(), clonedImage->t(), alpha, clonedImage->s(),clonedImage->t(), GDT_Byte, 0, 0);

        delete[] alpha;
    }
    srcDS->FlushCache();

    return srcDS;
}

osg::Image* reprojectImage(osg::Image* srcImage, const std::string srcWKT, double srcMinX, double srcMinY, double srcMaxX, double srcMaxY,
                           const std::string destWKT, double destMinX, double destMinY, double destMaxX, double destMaxY)
{
    //Create a dataset from the source image
    GDALDataset* srcDS = createDataSetFromImage(srcImage, srcMinX, srcMinY, srcMaxX, srcMaxY, srcWKT);


    void* transformer = GDALCreateGenImgProjTransformer(srcDS, NULL, NULL, destWKT.c_str(), 1, 0, 0);

    double outgeotransform[6];
    double extents[4];
    int width,height;
    GDALSuggestedWarpOutput2(srcDS,
                             GDALGenImgProjTransform, transformer,
                             outgeotransform,
                             &width,
                             &height,
                             extents,
                             0);

   
    GDALDataset* destDS = createMemDS(width, height, destMinX, destMinY, destMaxX, destMaxY, destWKT);

    GDALReprojectImage(srcDS, NULL,
                       destDS, NULL,
                       //GDALResampleAlg::GRA_NearestNeighbour,
                       GRA_Bilinear,
                       0,0,0,0,0);                    

    osg::Image* result = createImageFromDataset(destDS);
    
    delete srcDS;
    delete destDS;
    
    GDALDestroyGenImgProjTransformer(transformer);

    return result;
}    



GeoImage*
GeoImage::reproject(const SpatialReference* to_srs, const GeoExtent* to_extent) const
{
    GDALAllRegister();
    
    GeoExtent destExtent;
    if (to_extent)
    {
        destExtent = *to_extent;
    }
    else
    {
         destExtent = getExtent().transform(to_srs);    
    }
   
    osg::Image* resultImage = reprojectImage(getImage(),
                                       getSRS()->getWKT(),
                                       getExtent().xMin(), getExtent().yMin(), getExtent().xMax(), getExtent().yMax(),
                                       to_srs->getWKT(),
                                       destExtent.xMin(), destExtent.yMin(), destExtent.xMax(), destExtent.yMax());
    return new GeoImage(resultImage, destExtent);
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