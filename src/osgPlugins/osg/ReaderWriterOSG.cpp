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

#include <osgEarth/Registry>
#include <osgEarth/TileSource>

#include <osg/Notify>
#include <osg/io_utils>
#include <osg/Version>
#include <osgTerrain/Terrain>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>

#include <stdio.h>
#include <string.h>

#define LOG2(X) (::log((double)(X))/::log(2.0))

using namespace osgEarth;

#define PROPERTY_URL "url"
#define PROPERTY_LUMINANCE_TO_RGBA "luminance_to_rgba"

static
osg::Image* makeRGBA(osg::Image* image)
{
    osg::Image* result = new osg::Image;
    result->allocateImage(image->s(), image->t(), image->r(), GL_RGBA, GL_UNSIGNED_BYTE);

    if (image->getPixelFormat() == GL_LUMINANCE)
    {
        for (int r = 0; r < image->t(); ++r)
        {
            for (int c = 0; c < image->s(); ++c)
            {
                unsigned char val = *image->data(c, r);
                result->data(c,r)[0] = val;
                result->data(c,r)[1] = val;
                result->data(c,r)[2] = val;
                result->data(c,r)[3] = val;
            }
        }
    }

    return result;
}

/**
 * A TileSource that uses one of the stock OSG plugins to load an image file.
 */
class OSGSource : public TileSource
{
public:
    OSGSource( const osgDB::ReaderWriter::Options* options ) : TileSource( options ), _tileSize( 256 )
    {
        if ( options->getPluginData( PROPERTY_URL ) )
            _url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

        if ( options->getPluginData( PROPERTY_LUMINANCE_TO_RGBA ) )
            _convertLuminanceToRGBA = true;            

        if ( !_url.empty() )
            _image = osgDB::readImageFile( _url, options );

        if ( !_image.valid() )
            osg::notify(osg::WARN) << "[osgEarth::OSG driver] Cannot load data from [" << _url << "]" << std::endl;

        // calculate and store the maximum LOD for which to return data
        if ( _image.valid() )
        {
            int minSpan = osg::minimum( _image->s(), _image->t() );
            _maxDataLevel = LOG2(minSpan/_tileSize);

            if ( _convertLuminanceToRGBA && _image->getPixelFormat() == GL_LUMINANCE )
            {
                _image = makeRGBA( _image.get() );
            }
        }
    }

    // Return NULL - this driver REQUIRES that you set an override profile.
    virtual void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
        setProfile( overrideProfile );
    }
    
    // override: return the max LOD for which data exists
    unsigned int getMaxDataLevel() const
    {
        return _maxDataLevel;
    }

    // override: create an image given a tile key
    osg::Image* createImage( const TileKey* key )
    {
        if ( !_image.valid() || !getProfile() || key->getLevelOfDetail() > getMaxDataLevel() )
            return NULL;

        const GeoExtent& imageEx = getProfile()->getExtent();
        const GeoExtent& keyEx = key->getGeoExtent();

        double x0r = (keyEx.xMin()-imageEx.xMin())/imageEx.width();
        double x1r = (keyEx.xMax()-imageEx.xMin())/imageEx.width();
        double y0r = (keyEx.yMin()-imageEx.yMin())/imageEx.height();
        double y1r = (keyEx.yMax()-imageEx.yMin())/imageEx.height();

        // first crop out the image part we want:
        int crop_x = (int)( x0r*(float)_image->s() );
        int crop_y = (int)( y0r*(float)_image->t() );
        int crop_s = (int)( (x1r-x0r)*(float)_image->s() );
        int crop_t = (int)( (y1r-y0r)*(float)_image->t() );

        osg::Image* newImage = new osg::Image();
        newImage->setAllocationMode( osg::Image::USE_NEW_DELETE );
        newImage->allocateImage( crop_s, crop_t, 1, _image->getPixelFormat(), _image->getDataType(), _image->getPacking() );
        for( int row=crop_y; row<crop_y+crop_t; row++ )
            memcpy( newImage->data(0, row-crop_y), _image->data(crop_x, row), crop_s * _image->getPixelSizeInBits() / 8 );

        return newImage;
    }

    osg::HeightField* createHeightField( const TileKey* key )
    {
        return NULL;
    }

    virtual std::string getExtension()  const 
    {
        return osgDB::getFileExtension( _url );
    }


private:
    std::string _url;
    int _tileSize;
    int _maxDataLevel;
    bool _convertLuminanceToRGBA;
    osg::ref_ptr<osg::Image> _image;
};


/**
 * This driver defers loading of the source data to the appropriate OSG plugin. You
 * must explicity set an override profile when using this driver.
 *
 * For example, use this driver to load a simple jpeg file; then set the profile to
 * tell osgEarth its projection.
 */
class ReaderWriterOSG : public osgDB::ReaderWriter
{
public:
    ReaderWriterOSG()
    {
        supportsExtension( "osgearth_osg", "OSG driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "OSG ReaderWriter";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new OSGSource( options );
    }
};

REGISTER_OSGPLUGIN(osgearth_osg, ReaderWriterOSG)

