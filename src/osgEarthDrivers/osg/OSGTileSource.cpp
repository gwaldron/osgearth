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
#include "OSGOptions"

#include <osgEarth/HTTPClient>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgDB/FileNameUtils>

#include <cstring>

#define LOG2(X) (::log((double)(X))/::log(2.0))

using namespace osgEarth;
using namespace osgEarth::Drivers;

struct CopyAndSetAlpha
{
    bool operator()( const osg::Vec4& in, osg::Vec4& out ) {
        out = in;
        out.a() = 0.3333*(in.r() + in.g() + in.b());
        return true;
    }
};

static
osg::Image* makeRGBA(osg::Image* image)
{
    osg::Image* result = new osg::Image();
    result->allocateImage( image->s(), image->t(), image->r(), GL_RGBA, GL_UNSIGNED_BYTE );
    ImageUtils::PixelVisitor<CopyAndSetAlpha>().accept( image, result );
    return result;
}

class OSGTileSource : public TileSource
{
public:
    OSGTileSource( const TileSourceOptions& options ) :
      TileSource( options ),      
      _maxDataLevel( 21 ),
      _options( options )
    {
        //nop
    }

    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {
        setProfile( overrideProfile );

        _url = _options.url().value();
        if ( !_url.empty() )
        {
            _url = osgEarth::getFullPath( referenceURI, _url );
            HTTPClient::readImageFile( _url, _image ); //, getOptions() );
        }

        if ( !_image.valid() )
            OE_WARN << "[osgEarth::OSG driver] Cannot load data from [" << _url << "]" << std::endl;

        // calculate and store the maximum LOD for which to return data
        if ( _image.valid() )
        {
            int minSpan = osg::minimum( _image->s(), _image->t() );
            int tileSize = _options.tileSize().value();
            _maxDataLevel = LOG2((minSpan/tileSize)+1);
            //OE_NOTICE << "[osgEarth::OSG driver] minSpan=" << minSpan << ", _tileSize=" << tileSize << ", maxDataLevel = " << _maxDataLevel << std::endl;

            if ( _options.convertLuminanceToRGBA() == true && _image->getPixelFormat() == GL_LUMINANCE )
            {
                _image = makeRGBA( _image.get() );
            }
            else if ( _options.addAlpha() == true && !ImageUtils::hasAlphaChannel( _image.get() ) )
            {
                _image = makeRGBA( _image.get() );
            }
        }
    }
    
    //override
    unsigned int getMaxDataLevel() const 
    {
        return _maxDataLevel;
    }

    osg::Image*
    createImage( const TileKey& key, ProgressCallback* progress )
    {
        if ( !_image.valid() || !getProfile() || key.getLevelOfDetail() > getMaxDataLevel() )
            return NULL;

        const GeoExtent& imageEx = getProfile()->getExtent();
        const GeoExtent& keyEx = key.getExtent();

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
        newImage->setInternalTextureFormat( _image->getInternalTextureFormat());
        for( int row=crop_y; row<crop_y+crop_t; row++ )
            memcpy( newImage->data(0, row-crop_y), _image->data(crop_x, row), crop_s * _image->getPixelSizeInBits() / 8 );

        return newImage;
    }

    std::string
    getExtension() const 
    {
        return osgDB::getFileExtension( _url );
    }

private:
    std::string _url;
    int _maxDataLevel;
    osg::ref_ptr<osg::Image> _image;
    const OSGOptions _options;
};


/**
 * This driver defers loading of the source data to the appropriate OSG plugin. You
 * must explicity set an override profile when using this driver.
 *
 * For example, use this driver to load a simple jpeg file; then set the profile to
 * tell osgEarth its projection.
 */
class OSGTileSourceFactory : public TileSourceDriver
{
public:
    OSGTileSourceFactory()
    {
        supportsExtension( "osgearth_osg", "OSG image driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "OSG Image Driver";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new OSGTileSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_osg, OSGTileSourceFactory)

