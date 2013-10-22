/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgDB/FileNameUtils>

#include <cstring>

#define LC "[OSG Driver] "

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
osg::Image* makeRGBAandComputeAlpha(osg::Image* image)
{
    osg::Image* result = new osg::Image();
    result->allocateImage( image->s(), image->t(), image->r(), GL_RGBA, GL_UNSIGNED_BYTE );
    result->setInternalTextureFormat( GL_RGBA8 );
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

    Status initialize( const osgDB::Options* dbOptions )
    {
        osg::ref_ptr<osgDB::Options> localOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
        CachePolicy::NO_CACHE.apply(localOptions.get());

        if ( !getProfile() )
        {
            return Status::Error( "An explicit profile definition is required by the OSG driver." );
        }

        osg::ref_ptr<osg::Image> image;

        if ( !_options.url()->empty() )
        {
            ReadResult r = _options.url()->readImage( localOptions.get() );
            if ( r.succeeded() )
            {
                image = r.getImage();
            }
        }

        if ( !image.valid() )
        {
            return Status::Error( Stringify() <<  "Faild to load data from \"" << _options.url()->full() << "\"" );
        }

        // calculate and store the maximum LOD for which to return data
        if ( image.valid() )
        {
            if ( _options.maxDataLevel().isSet() )
            {
                _maxDataLevel = *_options.maxDataLevel();
            }
            else
            {
                int minSpan = osg::minimum( image->s(), image->t() );
                int tileSize = _options.tileSize().value();
                _maxDataLevel = (int)LOG2((minSpan/tileSize)+1);
                //OE_NOTICE << "[osgEarth::OSG driver] minSpan=" << minSpan << ", _tileSize=" << tileSize << ", maxDataLevel = " << _maxDataLevel << std::endl;
            }
            
            getDataExtents().push_back( DataExtent(getProfile()->getExtent(), 0, _maxDataLevel) );

            bool computeAlpha =
                (_options.convertLuminanceToRGBA() == true && image->getPixelFormat() == GL_LUMINANCE) ||
                (_options.addAlpha() == true && !ImageUtils::hasAlphaChannel( image.get() ) );

            if ( computeAlpha )
            {
                image = makeRGBAandComputeAlpha( image.get() );
            }
            else if ( ImageUtils::hasAlphaChannel( image.get() ))
            {
                image = ImageUtils::convertToRGBA8( image.get() );
            }
            else
            {
                image = ImageUtils::convertToRGB8( image.get() );
            }

            _image = GeoImage( image.get(), getProfile()->getExtent() );
        }

        _extension = osgDB::getFileExtension( _options.url()->full() );

        return STATUS_OK;
    }
    
    //override
    unsigned int getMaxDataLevel() const 
    {
        return _maxDataLevel;
    }

    osg::Image*
    createImage( const TileKey& key, ProgressCallback* progress )
    {
        if ( !_image.valid() || key.getLevelOfDetail() > getMaxDataLevel() )
            return NULL;

        GeoImage cropped = _image.crop( key.getExtent(), true, getPixelsPerTile(), getPixelsPerTile(), *_options.bilinearReprojection() );
        return cropped.valid() ? cropped.takeImage() : 0L;
    }

    std::string
    getExtension() const 
    {
        return _extension;
    }

private:
    std::string      _extension;
    int              _maxDataLevel;
    GeoImage         _image;
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

