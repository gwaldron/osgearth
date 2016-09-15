/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "TMSTileSource"
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::TileMapService;

#define LC "[TMSTileSource] "


TMSTileSource::TMSTileSource(const TileSourceOptions& options) :
TileSource(options),
_options  (options),
_forceRGB (false)
{
    _invertY = _options.tmsType() == "google";
}


Status
TMSTileSource::initialize(const osgDB::Options* dbOptions)
{
    // local copy of options we can modify if necessary.
    _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);

    // see if the use passed in a profile
    const Profile* profile = getProfile();

    // URI is mandatory.
    URI tmsURI = _options.url().value();
    if ( tmsURI.empty() )
    {
        return Status::Error( Status::ConfigurationError, "Fail: TMS driver requires a valid \"url\" property" );
    }

    // A repo is writable only if it's local.
    if ( tmsURI.isRemote() )
    {
        OE_DEBUG << LC << "Repo is remote; opening in read-only mode" << std::endl;
    }

    // Is this a new repo? (You can only create a new repo at a local non-archive URI.)
    bool isNewRepo = false;

    if (!tmsURI.isRemote() && 
        !osgEarth::isPathToArchivedFile(tmsURI.full()) &&
        !osgDB::fileExists(tmsURI.full()) )
    {
        isNewRepo = true;

        // new repo REQUIRES a profile:
        if ( !profile )
        {
            return Status::Error(Status::ConfigurationError, "Fail: profile required to create new TMS repo");
        }
    }

    // Take the override profile if one is given
    if ( profile )
    {
        OE_INFO << LC 
            << "Using express profile \"" << getProfile()->toString() 
            << "\" for URI \"" << tmsURI.base() << "\"" 
            << std::endl;

        DataExtentList extents;

        _tileMap = TMS::TileMap::create( 
            _options.url()->full(),
            profile,
            extents,
            _options.format().value(),
            _options.tileSize().value(), 
            _options.tileSize().value() );

        // If this is a new repo, write the tilemap file to disk now.
        if ( isNewRepo )
        {
            if ( !_options.format().isSet() )
            {
                return Status::Error(Status::ConfigurationError, "Cannot create new repo with required [format] property");
            }

            TMS::TileMapReaderWriter::write( _tileMap.get(), tmsURI.full() );
            OE_INFO << LC << "Created new TMS repo at " << tmsURI.full() << std::endl;
        }
    }

    else
    {
        // Attempt to read the tile map parameters from a TMS TileMap XML tile on the server:
        _tileMap = TMS::TileMapReaderWriter::read( tmsURI.full(), _dbOptions.get() );

        if (!_tileMap.valid())
        {
            return Status::Error( Status::ResourceUnavailable, Stringify() << "Failed to read tilemap from " << tmsURI.full() );
        }

        OE_INFO << LC
            << "TMS tile map datestamp = "
            << DateTime(_tileMap->getTimeStamp()).asRFC1123()
            << std::endl;

        profile = _tileMap->createProfile();
        if ( profile )
            setProfile( profile );
    }

    // Make sure we've established a profile by this point:
    if ( !profile )
    {
        return Status::Error( Stringify() << "Failed to establish a profile for " << tmsURI.full() );
    }

    // resolve the writer
    if ( !tmsURI.isRemote() && !resolveWriter() )
    {
        OE_WARN << LC << "Cannot create writer; writing disabled" << std::endl;
    }

    // TileMap and profile are valid at this point. Build the tile sets.
    // Automatically set the min and max level of the TileMap
    if ( _tileMap->getTileSets().size() > 0 )
    {
        OE_DEBUG << LC << "TileMap min/max " << _tileMap->getMinLevel() << ", " << _tileMap->getMaxLevel() << std::endl;
        if (_tileMap->getDataExtents().size() > 0)
        {
            for (DataExtentList::iterator itr = _tileMap->getDataExtents().begin(); itr != _tileMap->getDataExtents().end(); ++itr)
            {
                this->getDataExtents().push_back(*itr);
            }
        }
        else
        {
            //Push back a single area that encompasses the whole profile going up to the max level
            this->getDataExtents().push_back(DataExtent(profile->getExtent(), 0, _tileMap->getMaxLevel()));
        }
    }
 
    return STATUS_OK;
}


TimeStamp
TMSTileSource::getLastModifiedTime() const
{
    if ( _tileMap.valid() )
        return _tileMap->getTimeStamp();
    else
        return TileSource::getLastModifiedTime();
}


CachePolicy
TMSTileSource::getCachePolicyHint(const Profile* targetProfile) const
{
    // if the source is local and the profiles line up, don't cache (by default).
    if (!_options.url()->isRemote() &&
        targetProfile && 
        targetProfile->isEquivalentTo(getProfile()) )
    {
        return CachePolicy::NO_CACHE;
    }
    else
    {
        return CachePolicy::DEFAULT;
    }
}


osg::Image*
TMSTileSource::createImage(const TileKey&    key,
                           ProgressCallback* progress)
{
    if (_tileMap.valid() && key.getLevelOfDetail() <= _tileMap->getMaxLevel() )
    {
        std::string image_url = _tileMap->getURL( key, _invertY );

        osg::ref_ptr<osg::Image> image;
        if (!image_url.empty())
        {
            image = URI(image_url).readImage( _dbOptions.get(), progress ).getImage();
        }

        if (!image.valid())
        {
            if (image_url.empty() || !_tileMap->intersectsKey(key))
            {
                //We couldn't read the image from the URL or the cache, so check to see if the given key is less than the max level
                //of the tilemap and create a transparent image.
                if (key.getLevelOfDetail() <= _tileMap->getMaxLevel())
                {
                    OE_DEBUG << LC << "Returning empty image " << std::endl;
                    return ImageUtils::createEmptyImage();
                }
            }
        }
        
        if (image.valid() && _options.coverage() == true)
        {
            image->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
            ImageUtils::markAsUnNormalized(image, true);
        }

        return image.release();
    }
    return 0;
}

bool
TMSTileSource::storeImage(const TileKey& key,
                          osg::Image*    image,
                          ProgressCallback* progress)
{
    if ( !_writer.valid() )
    {
        OE_WARN << LC << "Repo is read-only; store failed" << std::endl;
        return false;
    }

    if (_tileMap.valid() && image)
    {
        // compute the URL from the tile map:
        std::string image_url = _tileMap->getURL(key, _invertY);

        // assert the folder exists:
        if ( osgEarth::makeDirectoryForFile(image_url) )
        {
            osgDB::ReaderWriter::WriteResult result;

            if ( _forceRGB && ImageUtils::hasAlphaChannel(image) )
            {
                osg::ref_ptr<osg::Image> rgbImage = ImageUtils::convertToRGB8(image);
                result = _writer->writeImage( *(rgbImage.get()), image_url, _dbOptions.get());
            }
            else
            {
                result = _writer->writeImage(*image, image_url, _dbOptions.get());
            }

            if ( result.error() )
            {
                OE_WARN << LC << "store failed; url=[" << image_url << "] message=[" << result.message() << "]" << std::endl;
                return false;
            }
        }
        else
        {
            OE_WARN << LC << "Failed to make directory for " << image_url << std::endl;
            return false;
        }

        return true;
    }

    return false;
}

int
TMSTileSource::getPixelsPerTile() const
{
    return _tileMap->getFormat().getWidth();
}

std::string
TMSTileSource::getExtension() const 
{
    return _tileMap->getFormat().getExtension();
}

bool
TMSTileSource::resolveWriter()
{
    _writer = osgDB::Registry::instance()->getReaderWriterForMimeType(
        _tileMap->getFormat().getMimeType());

    if ( !_writer.valid() )
    {
        _writer = osgDB::Registry::instance()->getReaderWriterForExtension(
            _tileMap->getFormat().getExtension());

        if ( !_writer.valid() )
        {
            _writer = osgDB::Registry::instance()->getReaderWriterForExtension(
                _options.format().value() );
        }
    }

    // The OSG JPEG writer does not accept RGBA images, so force conversion.
    _forceRGB =
        _writer.valid() &&
        (_writer->acceptsExtension("jpeg") || _writer->acceptsExtension("jpg") );

    if ( _forceRGB )
    {
        OE_INFO << LC << "Note: images will be stored as RGB" << std::endl;
    }

    return _writer.valid();
}
