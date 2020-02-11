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
#include <osgEarth/TileSourceImageLayer>
#include <osgEarth/Progress>
#include <osgEarth/TimeControl>

using namespace osgEarth;
using namespace osgEarth::Contrib;

#define LC "[TileSourceImageLayer] \"" << getName() << "\" "

//------------------------------------------------------------------------

void
TileSourceImageLayer::Options::fromConfig(const Config& conf)
{
    if (conf.hasValue("driver"))
        driver() = TileSourceOptions(conf);
}

Config
TileSourceImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    // driver?
    return conf;
}

//------------------------------------------------------------------------

REGISTER_OSGEARTH_LAYER(image, TileSourceImageLayer);

void
TileSourceImageLayer::init()
{
    ImageLayer::init();
}

Status
TileSourceImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    if (!_tileSource.valid())
    {
        if (!options().driver().isSet())
        {
            return Status(Status::ConfigurationError, "Missing required tilesource driver");
        }

        _tileSource = TileSourceFactory::create(options().driver().get());
    }

    Status tileSourceStatus;

    if (_tileSource.valid())
    {
        // add the osgDB options string if it's set.
        const optional<std::string>& osgOptions = _tileSource->getOptions().osgOptionString();
        if ( osgOptions.isSet() && !osgOptions->empty() )
        {
            std::string s = getReadOptions()->getOptionString();
            if ( !s.empty() )
                s = Stringify() << osgOptions.get() << " " << s;
            else
                s = osgOptions.get();
            getMutableReadOptions()->setOptionString( s );
        }

        // If we're setting any custom options, do so now before opening:
        if (options().tileSize().isSet())
            _tileSource->setPixelsPerTile(options().tileSize().get());

        if (options().noDataValue().isSet())
            _tileSource->setNoDataValue(options().noDataValue().get());

        if (options().minValidValue().isSet())
            _tileSource->setMinValidValue(options().minValidValue().get());

        if (options().maxValidValue().isSet())
            _tileSource->setMaxValidValue(options().maxValidValue().get());


        // report on a manual override profile:
        if ( _tileSource->getProfile() )
        {
            OE_INFO << LC << "Override profile: "  << _tileSource->getProfile()->toString() << std::endl;
        }

        // Configure the cache settings with information from the TileSource:
        if (getCacheSettings() && getCacheSettings()->isCacheEnabled())
        {
            // Unless the user has already configured an expiration policy, use the "last modified"
            // timestamp of the TileSource to set a minimum valid cache entry timestamp.
            const CachePolicy& cp = getCacheSettings()->cachePolicy().get();

            if ( !cp.minTime().isSet() && !cp.maxAge().isSet() && _tileSource->getLastModifiedTime() > 0)
            {
                // The "effective" policy overrides the runtime policy, but it does not get serialized.
                //getCacheSettings()->cachePolicy()->mergeAndOverride( cp );
                getCacheSettings()->cachePolicy()->minTime() = _tileSource->getLastModifiedTime();
                OE_INFO << LC << "driver says min valid timestamp = " << DateTime(*cp.minTime()).asRFC1123() << "\n";
            }
        }

        // Open the tile source (if it hasn't already been started)
        tileSourceStatus = _tileSource->getStatus();
        if (!tileSourceStatus.isOK())
        {
            tileSourceStatus = _tileSource->open(TileSource::MODE_READ, getReadOptions());
        }

        if (tileSourceStatus.isError())
        {
            _tileSource = NULL;
        }
        else
        {
            // Now that the tile source is open and ready, propagate any user-set
            // properties to and fro.
            if (!_tileSource->getDataExtents().empty())
            {
                dataExtents() = _tileSource->getDataExtents();
            }

            // Set the profile from the TileSource if possible:
            if (getProfile() == NULL)
            {
                OE_DEBUG << LC << "Get Profile from tile source" << std::endl;
                setProfile(_tileSource->getProfile());
            }
        }
    }

    // Otherwise, force cache-only mode (since there is no tilesource). The layer will try to
    // establish a profile from the metadata in the cache instead.
    if (!_tileSource.valid())
    {
        if (getCacheSettings()->isCacheEnabled() && options().cacheId().isSet())
        {
            OE_WARN << LC << tileSourceStatus.message() << std::endl;
            OE_WARN << LC << "will attempt to use the cache as a fallback data source" << std::endl;
            getCacheSettings()->cachePolicy() = CachePolicy::CACHE_ONLY;
        }

        // Finally: if we could not open a TileSource, and there's no cache available,
        // just disable the layer.
        else
        {
            //disable(tileSourceStatus.message());
            return tileSourceStatus;
        }
    }

    return Status::NoError;
}

GeoImage
TileSourceImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (!_tileSource.valid())
        return GeoImage::INVALID;

    // Fail is the image is blacklisted.
    if ( _tileSource->getBlacklist()->contains(key) )
    {
        OE_DEBUG << LC << "createImageFromTileSource: blacklisted(" << key.str() << ")" << std::endl;
        return GeoImage::INVALID;
    }

    if (!mayHaveData(key))
    {
        OE_DEBUG << LC << "createImageFromTileSource: mayHaveData(" << key.str() << ") == false" << std::endl;
        return GeoImage::INVALID;
    }

    // create an image from the tile source.
    osg::ref_ptr<osg::Image> result = _tileSource->createImage( key, NULL, progress );

    // Process images with full alpha to properly support MP blending.    
    if (result.valid() && 
        options().featherPixels() == true)
    {
        ImageUtils::featherAlphaRegions( result.get() );
    }    

    // If image creation failed (but was not intentionally canceled and 
    // didn't time out or end for any other recoverable reason), then
    // blacklist this tile for future requests.
    if (result == 0L)
    {
        if ( progress == 0L || !progress->isCanceled() )
        {
            _tileSource->getBlacklist()->add( key );
        }
    }

    if (progress && progress->isCanceled())
    {
        return GeoImage::INVALID;
    }

    return GeoImage(result.get(), key.getExtent());
}

Status
TileSourceImageLayer::writeImageImplementation(const TileKey& key, const osg::Image* image, ProgressCallback* progress) const
{
    return Status(Status::ServiceUnavailable);
}

SequenceControl*
TileSourceImageLayer::getSequenceControl()
{
    return dynamic_cast<SequenceControl*>(_tileSource.get());
}
