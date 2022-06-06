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
#include <osgEarth/TileSourceElevationLayer>
#include <osgEarth/Progress>

using namespace osgEarth;
using namespace osgEarth::Contrib;

#define LC "[ElevationLayer] \"" << getName() << "\" : "

//#define ANALYZE

//------------------------------------------------------------------------

void
TileSourceElevationLayer::Options::fromConfig( const Config& conf )
{
    if (conf.hasValue("driver"))
        driver() = TileSourceOptions(conf);
}

Config
TileSourceElevationLayer::Options::getConfig() const
{
    Config conf = TileLayer::Options::getConfig();
    return conf;
}
//------------------------------------------------------------------------

namespace
{
    // Opeartion that replaces invalid heights with the NO_DATA_VALUE marker.
    struct NormalizeNoDataValues : public TileSource::HeightFieldOperation
    {
        NormalizeNoDataValues(const ElevationLayer* layer)
        {
            _noDataValue   = layer->getNoDataValue();
            _minValidValue = layer->getMinValidValue();
            _maxValidValue = layer->getMaxValidValue();
        }

        void operator()(osg::ref_ptr<osg::HeightField>& hf)
        {
            if ( hf.valid() )
            {
                osg::FloatArray* values = hf->getFloatArray();
                for(osg::FloatArray::iterator i = values->begin(); i != values->end(); ++i)
                {
                    float& value = *i;
                    if ( osg::isNaN(value) || osg::equivalent(value, _noDataValue) || value < _minValidValue || value > _maxValidValue )
                    {
                        OE_DEBUG << "Replaced " << value << " with NO_DATA_VALUE" << std::endl;
                        value = NO_DATA_VALUE;
                    }
                } 
            }
        }

        float _noDataValue, _minValidValue, _maxValidValue;
    };
}

REGISTER_OSGEARTH_LAYER(elevation, TileSourceElevationLayer);

void
TileSourceElevationLayer::init()
{
    ElevationLayer::init();
}

TileSource::HeightFieldOperation*
TileSourceElevationLayer::getOrCreatePreCacheOp() const
{
    if ( !_preCacheOp.valid() )
    {
        static Mutex s_mutex;
        Threading::ScopedLock lock(s_mutex);
        if ( !_preCacheOp.valid() )
        {
            _preCacheOp = new NormalizeNoDataValues(this);
        }
    }
    return _preCacheOp.get();
}

Status
TileSourceElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
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

GeoHeightField
TileSourceElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (!_tileSource.valid() || !_tileSource->isOK())
        return GeoHeightField::INVALID;

    if (progress && progress->isCanceled())
        return GeoHeightField::INVALID;

    osg::ref_ptr<osg::HeightField> result;

    // If the key is blacklisted, fail.
    if ( _tileSource->getBlacklist()->contains( key ))
    {
        OE_DEBUG << LC << "Tile " << key.str() << " is blacklisted " << std::endl;
        if (progress) progress->message() = "blacklisted";
        return GeoHeightField::INVALID;
    }

    // Only try to get data if the source actually has data
    if (!mayHaveData(key))
    {
        OE_DEBUG << LC << "Source for layer has no data at " << key.str() << std::endl;
        //if (progress) progress->message() = "mayHaveData=false";
        return GeoHeightField::INVALID;
    }

    // Make it from the source:
    result = _tileSource->createHeightField( key, getOrCreatePreCacheOp(), progress );

    // Blacklist the tile if it is the same projection as the source and
    // we can't get it and it wasn't cancelled
    if (!result.valid())
    {
        if ( progress == 0L || !progress->isCanceled() )
        {
            _tileSource->getBlacklist()->add( key );
        }
    }

    if (progress && progress->isCanceled())
    {
        return GeoHeightField::INVALID;
    }

    return GeoHeightField(result.release(), key.getExtent());
}
