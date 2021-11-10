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
#include <osgEarth/FeatureSource>
#include <osgEarth/Filter>

#define LC "[FeatureSource] " << getName() << ": "

using namespace osgEarth;

//...................................................................

Config
FeatureSource::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();

    conf.set( "open_write",   openWrite() );
    conf.set( "profile",      profile() );
    conf.set( "geo_interpolation", "great_circle", geoInterp(), GEOINTERP_GREAT_CIRCLE );
    conf.set( "geo_interpolation", "rhumb_line",   geoInterp(), GEOINTERP_RHUMB_LINE );
    conf.set( "fid_attribute", fidAttribute() );
    conf.set( "rewind_polygons", rewindPolygons());
    conf.set( "vdatum", vdatum() );

    if (!filters().empty())
    {
        Config filtersConf;
        for(unsigned i=0; i<filters().size(); ++i)
        {
            filtersConf.add( filters()[i].getConfig() );
        }
        conf.set( "filters", filtersConf );
    }

    return conf;
}

void
FeatureSource::Options::fromConfig(const Config& conf)
{
    _rewindPolygons.init(true);

    conf.get( "open_write",   openWrite() );
    conf.get( "profile",      profile() );
    conf.get( "geo_interpolation", "great_circle", geoInterp(), GEOINTERP_GREAT_CIRCLE );
    conf.get( "geo_interpolation", "rhumb_line",   geoInterp(), GEOINTERP_RHUMB_LINE );
    conf.get( "fid_attribute", fidAttribute() );
    conf.get( "rewind_polygons", rewindPolygons());
    conf.get( "vdatum", vdatum() );

    const Config& filtersConf = conf.child("filters");
    for(ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back( *i );
}

//...................................................................

FeatureSource*
FeatureSource::create(const ConfigOptions& options)
{
    osg::ref_ptr<Layer> layer = Layer::create(options);
    FeatureSource* fs = dynamic_cast<FeatureSource*>(layer.get());
    if (fs)
    {
        layer.release();
        return fs;
    }
    return 0L;
}

//...................................................................

OE_LAYER_PROPERTY_IMPL(FeatureSource, bool, OpenWrite, openWrite);
OE_LAYER_PROPERTY_IMPL(FeatureSource, GeoInterpolation, GeoInterpolation, geoInterp);
OE_LAYER_PROPERTY_IMPL(FeatureSource, std::string, FIDAttribute, fidAttribute);
OE_LAYER_PROPERTY_IMPL(FeatureSource, bool, RewindPolygons, rewindPolygons);

void
FeatureSource::init()
{
    Layer::init();
    _blacklistMutex.setName("FeatureSource(OE).blacklist " + getName());
    _blacklistSize = 0u;
    _featuresCacheMutex.setName("FeatureSource(OE).featuresCache " + getName());
}

Status
FeatureSource::openImplementation()
{
    unsigned int l2CacheSize = 16u;
    if (options().l2CacheSize().isSet())
    {
        l2CacheSize = options().l2CacheSize().get();
    }

    if (l2CacheSize > 0)
    {
        // note: cannot use std::make_unique in C++11
        _featuresCache = std::unique_ptr<FeaturesLRU>(new FeaturesLRU(l2CacheSize));
    }

    Status parent = Layer::openImplementation();
    if (parent.isError())
        return parent;

    // Create and initialize the filters.
    _filters = FeatureFilterChain::create(options().filters(), getReadOptions());
    if (_filters.valid() && _filters->getStatus().isError())
    {
        return _filters->getStatus();
    }

    return Status::NoError;
}

const Status&
FeatureSource::create(
    const FeatureProfile* profile,
    const FeatureSchema& schema,
    const Geometry::Type& geometryType,
    const osgDB::Options* readOptions)
{
    return setStatus(Status::ResourceUnavailable, "Driver does not support create");
}

const FeatureProfile*
FeatureSource::setFeatureProfile(const FeatureProfile* fp)
{
    _featureProfile = fp;

    if (fp != nullptr && options().vdatum().isSet())
    {
        FeatureProfile* new_fp = new FeatureProfile(*fp);

        new_fp->setExtent(GeoExtent(
            SpatialReference::get(
                fp->getExtent().getSRS()->getHorizInitString(),
                options().vdatum().get()),
            fp->getExtent().bounds()));

        _featureProfile = new_fp;

        OE_INFO << LC << "Set vdatum = " << options().vdatum().get() << std::endl;
    }

    return _featureProfile.get();
}

const FeatureProfile*
FeatureSource::getFeatureProfile() const
{
    return _featureProfile.get();
}

const FeatureFilterChain*
FeatureSource::getFilters() const
{
    return _filters.get();
}

const FeatureSchema&
FeatureSource::getSchema() const
{
    static FeatureSchema s_emptySchema;
    return s_emptySchema;
}

void
FeatureSource::addToBlacklist( FeatureID fid )
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.insert( fid );
    _blacklistSize = _blacklist.size();
}

void
FeatureSource::removeFromBlacklist( FeatureID fid )
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.erase( fid );
    _blacklistSize = _blacklist.size();
}

void
FeatureSource::clearBlacklist()
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.clear();
    _blacklistSize = 0u;
}

bool
FeatureSource::isBlacklisted( FeatureID fid ) const
{
    // help reduce mutex contention
    if (_blacklistSize == 0u)
        return false;
    Threading::ScopedReadLock lock(_blacklistMutex);
    return _blacklist.find( fid ) != _blacklist.end();
}

void
FeatureSource::applyFilters(FeatureList& features, const GeoExtent& extent) const
{
    // apply filters before returning.
    if (_filters.valid() && _filters->empty() == false)
    {
        FilterContext cx;
        cx.setProfile( getFeatureProfile() );
        cx.extent() = extent;
        for(FeatureFilterChain::const_iterator filter = _filters->begin(); filter != _filters->end(); ++filter)
        {
            cx = filter->get()->push( features, cx );
        }
    }
}

const GeoExtent&
FeatureSource::getExtent() const
{
    if (_featureProfile.valid())
        return _featureProfile->getExtent();
    else
        return Layer::getExtent();
}

FeatureCursor*
FeatureSource::createFeatureCursor(
    const Query& query,
    ProgressCallback* progress)
{
    return createFeatureCursor(
        query,
        nullptr, // filters
        nullptr, // context
        progress);
}

FeatureCursor*
FeatureSource::createFeatureCursor(
    const Query& query,
    FeatureFilterChain* filters,
    FilterContext* context,
    ProgressCallback* progress)
{
    osg::ref_ptr< FeatureCursor > cursor;

    bool fromCache = false;

    if (_featuresCache)
    {
        // Try reading from the cache first if we have a TileKey.
        if (query.tileKey().isSet())
        {
            ScopedMutexLock lk(_featuresCacheMutex);
            FeaturesLRU::Record result;
            _featuresCache->get(*query.tileKey(), result);
            if (result.valid())
            {
                FeatureList copy(result.value().size());
                std::transform(result.value().begin(), result.value().end(), copy.begin(),
                    [&](const osg::ref_ptr<Feature>& feature) {
                        return osg::clone(feature.get(), osg::CopyOp::DEEP_COPY_ALL);
                    });
                cursor = new FeatureListCursor(copy);
                fromCache = true;
            }
        }
    }

    // Call the implementation if we didn't get a cursor from the cache.
    if (!cursor.valid())
    {
        cursor = createFeatureCursorImplementation(query, progress);
    }

    // Insert it into the cache if we read it from the source itself.
    if (_featuresCache && !fromCache && cursor.valid() && query.tileKey().isSet())
    {
        ScopedMutexLock lk(_featuresCacheMutex);
        FeatureList features;
        cursor->fill(features);

#if 1
        FeatureList copy(features.size());
        std::transform(features.begin(), features.end(), copy.begin(),
            [&](const osg::ref_ptr<Feature>& feature) {
                return osg::clone(feature.get(), osg::CopyOp::DEEP_COPY_ALL);
            });
        _featuresCache->insert(*query.tileKey(), copy);
#else
        // original code: stored raw features in the cache, but they are not const.
        // revisit if/when we refactor this
        _featuresCache->insert(*query.tileKey(), features);
#endif

        cursor = new FeatureListCursor(features);
    }

    if (cursor.valid() && filters)
        return new FilteredFeatureCursor(cursor.get(), filters, context);
    else
        return cursor.release();
}

namespace
{
    struct MultiCursor : public FeatureCursor
    {
        typedef std::vector<osg::ref_ptr<FeatureCursor> > Cursors;

        Cursors _cursors;
        Cursors::iterator _iter;

        MultiCursor(ProgressCallback* progress) :
            FeatureCursor(progress), _iter(_cursors.end()) { }

        void finish()
        {
            _iter = _cursors.begin();
            while(_iter != _cursors.end() && !_iter->get()->hasMore())
                _iter++;
        }

        bool hasMore() const
        {
            return _iter != _cursors.end() && _iter->get()->hasMore();
        }

        Feature* nextFeature()
        {
            Feature* f = _iter->get()->nextFeature();

            while(_iter != _cursors.end() && !_iter->get()->hasMore())
                _iter++;

            return f;
        }
    };
}

FeatureCursor*
FeatureSource::createFeatureCursor(
    const TileKey& key,
    ProgressCallback* progress)
{
    return createFeatureCursor(
        key,
        Distance(0.0, Units::METERS),
        nullptr, // filters
        nullptr, // context
        progress);
}

FeatureCursor*
FeatureSource::createFeatureCursor(
    const TileKey& key,
    FeatureFilterChain* filters,
    FilterContext* context,
    ProgressCallback* progress)
{
    return createFeatureCursor(
        key,
        Distance(0.0, Units::METERS),
        filters,
        context,
        progress);
}

FeatureCursor*
FeatureSource::createFeatureCursor(
    const TileKey& key,
    const Distance& buffer,
    FeatureFilterChain* filters,
    FilterContext* context,
    ProgressCallback* progress)
{
    std::unordered_set<TileKey> keys;
    getKeys(key, buffer, keys);

    if (!keys.empty())
    {
        osg::ref_ptr<MultiCursor> multi = new MultiCursor(progress);

        // Query and collect all the features we need for this tile.
        for (auto& i : keys)
        {
            Query query;
            query.tileKey() = i;

            osg::ref_ptr<FeatureCursor> cursor = createFeatureCursor(
                query,
                filters,
                context,
                progress);

            if (cursor.valid())
            {
                multi->_cursors.push_back(cursor.get());
            }
        }

        if (multi->_cursors.empty())
            return nullptr;

        multi->finish();
        return multi.release();
    }

    else
    {
        GeoExtent localExtent = key.getExtent().transform(_featureProfile->getSRS());
        if (localExtent.isInvalid())
            return nullptr;

        localExtent.expand(buffer*2.0, buffer*2.0);

        // Set up the query; bounds must be in the feature SRS:
        Query query;
        query.bounds() = localExtent.bounds();

        return createFeatureCursor(
            query,
            filters,
            context,
            progress);
    }

    return nullptr;
}

unsigned
FeatureSource::getKeys(
    const TileKey& key,
    const Distance& buffer,
    std::unordered_set<TileKey>& output) const
{
    if (_featureProfile.valid())
    {
        // We need to translate the caller's tilekey into feature source
        // tilekeys and combine multiple queries into one.
        const Profile* profile =
            _featureProfile->isTiled() ? _featureProfile->getTilingProfile() :
            key.getProfile();

        if (profile)
        {
            std::vector<TileKey> intersectingKeys;
            if (buffer.as(Units::METERS) == 0.0)
            {
                profile->getIntersectingTiles(key, intersectingKeys);
            }
            else
            {
                GeoExtent extent = key.getExtent();
                double d = buffer.asDistance(extent.getSRS()->getUnits(), 0.5*(extent.yMin() + extent.yMax()));
                extent.expand(d, d);
                unsigned lod = profile->getEquivalentLOD(key.getProfile(), key.getLOD());
                profile->getIntersectingTiles(extent, lod, intersectingKeys);
            }

            for (int i = 0; i < intersectingKeys.size(); ++i)
            {
                if (_featureProfile->getMaxLevel() >= 0 && (int)intersectingKeys[i].getLOD() > _featureProfile->getMaxLevel())
                    output.insert(intersectingKeys[i].createAncestorKey(_featureProfile->getMaxLevel()));
                else
                    output.insert(intersectingKeys[i]);
            }
        }
        else
        {
            // plan B
            output.insert(key);
        }
    }

    return output.size();
}
