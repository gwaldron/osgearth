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
        // note: "true" = threadsafe
        _featuresCache = std::unique_ptr<FeaturesLRU>(new FeaturesLRU(true, l2CacheSize));
    }

    Status parent = Layer::openImplementation();
    if (parent.isError())
        return parent;

    // Create and initialize the filters.
    _filters = FeatureFilterChain::create(options().filters(), getReadOptions());
    if (_filters.getStatus().isError())
    {
        return _filters.getStatus();
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

const FeatureFilterChain&
FeatureSource::getFilters() const
{
    return _filters;
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

#if 0
void
FeatureSource::applyFilters(FeatureList& features, const GeoExtent& extent) const
{
    // apply filters before returning.
    if (!_filters.empty())
    {
        FilterContext cx;
        cx.setProfile(getFeatureProfile());
        cx.extent() = extent;
        for (auto& filter : _filters)
        {
            cx = filter->push(features, cx);
        }
    }
}
#endif

const GeoExtent&
FeatureSource::getExtent() const
{
    if (_featureProfile.valid())
        return _featureProfile->getExtent();
    else
        return Layer::getExtent();
}

FeatureCursor
FeatureSource::createFeatureCursor(
    const Query& query,
    ProgressCallback* progress) const
{
    return createFeatureCursor(query, {}, nullptr, progress);
}

FeatureCursor
FeatureSource::createFeatureCursor(
    const Query& query,
    const FeatureFilterChain& additional_filters,
    FilterContext* context,
    ProgressCallback* progress) const
{
    bool fromCache = false;

    // construct a context if one is not provided
    FilterContext cx_temp;
    cx_temp.setProfile(getFeatureProfile());
    if (query.bounds().isSet())
        cx_temp.extent() = GeoExtent(getFeatureProfile()->getSRS(), query.bounds().get());
    else
        cx_temp.extent() = getFeatureProfile()->getExtent();
    FilterContext* cx_to_use = context ? context : &cx_temp;

    FeatureCursor cursor;

    if (_featuresCache)
    {
        // Try reading from the cache first if we have a TileKey.
        if (query.tileKey().isSet())
        {
            FeaturesLRU::Record result;
            _featuresCache->get(query.tileKey().value(), result);
            if (result.valid())
            {
                cursor.set(new FeatureListCursorImpl(result.value()));
                fromCache = true;
            }
        }
    }

    // Call the implementation if we didn't get a cursor from the cache.
    if (!fromCache)
    {
        cursor.set(createFeatureCursorImplementation(query, progress));

        // Apply any filters owned by the feature source itself.
        // Anything in the L2 memory cache will have run through these filters.
        if (!getFilters().empty())
        {
            cursor.set(new FilteredFeatureCursorImpl(cursor, getFilters(), cx_to_use));
        }

        // Insert it into the L2 memory cache if we read it from the source itself.
        if (_featuresCache && query.tileKey().isSet())
        {
            FeatureList features;
            cursor.fill(features);
            _featuresCache->insert(query.tileKey().value(), features);
            cursor.set(new FeatureListCursorImpl(features));
        }
    }

    // Apply any additional filters the caller requested. Results of this filtering
    // process is not cached in the L2 cahce, so use it sparingly.
    if (!additional_filters.empty())
    {
        cursor.set(new FilteredFeatureCursorImpl(cursor, additional_filters, cx_to_use));
    }

    return cursor;
}

namespace
{
    struct MultiCursorImpl : public FeatureCursorImplementation
    {
        using Cursors = std::vector<FeatureCursor>;

        Cursors _cursors;
        Cursors::iterator _iter;
        osg::ref_ptr<ProgressCallback> _progress;

        MultiCursorImpl(ProgressCallback* progress) :
            _progress(progress), _iter(_cursors.end()) { }

        void finish()
        {
            _iter = _cursors.begin();
            while(_iter != _cursors.end() && !_iter->hasMore())
                _iter++;
        }

        bool hasMore() const override
        {
            return _iter != _cursors.end() && _iter->hasMore();
        }

        osg::ref_ptr<const Feature> nextFeature() override
        {
            auto f = _iter->nextFeature();

            while(_iter != _cursors.end() && !_iter->hasMore())
                _iter++;

            return f;
        }
    };
}

FeatureCursor
FeatureSource::createFeatureCursor(
    ProgressCallback* progress) const
{
    return createFeatureCursor(
        Query(),
        { }, // filters
        nullptr, // context
        progress);
}

FeatureCursor
FeatureSource::createFeatureCursor(
    const TileKey& key,
    ProgressCallback* progress) const
{
    return createFeatureCursor(
        key,
        Distance(0.0, Units::METERS),
        { }, // filters
        nullptr, // context
        progress);
}

FeatureCursor
FeatureSource::createFeatureCursor(
    const TileKey& key,
    const Distance& buffer,
    const FeatureFilterChain& additional_filters,
    FilterContext* context,
    ProgressCallback* progress) const
{
    std::unordered_set<TileKey> keys;
    getKeys(key, buffer, keys);

    if (!keys.empty())
    {
        auto impl = new MultiCursorImpl(progress);
        FeatureCursor cursor(impl);

        // Query and collect all the features we need for this tile.
        for (auto& i : keys)
        {
            Query query;
            query.tileKey() = i;

            FeatureCursor cursor = createFeatureCursor(
                query,
                additional_filters,
                context,
                progress);

            if (cursor.hasMore())
            {
                impl->_cursors.push_back(cursor);
            }
        }

        if (!impl->_cursors.empty())
            impl->finish();
        else
            cursor.set(nullptr);

        return cursor;
    }

    else
    {
        if (!_featureProfile)
            return nullptr;

        GeoExtent localExtent = key.getExtent().transform(_featureProfile->getSRS());
        if (localExtent.isInvalid())
            return nullptr;

        localExtent.expand(buffer*2.0, buffer*2.0);

        // Set up the query; bounds must be in the feature SRS:
        Query query;
        query.bounds() = localExtent.bounds();

        return createFeatureCursor(
            query,
            additional_filters,
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
