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
#include "FeatureSource"
#include "Query"

#define LC "[FeatureSource] " << getName() << ": "

using namespace osgEarth;

//...................................................................

Config
FeatureSource::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();

    conf.set("open_write", openWrite());
    conf.set("profile", profile());
    conf.set("geo_interpolation", "great_circle", geoInterp(), GEOINTERP_GREAT_CIRCLE);
    conf.set("geo_interpolation", "rhumb_line", geoInterp(), GEOINTERP_RHUMB_LINE);
    conf.set("fid_attribute", fidAttribute());
    conf.set("rewind_polygons", rewindPolygons());
    conf.set("vdatum", vdatum());

    if (!filters().empty())
    {
        conf.set_with_function("filters", [this](Config& conf) {
            for (auto& filter : filters())
                conf.add(filter.getConfig());
        });
    }

    return conf;
}

void
FeatureSource::Options::fromConfig(const Config& conf)
{
    conf.get("open_write", openWrite());
    conf.get("profile", profile());
    conf.get("geo_interpolation", "great_circle", geoInterp(), GEOINTERP_GREAT_CIRCLE);
    conf.get("geo_interpolation", "rhumb_line", geoInterp(), GEOINTERP_RHUMB_LINE);
    conf.get("fid_attribute", fidAttribute());
    conf.get("rewind_polygons", rewindPolygons());
    conf.get("vdatum", vdatum());

    for(auto& filterConf : conf.child("filters").children())
        filters().push_back(filterConf);
}

//...................................................................

OE_LAYER_PROPERTY_IMPL(FeatureSource, bool, OpenWrite, openWrite);
OE_LAYER_PROPERTY_IMPL(FeatureSource, GeoInterpolation, GeoInterpolation, geoInterp);
OE_LAYER_PROPERTY_IMPL(FeatureSource, std::string, FIDAttribute, fidAttribute);
OE_LAYER_PROPERTY_IMPL(FeatureSource, bool, RewindPolygons, rewindPolygons);

void
FeatureSource::init()
{
    super::init();
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
        _featuresCache = std::unique_ptr<FeaturesLRU>(new FeaturesLRU(l2CacheSize));
    }

    Status parent = super::openImplementation();
    if (parent.isError())
        return parent;

    // Create and initialize the filters.
    _filters = FeatureFilterChain::create(options().filters(), getReadOptions());
    return _filters.getStatus();
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

        OE_DEBUG << LC << "Set vdatum = " << options().vdatum().get() << std::endl;
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

void
FeatureSource::applyFilters(FeatureList& features, const GeoExtent& extent) const
{
    // apply filters before returning.
    if (!_filters.empty())
    {
        FilterContext cx;
        cx.setProfile(getFeatureProfile());
        cx.extent() = extent;
        for(auto& filter : _filters)
        {
            cx = filter->push(features, cx);
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
            while (_iter != _cursors.end() && !_iter->get()->hasMore())
                _iter++;
        }

        bool hasMore() const
        {
            return _iter != _cursors.end() && _iter->get()->hasMore();
        }

        Feature* nextFeature()
        {
            Feature* f = _iter->get()->nextFeature();

            while (_iter != _cursors.end() && !_iter->get()->hasMore())
                _iter++;

            return f;
        }
    };
}

void
FeatureSource::dirty()
{
    if (_featuresCache)
    {
        std::lock_guard<std::mutex> lk(_featuresCacheMutex);
        _featuresCache->clear();
    }

    super::dirty();
}

osg::ref_ptr<FeatureCursor>
FeatureSource::createFeatureCursor(
    const Query& query,
    const FeatureFilterChain& post_filters,
    FilterContext* context,
    ProgressCallback* progress) const
{
    osg::ref_ptr<FeatureCursor> result;

    bool fromCache = false;

    FilterContext temp_cx;
    if (context)
        temp_cx = *context;


    if (temp_cx.profile() == nullptr)
        temp_cx.setProfile(getFeatureProfile());
    

    // TileKey path:
    if (query.tileKey().isSet())
    {
        // Try reading from the cache first if we have a TileKey.
        if (_featuresCache)
        {
            FeaturesLRU::Record cache_entry;
            {
                std::lock_guard<std::mutex> lk(_featuresCacheMutex);
                _featuresCache->get(*query.tileKey(), cache_entry);
            }
            if (result.valid())
            {
                FeatureList copy(cache_entry.value().size());
                std::transform(cache_entry.value().begin(), cache_entry.value().end(), copy.begin(),
                    [&](auto& feature) { return new Feature(*feature); });
                result = new FeatureListCursor(std::move(copy));
                fromCache = true;
            }
        }

        if (!temp_cx.extent().isSet())
        {
            temp_cx.extent() = query.tileKey()->getExtent();
        }

        if (!result.valid())
        {
            std::unordered_set<TileKey> keys;
            getKeys(query.tileKey().value(), query.buffer().value(), keys);

            osg::ref_ptr<MultiCursor> multi = new MultiCursor(progress);

            // Query and collect all the features we need for this tile.
            for (auto& sub_key : keys)
            {
                auto sub_cursor = createFeatureCursorImplementation(Query(sub_key), progress);
                if (sub_cursor)
                    multi->_cursors.emplace_back(sub_cursor);

                if (progress && progress->isCanceled())
                    return {};
            }

            if (multi->_cursors.empty())
            {
                return { };
            }

            multi->finish();
            result = multi;
        }
    }

    else
    {
        OE_SOFT_ASSERT(!query.buffer().isSet(), "Buffer not supported for non-tilekey queries; ignoring");

        if (!temp_cx.extent().isSet() && _featureProfile.valid())
        {
            if (query.bounds().isSet())
                temp_cx.extent() = GeoExtent(_featureProfile->getSRS(), query.bounds().get());
            else
                temp_cx.extent() = _featureProfile->getExtent();
        }

        result = createFeatureCursorImplementation(query, progress);
    }

    if (result.valid())
    {
        if (!fromCache)
        {
            // Apply this feature source's core filters. This happend pre-cache, as opposed 
            // to the caller's filters, which apply post-cache.
            if (!_filters.empty())
            {
                FeatureList features;
                result->fill(features);
                temp_cx = _filters.push(features, temp_cx);
                result = new FeatureListCursor(std::move(features));
            }

            // Apply the optional FID attribute:
            if (options().fidAttribute().isSet())
            {
                FeatureList features;
                result->fill(features);
                for(auto& feature : features)
                {
                    std::string attr = feature->getString(options().fidAttribute().get());
                    for (auto& c : attr)
                        if (!isdigit(c))
                            c = ' ';
                    feature->setFID(as<FeatureID>(attr, 0));
                }
                result = new FeatureListCursor(std::move(features));
            }

            // Write the feature set to the L2 cache.
            // TODO: If we have a persistent cache, write to that as well here
            if (_featuresCache)
            {
                FeatureList features;
                result->fill(features, [](const Feature* f) { return f != nullptr; });

                // clone the list for caching:
                FeatureList clone(features.size());
                std::transform(features.begin(), features.end(), clone.begin(),
                    [&](auto& feature) { return new Feature(*feature); });

                std::lock_guard<std::mutex> lk(_featuresCacheMutex);
                _featuresCache->insert(*query.tileKey(), clone);

                result = new FeatureListCursor(std::move(features));
            }
        }

        // apply caller's filters. These are NOT cached by this class because the 
        // modifications are the resposibility of the caller.
        if (!post_filters.empty())
        {
            FeatureList features;
            result->fill(features);
            temp_cx = post_filters.push(features, temp_cx);
            result = new FeatureListCursor(std::move(features));
        }

        // copy out temporary context back out.
        if (context)
            *context = temp_cx;
    }
    
    return result;
}

unsigned
FeatureSource::getKeys(const TileKey& key, const Distance& buffer, std::unordered_set<TileKey>& output) const
{
    if (_featureProfile.valid())
    {
        unsigned firstLevel = _featureProfile->getFirstLevel();
        unsigned maxLevel = _featureProfile->getMaxLevel() >= 0u ? _featureProfile->getMaxLevel() : UINT_MAX;

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
                auto lod = intersectingKeys[i].getLOD();
                if (lod > maxLevel)
                {
                    // fall back to the max level if necessary:
                    output.insert(intersectingKeys[i].createAncestorKey(maxLevel));
                }
                else if (lod >= firstLevel)
                {
                    // must be at least the first available level
                    output.insert(intersectingKeys[i]);
                }
            }
        }
        else
        {
            // plan B
            if (key.getLOD() >= firstLevel && key.getLOD() <= maxLevel)
            {
                output.insert(key);
            }
        }
    }

    return output.size();
}

void FeatureSource::addedToMap(const class Map* map)
{
    for (auto& filter : _filters)
    {
        filter->addedToMap(map);
    }

    super::addedToMap(map);
}
