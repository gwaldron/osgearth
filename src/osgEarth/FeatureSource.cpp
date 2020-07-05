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
using namespace OpenThreads;

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
    _blacklistMutex.setName(getName());
}

Status
FeatureSource::openImplementation()
{
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

void
FeatureSource::setFeatureProfile(const FeatureProfile* fp)
{
    _featureProfile = fp;
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
}

void
FeatureSource::removeFromBlacklist( FeatureID fid )
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.erase( fid );
}

void
FeatureSource::clearBlacklist()
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.clear();
}

bool
FeatureSource::isBlacklisted( FeatureID fid ) const
{
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

FeatureCursor*
FeatureSource::createFeatureCursor(const Query& query, ProgressCallback* progress)
{
    return createFeatureCursorImplementation(query, progress);
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
FeatureSource::createFeatureCursor(const TileKey& key, ProgressCallback* progress)
{
    return createFeatureCursor(key, Distance(0.0, Units::METERS), progress);
}

FeatureCursor*
FeatureSource::createFeatureCursor(const TileKey& key, const Distance& buffer, ProgressCallback* progress)
{
    if (_featureProfile.valid())
    {
        // If this is a tiled FS we need to translate the caller's tilekey into
        // feature source tilekeys and combine multiple queries into one.
        const Profile* tilingProfile = _featureProfile->getTilingProfile();
        if (tilingProfile)
        {
            std::vector<TileKey> intersectingKeys;
            if (buffer.as(Units::METERS) == 0.0)
            {
                tilingProfile->getIntersectingTiles(key, intersectingKeys);
            }
            else
            {
                // TODO
                // total cheat to just get the surrounding tiles :)
                GeoExtent extent = key.getExtent();
                extent.expand(extent.width()/2.0, extent.height()/2.0);
                unsigned lod = tilingProfile->getEquivalentLOD(key.getProfile(), key.getLOD());
                tilingProfile->getIntersectingTiles(extent, lod, intersectingKeys);
            }

            UnorderedSet<TileKey> featureKeys;
            for (int i = 0; i < intersectingKeys.size(); ++i)
            {        
                if (intersectingKeys[i].getLOD() > _featureProfile->getMaxLevel())
                    featureKeys.insert(intersectingKeys[i].createAncestorKey(_featureProfile->getMaxLevel()));
                else
                    featureKeys.insert(intersectingKeys[i]);
            }

            osg::ref_ptr<MultiCursor> multi = new MultiCursor(progress);

            // Query and collect all the features we need for this tile.
            for (UnorderedSet<TileKey>::const_iterator i = featureKeys.begin(); i != featureKeys.end(); ++i)
            {
                Query query;        
                query.tileKey() = *i;

                osg::ref_ptr<FeatureCursor> cursor = createFeatureCursor(query, progress);
                if (cursor.valid())
                {
                    multi->_cursors.push_back(cursor.get());
                }
            }

            if (multi->_cursors.empty())
                return NULL;

            multi->finish();
            return multi.release();
        }

        else
        {
            GeoExtent localExtent = key.getExtent().transform(_featureProfile->getSRS());
            if (localExtent.isInvalid())
                return NULL;

            localExtent.expand(buffer*2.0, buffer*2.0);

            // Set up the query; bounds must be in the feature SRS:
            Query query;
            query.bounds() = localExtent.bounds();

            return createFeatureCursor(query, progress);
        }
    }

    return NULL;
}
