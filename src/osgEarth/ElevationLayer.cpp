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
#include <osgEarth/ElevationLayer>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osgEarth/MemCache>
#include <osgEarth/Metrics>
#include <osgEarth/NetworkMonitor>
#include <osgEarth/Math>
#include <cinttypes>

using namespace osgEarth;

#define LC "[" << className() << "] \"" << getName() << "\" "

//#define ANALYZE

//------------------------------------------------------------------------

Config
ElevationLayer::Options::getConfig() const
{
    Config conf = TileLayer::Options::getConfig();
    conf.set("vdatum", verticalDatum() );
    conf.set("offset", offset());
    conf.set("nodata_policy", "default",     _noDataPolicy, NODATA_INTERPOLATE );
    conf.set("nodata_policy", "interpolate", _noDataPolicy, NODATA_INTERPOLATE );
    conf.set("nodata_policy", "msl",         _noDataPolicy, NODATA_MSL );
    return conf;
}

void
ElevationLayer::Options::fromConfig( const Config& conf )
{
    conf.get("vdatum", verticalDatum() );
    conf.get("vsrs", verticalDatum() );    // back compat
    conf.get("offset", offset() );
    conf.get("nodata_policy", "default",     _noDataPolicy, NODATA_INTERPOLATE );
    conf.get("nodata_policy", "interpolate", _noDataPolicy, NODATA_INTERPOLATE );
    conf.get("nodata_policy", "msl",         _noDataPolicy, NODATA_MSL );

    // ElevationLayers are special in that visible really maps to whether the layer is open or closed
    // If a layer is marked as enabled (openAutomatically) but also marked as visible=false set
    // openAutomatically to false to prevent the layer from being opened at startup.
    // This prevents a deadlock b/c setVisible is called during the VisibleLayer openImplementation and it
    // will call setVisible on the Layer there which will result in trying to close the layer while it's opening.
    // There may be a better way to sync these up but will require a bit of rework.
    if (*openAutomatically() && !*visible())
    {
        openAutomatically() = false;
    }
}

//------------------------------------------------------------------------

namespace
{
    // perform very basic sanity-check validation on a heightfield.
    bool validateHeightField(osg::HeightField* hf)
    {
        if (!hf)
            return false;
        if (hf->getNumRows() < 1 || hf->getNumRows() > 1024) {
            OE_WARN << "row count = " << hf->getNumRows() << std::endl;
            return false;
        }
        if (hf->getNumColumns() < 1 || hf->getNumColumns() > 1024) {
            OE_WARN << "col count = " << hf->getNumColumns() << std::endl;
            return false;
        }
        if (hf->getHeightList().size() != hf->getNumColumns() * hf->getNumRows()) {
            OE_WARN << "mismatched data size" << std::endl;
            return false;
        }
        //if (hf->getXInterval() < 1e-5 || hf->getYInterval() < 1e-5)
        //    return false;

        return true;
    }
}

//------------------------------------------------------------------------

void
ElevationLayer::init()
{
    TileLayer::init();

    // open and visible are the same thing for elevation layers
    _visibleTiedToOpen = true;

    // override with a different default tile size since elevation
    // tiles need overlapping edges
    if (!options().tileSize().isSet())
    {
        options().tileSize().init(257u);
    }

    // a small L2 cache will help with things like normal map creation
    // (i.e. queries that sample neighboring tiles)
    if (!options().l2CacheSize().isSet())
    {
        options().l2CacheSize() = 4u;
    }

    // Disable max-level support for elevation data because it makes no sense.
    options().maxLevel().clear();
    options().maxResolution().clear();

    // elevation layers do not render directly; rather, a composite of elevation data
    // feeds the terrain engine to permute the mesh.
    setRenderType(RENDERTYPE_NONE);
}

//void
//ElevationLayer::setVisible(bool value)
//{
//    VisibleLayer::setVisible(value);
//    if (value)
//        open();
//    else
//        close();
//}

void
ElevationLayer::setVerticalDatum(const std::string& value)
{
    setOptionThatRequiresReopen(options().verticalDatum(), value);

    // vertical datum change requires a profile override:
    applyProfileOverrides(_profile);
}

const std::string&
ElevationLayer::getVerticalDatum() const
{
    return options().verticalDatum().get();
}

void
ElevationLayer::setOffset(bool value)
{
    setOptionThatRequiresReopen(options().offset(), value);
}

bool
ElevationLayer::getOffset() const
{
    return options().offset().get();
}

void
ElevationLayer::setNoDataPolicy(const ElevationNoDataPolicy& value)
{
    setOptionThatRequiresReopen(options().noDataPolicy(), value);
}

const ElevationNoDataPolicy&
ElevationLayer::getNoDataPolicy() const
{
    return options().noDataPolicy().get();
}

void
ElevationLayer::normalizeNoDataValues(osg::HeightField* hf) const
{
    if ( hf )
    {
        osg::FloatArray* values = hf->getFloatArray();
        for(osg::FloatArray::iterator i = values->begin(); i != values->end(); ++i)
        {
            float& value = *i;
            if ( osg::isNaN(value) || osg::equivalent(value, getNoDataValue()) || value < getMinValidValue() || value > getMaxValidValue() )
            {
                value = NO_DATA_VALUE;
            }
        }
    }
}

void
ElevationLayer::applyProfileOverrides(osg::ref_ptr<const Profile>& inOutProfile) const
{
    // Check for a vertical datum override.
    if ( inOutProfile.valid() && options().verticalDatum().isSet() )
    {
        std::string vdatum = options().verticalDatum().get();

        std::string profileVDatumStr = _profile->getSRS()->getVertInitString();
        if (profileVDatumStr.empty())
            profileVDatumStr = "geodetic";

        OE_INFO << LC << "Override vdatum = " << vdatum << " (was " << profileVDatumStr << ")" << std::endl;

        if ( !ciEquals(getProfile()->getSRS()->getVertInitString(), vdatum) )
        {
            ProfileOptions po = getProfile()->toProfileOptions();
            po.vsrsString() = vdatum;
            inOutProfile = Profile::create(po);
        }
    }
}

GeoHeightField
ElevationLayer::assembleHeightField(const TileKey& key, ProgressCallback* progress)
{
    // If we got here, assert that there's a non-null layer profile.
    if (!getProfile())
    {
        setStatus(Status::Error(Status::AssertionFailure, "assembleHeightField with undefined profile"));
        return { };
    }

    GeoHeightField output;
    unsigned targetLOD = getProfile()->getEquivalentLOD(key.getProfile(), key.getLOD());

    std::vector<TileKey> intersectingKeys;
    getProfile()->getIntersectingTiles(key, intersectingKeys);

    using KeyedSource = std::pair<TileKey, GeoHeightField>;
    std::vector<KeyedSource> sources;

    if (intersectingKeys.size() > 0)
    {
        bool hasAtLeastOneSourceAtTargetLOD = false;

        for (auto& intersectingKey : intersectingKeys)
        {
            TileKey subKey = intersectingKey;
            GeoHeightField subTile;
            while (subKey.valid() && !subTile.valid())
            {
                subTile = createHeightFieldInKeyProfile(subKey, progress);
                if (!subTile.valid())
                    subKey.makeParent();

                if (progress && progress->isCanceled())
                    return {};
            }

            if (subTile.valid())
            {
                if (subKey.getLOD() == targetLOD)
                {
                    hasAtLeastOneSourceAtTargetLOD = true;
                }

                // got a valid image, so add it to our sources collection:
                sources.emplace_back(subKey, subTile);
            }
        }

        // If we actually got at least one piece of usable data,
        // move ahead and build a mosaic of all sources.
        if (hasAtLeastOneSourceAtTargetLOD)
        {
            unsigned cols = getTileSize();
            unsigned rows = getTileSize();

            // sort the sources by LOD (highest first).
            std::sort(
                sources.begin(), sources.end(),
                [](const auto& lhs, const auto& rhs) {
                    return lhs.first.getLOD() > rhs.first.getLOD();
                });

            // assume all tiles to mosaic are in the same SRS.
            auto* key_srs = key.getExtent().getSRS();
            auto* source_srs = sources[0].second.getSRS();

            // new output:
            auto mosaic = new osg::HeightField();
            mosaic->allocate(cols, rows);
            auto& heights = mosaic->getHeightList();
            for(int i=0; i<cols*rows; ++i)
                heights[i] = NO_DATA_VALUE;

            // Working set of points. it's much faster to xform an entire vector all at once.
            std::vector<osg::Vec3d> points;
            points.resize(cols * rows);

            double minx, miny, maxx, maxy;
            key.getExtent().getBounds(minx, miny, maxx, maxy);
            double dx = (maxx - minx) / (double)(cols-1);
            double dy = (maxy - miny) / (double)(rows-1);

            // Source bounds and projection grid. We use cols-1 and rows-1
            // since we want to sample edge to edge, unlike with imagery
            Bounds sourceBounds;
            sources[0].second.getSRS()->getBounds(sourceBounds);

            // build a grid of sample points:
            for (unsigned t = 0; t < rows; ++t)
            {
                double y = miny + (dy * (double)t);
                for (unsigned s = 0; s < cols; ++s)
                {
                    double x = minx + (dx * (double)s);
                    points[t * cols + s] = { x, y, 0.0 };
                }
            }

            // transform the sample points to the SRS of our source data tiles.
            // NOTE: point.z() will hold a vertical offset if the layers' vdatums are different;
            // we will add it back in later.
            if (source_srs && key_srs)
            {
                key_srs->transform(points, source_srs);

                if (sourceBounds.valid())
                {
                    for (auto& point : points)
                    {
                        point.x() = clamp(point.x(), sourceBounds.xMin(), sourceBounds.xMax());
                        point.y() = clamp(point.y(), sourceBounds.yMin(), sourceBounds.yMax());
                    }
                }
            }

            // Mosaic our sources into a single output image.
            for (int row = 0; row < rows; ++row)
            {
                for (int col = 0; col < cols; ++col)
                {
                    int i = row * cols + col;
                    auto& h = mosaic->getHeight(col, row);
                    for (unsigned k = 0; k < sources.size() && h == NO_DATA_VALUE; ++k)
                    {
                        auto& source = sources[k].second;
                        if (source.getExtent().contains(points[i].x(), points[i].y())) // prevents clamping of out-of-bounds points
                        {
                            h = sources[k].second.getElevation(points[i].x(), points[i].y(), INTERP_BILINEAR);
                        }
                    }
                    if (h != NO_DATA_VALUE)
                    {
                        // apply reverse vdatum offset if necessary
                        h -= points[i].z();
                    }
                }
            }

            output = GeoHeightField(mosaic, key.getExtent());
        }
    }

    return output;
}

#if 0
void
ElevationLayer::assembleHeightField(const TileKey& key,
                                    osg::ref_ptr<osg::HeightField>& out_hf,
                                    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    // Collect the heightfields for each of the intersecting tiles.
    GeoHeightFieldVector heightFields;

    //Determine the intersecting keys
    std::vector< TileKey > intersectingTiles;

    if (key.getLOD() > 0u)
    {
        getProfile()->getIntersectingTiles(key, intersectingTiles);
    }

    else
    {
        // LOD is zero - check whether the LOD mapping went out of range, and if so,
        // fall back until we get valid tiles. This can happen when you have two
        // profiles with very different tile schemes, and the "equivalent LOD"
        // surpasses the max data LOD of the tile source.
        unsigned numTilesThatMayHaveData = 0u;

        int intersectionLOD = getProfile()->getEquivalentLOD(key.getProfile(), key.getLOD());

        while (numTilesThatMayHaveData == 0u && intersectionLOD >= 0)
        {
            intersectingTiles.clear();
            getProfile()->getIntersectingTiles(key.getExtent(), intersectionLOD, intersectingTiles);

            for (unsigned int i = 0; i < intersectingTiles.size(); ++i)
            {
                const TileKey& layerKey = intersectingTiles[i];
                if (mayHaveData(layerKey) == true)
                {
                    ++numTilesThatMayHaveData;
                }
            }

            --intersectionLOD;
        }
    }

    // collect heightfield for each intersecting key. Note, we're hitting the
    // underlying tile source here, so there's no vetical datum shifts happening yet.
    // we will do that later.
    if ( intersectingTiles.size() > 0 )
    {
        for (unsigned int i = 0; i < intersectingTiles.size(); ++i)
        {
            const TileKey& layerKey = intersectingTiles[i];

            if ( isKeyInLegalRange(layerKey) )
            {
                Threading::ScopedReadLock lock(inUseMutex());
                GeoHeightField hf = createHeightFieldImplementation(layerKey, progress);
                if (hf.valid())
                {
                    heightFields.push_back( hf );
                }
            }
        }

        // If we actually got a HeightField, resample/reproject it to match the incoming TileKey's extents.
        if (heightFields.size() > 0)
        {
            unsigned int width = 0;
            unsigned int height = 0;

            for (GeoHeightFieldVector::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
            {
                if (itr->getHeightField()->getNumColumns() > width)
                    width = itr->getHeightField()->getNumColumns();
                if (itr->getHeightField()->getNumRows() > height)
                    height = itr->getHeightField()->getNumRows();
            }

            //Now sort the heightfields by resolution to make sure we're sampling the highest resolution one first.
            std::sort( heightFields.begin(), heightFields.end(), GeoHeightField::SortByResolutionFunctor());

            out_hf = new osg::HeightField();
            out_hf->allocate(width, height);

            //Go ahead and set up the heightfield so we don't have to worry about it later
            double minx, miny, maxx, maxy;
            key.getExtent().getBounds(minx, miny, maxx, maxy);
            double dx = (maxx - minx)/(double)(width-1);
            double dy = (maxy - miny)/(double)(height-1);

            //Create the new heightfield by sampling all of them.
            for (unsigned int c = 0; c < width; ++c)
            {
                double x = minx + (dx * (double)c);
                for (unsigned r = 0; r < height; ++r)
                {
                    double y = miny + (dy * (double)r);

                    //For each sample point, try each heightfield.  The first one with a valid elevation wins.
                    float elevation = NO_DATA_VALUE;
                    osg::Vec3 normal(0,0,1);

                    for (GeoHeightFieldVector::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
                    {
                        // get the elevation value, at the same time transforming it vertically into the
                        // requesting key's vertical datum.
                        float e = 0.0;
                        if (itr->getElevation(key.getExtent().getSRS(), x, y, INTERP_BILINEAR, key.getExtent().getSRS(), e))
                        {
                            elevation = e;
                            break;
                        }
                    }
                    out_hf->setHeight( c, r, elevation );
                }
            }
        }
        else
        {
            //if (progress && progress->message().empty())
            //    progress->message() = "assemble yielded no heightfields";
        }
    }
    else
    {
        //if (progress && progress->message().empty())
        //    progress->message() = "assemble yielded no intersecting tiles";
    }


    // If the progress was cancelled clear out any of the output data.
    if (progress && progress->isCanceled())
    {
        out_hf = 0;
    }
}
#endif

GeoHeightField
ElevationLayer::createHeightField(const TileKey& key)
{
    return createHeightField(key, 0L);
}

GeoHeightField
ElevationLayer::createHeightField(const TileKey& key, ProgressCallback* progress)
{    
    OE_PROFILING_ZONE;
    OE_PROFILING_ZONE_TEXT(getName());
    OE_PROFILING_ZONE_TEXT(key.str());

    // If the layer is disabled, bail out
    if (!isOpen())
    {
        return GeoHeightField::INVALID;
    }

    NetworkMonitor::ScopedRequestLayer layerRequest(getName());

    GeoHeightField result = createHeightFieldInKeyProfile(key, progress);

    return result;
}

GeoHeightField
ElevationLayer::createHeightFieldInKeyProfile(const TileKey& key, ProgressCallback* progress)
{
    GeoHeightField result;
    osg::ref_ptr<osg::HeightField> hf;

    osg::ref_ptr< const Profile > profile = getProfile();
    if (!profile.valid() || !isOpen())
    {
        return result;
    }

    // Prevents more than one thread from creating the same object
    // at the same time. This helps a lot with elevation data since
    // the many queries cross tile boundaries (like calculating 
    // normal maps)
    auto& policy = getCacheSettings()->cachePolicy().get();
    auto* cacheBin = policy.isCacheEnabled() ? getCacheBin(key.getProfile()) : nullptr;

    // cache key combines the key with the full signature (incl vdatum)
    // the cache key combines the Key and the horizontal profile.
    auto cacheKey = Cache::makeCacheKey(key.str() + "-" + key.getProfile()->getHorizSignature(), "elevation");
    std::string memCacheKey;

    // see if there's a persistent cache.
    bool memCacheAvailable = _memCache.valid();

    // Prevent multiple threads from creating the same object at the same time
    // IF there is a cache present. The gate goes here so that the first thread
    // to enter has a chance to write to a cache, allowing the followers to read
    // from that cache immediately when unblocked.
    // 
    // NOTE! This can cause deadlock. Cause: the ElevationPool has a r/w lock that it
    // uses when synchronizing and refreshing from the Map. If this functions is called
    // from the ElevationPool and this gate locks while holding that read lock, all
    // other threads will be blocked and you get deadlock.T
    //ScopedGate<TileKey> gate(_sentry, key, memCacheAvailable || diskCacheAvailable);

    // Try the L2 memory cache first:
    bool fromMemCache = false;
    if ( _memCache.valid() )
    {
        memCacheKey = std::to_string(getRevision()) + cacheKey;

        CacheBin* bin = _memCache->getOrCreateDefaultBin();
        ReadResult cacheResult = bin->readObject(memCacheKey, 0L);
        if ( cacheResult.succeeded() )
        {
            result = GeoHeightField(
                static_cast<osg::HeightField*>(cacheResult.releaseObject()),
                key.getExtent());

            fromMemCache = true;
        }
    }

    // Next try the main cache:
    if ( !result.valid() )
    {
        // validate the existance of a valid layer profile.
        if ( !policy.isCacheOnly() && !getProfile() )
        {
            disable("Could not establish a valid profile.. did you set one?");
            return GeoHeightField::INVALID;
        }

        // Now attempt to read from the cache. Since the cached data is stored in the
        // map profile, we can try this first.
        bool fromCache = false;

        osg::ref_ptr< osg::HeightField > cachedHF;

        if (cacheBin != nullptr && policy.isCacheReadable())
        {
            ReadResult r = cacheBin->readObject(cacheKey, nullptr);
            if ( r.succeeded() )
            {
                bool expired = policy.isExpired(r.lastModifiedTime());
                cachedHF = r.get<osg::HeightField>();
                if ( cachedHF && validateHeightField(cachedHF.get()) )
                {
                    if (!expired)
                    {
                        hf = cachedHF;
                        fromCache = true;
                    }
                }
            }
        }

        // if we're cache-only, but didn't get data from the cache, fail silently.
        if ( !hf.valid() && policy.isCacheOnly() )
        {
            return GeoHeightField::INVALID;
        }

        // If we didn't get anything from cache, time to create one:
        if ( !hf.valid() )
        {
            // Check that the key is legal (in valid LOD range, etc.)
            if ( !isKeyInLegalRange(key) )
            {
                return GeoHeightField::INVALID;
            }

            if (key.getProfile()->isHorizEquivalentTo(profile.get()))
            {
                Threading::ScopedReadLock lock(inUseMutex());
                result = createHeightFieldImplementation(key, progress);
            }
            else
            {
                // If the profiles are different, use a compositing method to assemble the tile.
                result = assembleHeightField(key, progress);
            }

            // Check for cancelation before writing to a cache
            if (progress && progress->isCanceled())
            {
                return GeoHeightField::INVALID;
            }

            // The const_cast is safe here because we just created the
            // heightfield from scratch...not from a cache.
            hf = const_cast<osg::HeightField*>(result.getHeightField());

            // validate it to make sure it's legal.
            if ( hf.valid() && !validateHeightField(hf.get()) )
            {
                OE_WARN << LC << "Generated an illegal heightfield!" << std::endl;
                hf = 0L; // to fall back on cached data if possible.
            }

            // Pre-caching operations:
            {
                OE_PROFILING_ZONE_NAMED("nodata normalize");
                normalizeNoDataValues(hf.get());
            }

            // If the result is good, we now have a heightfield but its vertical values
            // are still relative to the source's vertical datum. Convert them.
            if (hf.valid() && !key.getExtent().getSRS()->isVertEquivalentTo(profile->getSRS()))
            {
                OE_PROFILING_ZONE_NAMED("vdatum xform");

                VerticalDatum::transform(
                    profile->getSRS()->getVerticalDatum(),    // from
                    key.getExtent().getSRS()->getVerticalDatum(),  // to
                    key.getExtent(),
                    hf.get());
            }

            // Invoke user callbacks
            if (result.valid())
            {
                invoke_onCreate(key, result);
            }

            // If we have a cacheable heightfield, and it didn't come from the cache
            // itself, cache it now.
            if (hf.valid() &&
                !fromCache &&
                cacheBin && policy.isCacheWriteable() )
            {
                OE_PROFILING_ZONE_NAMED("cache write");
                cacheBin->write(cacheKey, hf.get(), nullptr);
            }

            // If we have an expired heightfield from the cache and were not able to create
            // any new data, just return the cached data.
            if (!hf.valid() && cachedHF.valid())
            {
                hf = cachedHF;
            }

            // No luck on any path:
            if ( !hf.valid() )
            {
                return GeoHeightField::INVALID;
            }
        }

        if ( hf.valid() )
        {
            result = GeoHeightField( hf.get(), key.getExtent() );
        }
    }

    // Check for cancelation before writing to a cache:
    if ( progress && progress->isCanceled() )
    {
        return GeoHeightField::INVALID;
    }

    // write to mem cache if needed:
    if ( result.valid() && !fromMemCache && _memCache.valid() )
    {
        CacheBin* bin = _memCache->getOrCreateDefaultBin();
        bin->write(memCacheKey, result.getHeightField(), 0L);
    }

    return result;
}

Status
ElevationLayer::writeHeightField(const TileKey& key, const osg::HeightField* hf, ProgressCallback* progress) const
{
    if (isWritingSupported() && isWritingRequested())
    {
        Threading::ScopedReadLock lock(inUseMutex());
        return writeHeightFieldImplementation(key, hf, progress);
    }
    return Status::ServiceUnavailable;
}

Status
ElevationLayer::writeHeightFieldImplementation(const TileKey& key, const osg::HeightField* hf, ProgressCallback* progress) const
{
    return Status::ServiceUnavailable;
}

void
ElevationLayer::invoke_onCreate(const TileKey& key, GeoHeightField& data)
{
    if (_callbacks.empty() == false) // not thread-safe but that's ok
    {
        // Copy the vector to prevent thread lockup
        Callbacks temp;

        _callbacks.lock();
        temp = _callbacks;
        _callbacks.unlock();

        for(Callbacks::const_iterator i = temp.begin();
            i != temp.end();
            ++i)
        {
            i->get()->onCreate(key, data);
        }
    }
}

void
ElevationLayer::addCallback(ElevationLayer::Callback* c)
{
    _callbacks.lock();
    _callbacks.push_back(c);
    _callbacks.unlock();
}

void
ElevationLayer::removeCallback(ElevationLayer::Callback* c)
{
    _callbacks.lock();
    Callbacks::iterator i = std::find(_callbacks.begin(), _callbacks.end(), c);
    if (i != _callbacks.end())
        _callbacks.erase(i);
    _callbacks.unlock();
}


//------------------------------------------------------------------------

#undef  LC
#define LC "[ElevationLayers] "

ElevationLayerVector::ElevationLayerVector()
{
    //nop
}


ElevationLayerVector::ElevationLayerVector(const ElevationLayerVector& rhs) :
osg::MixinVector< osg::ref_ptr<ElevationLayer> >( rhs )
{
    //nop
}



namespace
{
    struct LayerData
    {
        osg::ref_ptr<ElevationLayer> layer;
        TileKey key;
        bool isFallback = false;
        int index = 0;
    };

    using LayerDataVector = std::vector<LayerData>;

    // thread-local working space to prevent constant reallocation
    struct Workspace
    {
        LayerDataVector contenders;
        LayerDataVector offsets;

        GeoHeightFieldVector heightFields;
        std::vector<TileKey> heightFieldActualKeys;
        GeoHeightFieldVector offsetFields;
        std::vector<bool>  heightFallback;
        std::vector<bool>  heightFailed;
        std::vector<bool>  offsetFailed;
    };
    //thread_local Workspace s_per_thread_workspace;
}

bool
ElevationLayerVector::populateHeightField(
    osg::HeightField*   hf,
    std::vector<float>* resolutions,
    const TileKey&      key,
    const Profile*      haeProfile,
    RasterInterpolation interpolation,
    ProgressCallback*   progress) const
{
    // heightfield must already exist.
    if ( !hf )
        return false;

    OE_PROFILING_ZONE;

    // if the caller provided an "HAE map profile", he wants an HAE elevation grid even if
    // the map profile has a vertical datum. This is the usual case when building the 3D
    // terrain, for example. Construct a temporary key that doesn't have the vertical
    // datum info and use that to query the elevation data.
    TileKey keyToUse = key;
    if ( haeProfile )
    {
        keyToUse = TileKey(key.getLOD(), key.getTileX(), key.getTileY(), haeProfile );
    }

    // Collect the valid layers for this tile.
    Workspace w;
#if 0
    auto& w = s_per_thread_workspace;
    w.contenders.clear();
    w.offsets.clear();
#endif

#ifdef ANALYZE
    struct LayerAnalysis {
        LayerAnalysis() : samples(0), used(false), failed(false), fallback(false), actualKeyValid(true) { }
        int samples; bool used; bool failed; bool fallback; bool actualKeyValid; std::string message;
    };
    std::map<ElevationLayer*, LayerAnalysis> layerAnalysis;
#endif

    int i;

    // Track the number of layers that would return fallback data.
    // if ALL layers would provide fallback data, we can exit early
    // and return nothing.
    unsigned numFallbackLayers = 0;

    // Check them in reverse order since the highest priority is last.
    for (i = size()-1; i>=0; --i)
    {
        ElevationLayer* layer = (*this)[i].get();

        if (layer->isOpen())
        {
            // calculate the resolution-mapped key (adjusted for tile resolution differential).
            TileKey mappedKey = keyToUse.mapResolution(
                hf->getNumColumns(),
                layer->getTileSize() );

            bool useLayer = true;
            TileKey bestKey( mappedKey );

            // Check whether the non-mapped key is valid according to the user's minLevel setting.
            // We wll ignore the maxDataLevel setting, because we account for that by getting
            // the "best available" key later. We must keep these layers around in case we need
            // to fill in empty spots.
            if (key.getLOD() < layer->getMinLevel())
            {
                useLayer = false;
            }

            // GW - this was wrong because it would exclude layers with a maxDataLevel set
            // below the requested LOD ... when in fact we need around for fallback.
            //if ( !layer->isKeyInLegalRange(key) )
            //{
            //    useLayer = false;
            //}

            // Find the "best available" mapped key from the tile source:
            else
            {
                bestKey = layer->getBestAvailableTileKey(mappedKey);
                if (bestKey.valid())
                {
                    // If the bestKey is not the mappedKey, this layer is providing
                    // fallback data (data at a lower resolution than requested)
                    if ( mappedKey != bestKey )
                    {
                        numFallbackLayers++;
                    }
                }
                else
                {
                    useLayer = false;
                }
            }

            if ( useLayer )
            {
                if ( layer->isOffset() )
                {
                    w.offsets.push_back(LayerData());
                    LayerData& ld = w.offsets.back();
                    ld.layer = layer;
                    ld.key = bestKey;
                    ld.isFallback = bestKey != mappedKey;
                    ld.index = i;
                }
                else
                {
                    w.contenders.push_back(LayerData());
                    LayerData& ld = w.contenders.back();
                    ld.layer = layer;
                    ld.key = bestKey;
                    ld.isFallback = bestKey != mappedKey;
                    ld.index = i;
                }

#ifdef ANALYZE
                layerAnalysis[layer].used = true;
#endif
            }
        }
    }

    // nothing? bail out.
    if ( w.contenders.empty() && w.offsets.empty() )
    {
        return false;
    }

    // if everything is fallback data, bail out.
    if ( w.contenders.size() + w.offsets.size() == numFallbackLayers )
    {
        return false;
    }

    // Sample the layers into our target.
    unsigned numColumns = hf->getNumColumns();
    unsigned numRows    = hf->getNumRows();
    double   xmin       = key.getExtent().xMin();
    double   ymin       = key.getExtent().yMin();
    double   dx         = key.getExtent().width() / (double)(numColumns-1);
    double   dy         = key.getExtent().height() / (double)(numRows-1);

    const SpatialReference* keySRS = keyToUse.getProfile()->getSRS();

    bool realData = false;

    unsigned int total = numColumns * numRows;

    int nodataCount = 0;

    TileKey actualKey; // Storage if a new key needs to be constructed

    bool requiresResample = true;

    // If we only have a single contender layer, and the tile is the same size as the requested
    // heightfield then we just use it directly and avoid having to resample it
    if (w.contenders.size() == 1 && w.offsets.empty())
    {
        ElevationLayer* layer = w.contenders[0].layer.get();

        GeoHeightField layerHF = layer->createHeightField(w.contenders[0].key, progress);
        if (layerHF.valid())
        {
            if (layerHF.getHeightField()->getNumColumns() == hf->getNumColumns() &&
                layerHF.getHeightField()->getNumRows() == hf->getNumRows())
            {
                requiresResample = false;

                memcpy(hf->getFloatArray()->asVector().data(),
                    layerHF.getHeightField()->getFloatArray()->asVector().data(),
                    sizeof(float) * hf->getFloatArray()->size()
                );

                realData = true;

                if (resolutions)
                {
                    std::pair<double,double> res = w.contenders[0].key.getResolution(hf->getNumColumns());
                    for(unsigned i=0; i<hf->getNumColumns()*hf->getNumRows(); ++i)
                        (*resolutions)[i] = res.second;
                }
            }
        }
    }

    // If we need to mosaic multiple layers or resample it to a new output tilesize go through a resampling loop.
    if (requiresResample)
    {
        // We will load the actual heightfields on demand. We might not need them all.
        w.heightFields.assign(w.contenders.size(), {});
        w.heightFieldActualKeys.assign(w.contenders.size(), {});
        w.offsetFields.assign(w.offsets.size(), {});
        w.heightFallback.assign(w.contenders.size(), false);
        w.heightFailed.assign(w.contenders.size(), false);
        w.offsetFailed.assign(w.offsets.size(), false);

        // Initialize the actual keys to match the contender keys.
        // We'll adjust these as necessary if we need to fall back
        for(unsigned i=0; i< w.contenders.size(); ++i)
        {
            w.heightFieldActualKeys[i] = w.contenders[i].key;
        }

        // The maximum number of heightfields to keep in this local cache
        const unsigned maxHeightFields = 50;
        unsigned numHeightFieldsInCache = 0;

        for (unsigned c = 0; c < numColumns; ++c)
        {
            double x = xmin + (dx * (double)c);

            // periodically check for cancelation
            if (progress && progress->isCanceled())
            {
                return false;
            }

            for (unsigned r = 0; r < numRows; ++r)
            {
                double y = ymin + (dy * (double)r);

                // Collect elevations from each layer as necessary.
                int resolvedIndex = -1;

                float resolution = FLT_MAX;

                osg::Vec3 normal_sum(0, 0, 0);

                for (int i = 0; i < w.contenders.size() && resolvedIndex < 0; ++i)
                {
                    ElevationLayer* layer = w.contenders[i].layer.get();
                    TileKey& contenderKey = w.contenders[i].key;
                    int index = w.contenders[i].index;

                    if (w.heightFailed[i])
                        continue;

                    GeoHeightField& layerHF = w.heightFields[i];
                    TileKey& actualKey = w.heightFieldActualKeys[i];

                    if (!layerHF.valid())
                    {
                        // We couldn't get the heightfield from the cache, so try to create it.
                        // We also fallback on parent layers to make sure that we have data at the location even if it's fallback.
                        while (!layerHF.valid() && actualKey.valid() && layer->isKeyInLegalRange(actualKey))
                        {
                            layerHF = layer->createHeightField(actualKey, progress);
                            if (!layerHF.valid())
                            {
                                actualKey.makeParent();
                            }
                        }

                        // Mark this layer as fallback if necessary.
                        if (layerHF.valid())
                        {
                            //TODO: check this. Should it be actualKey != keyToUse...?
                            w.heightFallback[i] =
                                w.contenders[i].isFallback ||
                                (actualKey != contenderKey);

                            numHeightFieldsInCache++;
                        }
                        else
                        {
                            w.heightFailed[i] = true;
#ifdef ANALYZE
                            layerAnalysis[layer].failed = true;
                            layerAnalysis[layer].actualKeyValid = actualKey->valid();
                            if (progress) layerAnalysis[layer].message = progress->message();
#endif
                            continue;
                        }
                    }

                    if (layerHF.valid())
                    {
                        bool isFallback = w.heightFallback[i];
#ifdef ANALYZE
                        layerAnalysis[layer].fallback = isFallback;
#endif

                        // We only have real data if this is not a fallback heightfield.
                        if (!isFallback)
                        {
                            realData = true;
                        }

                        float elevation;
                        if (layerHF.getElevation(keySRS, x, y, interpolation, keySRS, elevation))
                        {
                            if (elevation != NO_DATA_VALUE)
                            {
                                // remember the index so we can only apply offset layers that
                                // sit on TOP of this layer.
                                resolvedIndex = index;

                                hf->setHeight(c, r, elevation);

                                resolution = actualKey.getResolution(hf->getNumColumns()).second;
#ifdef ANALYZE
                                layerAnalysis[layer].samples++;
#endif
                            }
                            else
                            {
                                ++nodataCount;
                            }
                        }
                    }


                    // Clear the heightfield cache if we have too many heightfields in the cache.
                    if (numHeightFieldsInCache >= maxHeightFields)
                    {
                        //OE_NOTICE << "Clearing cache" << std::endl;
                        for (unsigned int k = 0; k < w.heightFields.size(); k++)
                        {
                            w.heightFields[k] = GeoHeightField::INVALID;
                            w.heightFallback[k] = false;
                        }
                        numHeightFieldsInCache = 0;
                    }
                }

                for (int i = w.offsets.size() - 1; i >= 0; --i)
                {
                    if (progress && progress->isCanceled())
                        return false;

                    // Only apply an offset layer if it sits on top of the resolved layer
                    // (or if there was no resolved layer).
                    if (resolvedIndex >= 0 && w.offsets[i].index < resolvedIndex)
                        continue;

                    TileKey& contenderKey = w.offsets[i].key;

                    if (w.offsetFailed[i] == true)
                        continue;

                    GeoHeightField& layerHF = w.offsetFields[i];
                    if (!layerHF.valid())
                    {
                        ElevationLayer* offset = w.offsets[i].layer.get();

                        layerHF = offset->createHeightField(contenderKey, progress);
                        if (!layerHF.valid())
                        {
                            w.offsetFailed[i] = true;
                            continue;
                        }
                    }

                    // If we actually got a layer then we have real data
                    realData = true;

                    float elevation = 0.0f;
                    if (layerHF.getElevation(keySRS, x, y, interpolation, keySRS, elevation) &&
                        elevation != NO_DATA_VALUE &&
                        !osg::equivalent(elevation, 0.0f) )
                    {
                        hf->getHeight(c, r) += elevation;

                        // Technically this is correct, but the resultin normal maps
                        // look awful and faceted.
                        resolution = std::min(
                            resolution,
                            (float)contenderKey.getResolution(hf->getNumColumns()).second);
                    }
                }

                if (resolutions)
                {
                    (*resolutions)[r*numColumns+c] = resolution;
                }
            }
        }
    }

#ifdef ANALYZE
    {
        static std::mutex m;
        std::lock_guard<std::mutex> lock(m);
        std::cout << key.str() << ": ";
        for (std::map<ElevationLayer*, LayerAnalysis>::const_iterator i = layerAnalysis.begin();
            i != layerAnalysis.end(); ++i)
        {
            std::cout << i->first->getName()
                << " used=" << i->second.used
                << " failed=" << i->second.failed
                << " akv=" << i->second.actualKeyValid
                << " fallback=" << i->second.fallback
                << " samples=" << i->second.samples
                << " msg=" << i->second.message
                << "; ";
        }
        std::cout << std::endl;
    }
#endif

    // Resolve any invalid heights in the output heightfield.
    HeightFieldUtils::resolveInvalidHeights(hf, key.getExtent(), NO_DATA_VALUE, 0L);

    if (progress && progress->isCanceled())
    {
        return false;
    }

    // Return whether or not we actually read any real data
    return realData;
}
