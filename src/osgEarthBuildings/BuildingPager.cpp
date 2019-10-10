/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include "BuildingPager"
#include "Analyzer"
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osgEarthSymbology/Query>
#include <osgEarthSymbology/StyleSheet>
#include <osgUtil/Optimizer>
#include <osgUtil/Statistics>
#include <osg/Version>
#include <osg/CullFace>
#include <osg/Geometry>
#include <osg/MatrixTransform>

#include <osgDB/WriteFile>

#include <osg/ConcurrencyViewerMacros>

#define LC "[BuildingPager] "

using namespace osgEarth::Buildings;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define OE_TEST OE_DEBUG

#define USE_OSGEARTH_ELEVATION_POOL

#ifndef GL_CLIP_DISTANCE0
#define GL_CLIP_DISTANCE0 0x3000
#endif

namespace
{
    // Callback to force building threads onto the high-latency pager queue.
    struct HighLatencyFileLocationCallback : public osgDB::FileLocationCallback
    {
        Location fileLocation(const std::string& filename, const osgDB::Options* options)
        {
            return REMOTE_FILE;
        }

        bool useFileCache() const { return false; }
    };

    // Callback that culls unused stuff from the ObjectCache.
    // Unfortunately we cannot use this in OSG < 3.5.1 because of an OSG threading bug;
    // https://github.com/openscenegraph/OpenSceneGraph/commit/5b17e3bc2a0c02cf84d891bfdccf14f170ee0ec8 
    struct TendArtCacheCallback : public osg::NodeCallback
    {
        TendArtCacheCallback(osgDB::ObjectCache* cache) : _cache(cache) { }

        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            if (nv->getFrameStamp())
            {
                _cache->updateTimeStampOfObjectsInCacheWithExternalReferences(nv->getFrameStamp()->getReferenceTime());
                _cache->removeExpiredObjectsInCache(10.0);
            }
            traverse(node, nv);
        }        
        
        osg::ref_ptr<osgDB::ObjectCache> _cache;
    };

    struct ArtCache : public osgDB::ObjectCache
    {
        unsigned size() const { return this->_objectCache.size(); }
    };
}


BuildingPager::BuildingPager(const Profile* profile) :
SimplePager( profile ),
_index     ( 0L ),
_filterUsage(FILTER_USAGE_NORMAL)
{
    // Replace tiles with higher LODs.
    setAdditive( false );

    // Force building generation onto the high latency queue.
    setFileLocationCallback( new HighLatencyFileLocationCallback() );

    _profile = ::getenv("OSGEARTH_BUILDINGS_PROFILE") != 0L;

    // An object cache for shared resources like textures, atlases, and instanced models.
    _artCache = new ArtCache(); //osgDB::ObjectCache();

    // Texture object cache
    _texCache = new TextureCache();

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,1)
    // Read this to see why the version check exists:
    // https://github.com/openscenegraph/OpenSceneGraph/commit/5b17e3bc2a0c02cf84d891bfdccf14f170ee0ec8

    // This callack expires unused items from the art cache periodically
    this->addCullCallback(new TendArtCacheCallback(_artCache.get()));
#endif

    this->getOrCreateStateSet()->setAttributeAndModes(
        new osg::CullFace(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
}

void
BuildingPager::setSession(Session* session)
{
    _session = session;

    if ( session )
    {
        _compiler = new BuildingCompiler(session);

        _compiler->setUsage(_filterUsage);

        // Analyze the styles to determine the min and max LODs.
        // Styles are named by LOD.
        if ( _session->styles() )
        {
            optional<unsigned> minLOD(0u), maxLOD(0u);
            for(unsigned i=0; i<30; ++i)
            {
                std::string styleName = Stringify() << i;
                const Style* style = _session->styles()->getStyle(styleName, false);
                if ( style )
                {
                    if ( !minLOD.isSet() )
                    {
                        minLOD = i;
                    }
                    else if ( !maxLOD.isSet() || maxLOD.get() < i)
                    {
                        maxLOD = i;
                    }
                }
            }
            if ( minLOD.isSet() && !maxLOD.isSet() )
                maxLOD = minLOD.get();

            setMinLevel( minLOD.get() );
            setMaxLevel( maxLOD.get() );

            OE_INFO << LC << "Min level = " << getMinLevel() << "; max level = " << getMaxLevel() << std::endl;
        }
    }
}

void
BuildingPager::setFeatureSource(FeatureSource* features)
{
    _features = features;
}

void
BuildingPager::setCatalog(BuildingCatalog* catalog)
{
    _catalog = catalog;
}

void
BuildingPager::setCompilerSettings(const CompilerSettings& settings)
{
    _compilerSettings = settings;

    // Apply the range factor from the settings:
    if (_compilerSettings.rangeFactor().isSet())
    {
        this->setRangeFactor(_compilerSettings.rangeFactor().get());
    }
}

void BuildingPager::setIndex(FeatureIndexBuilder* index)
{
    _index = index;
}

void
BuildingPager::setElevationPool(ElevationPool* pool)
{
    _elevationPool = pool;
}

void BuildingPager::setFilterUsage(FilterUsage usage)
{
   _filterUsage = usage;
}

bool
BuildingPager::cacheReadsEnabled(const osgDB::Options* readOptions) const
{
    CacheSettings* cacheSettings = CacheSettings::get(readOptions);
    return
        cacheSettings && 
        cacheSettings->getCacheBin() &&
        cacheSettings->cachePolicy()->isCacheReadable();
}

bool
BuildingPager::cacheWritesEnabled(const osgDB::Options* writeOptions) const
{
    CacheSettings* cacheSettings = CacheSettings::get(writeOptions);
    return
        cacheSettings &&
        cacheSettings->getCacheBin() &&
        cacheSettings->cachePolicy()->isCacheWriteable();
}

osg::Node*
BuildingPager::createNode(const TileKey& tileKey, ProgressCallback* progress)
{
    if ( !_session.valid() || !_compiler.valid() || !_features.valid() )
    {
        OE_WARN << LC << "Misconfiguration error; make sure Session and FeatureSource are set\n";
        return 0L;
    }


    // For debugging. This tile is in Seattle.
    //if (tileKey.str() != "14/2625/5725" && tileKey.str() != "13/1312/2862")
    //    return 0L;

    if ( progress )
        progress->collectStats() = _profile;

    OE_START_TIMER(total);
    unsigned numFeatures = 0;
    
    std::string activityName("Load building tile " + tileKey.str());
    Registry::instance()->startActivity(activityName);

    osg::CVMarkerSeries series("PagingThread");
    osg::CVSpan UpdateTick(series, 4, activityName.c_str());

    // result:
    osg::ref_ptr<osg::Node> node;


    // I/O Options to use throughout the build process.
    // Install an "art cache" in the read options so that images can be 
    // shared throughout the creation process. This is critical for sharing 
    // textures and especially for texture atlas usage.
    osg::ref_ptr<osgDB::Options> readOptions = Registry::cloneOrCreateOptions(_session->getDBOptions());
    readOptions->setObjectCache(_artCache.get());
    readOptions->setObjectCacheHint(osgDB::Options::CACHE_IMAGES);

    // TESTING:
    Registry::instance()->startActivity("Bld art cache", Stringify()<<((ArtCache*)(_artCache.get()))->size());
    Registry::instance()->startActivity("Bld tex cache", Stringify() << _texCache->_cache.size());
    Registry::instance()->startActivity("RCache skins", Stringify() << _session->getResourceCache()->getSkinStats()._entries);
    Registry::instance()->startActivity("RCache insts", Stringify() << _session->getResourceCache()->getInstanceStats()._entries);

    // Holds all the final output.
    CompilerOutput output;
    output.setName(tileKey.str());
    output.setTileKey(tileKey);
    output.setIndex(_index);
    output.setTextureCache(_texCache.get());
    output.setFilterUsage(_filterUsage);
    
    bool canceled = false;
    bool caching = true;

    osg::CVMarkerSeries series2("SubloadParentTask");
    // Try to load from the cache.
    if (cacheReadsEnabled(readOptions.get()) && !canceled)
    {
        OE_START_TIMER(readCache);

        osg::CVSpan UpdateTick(series2, 4, "ReadFromCache");

        node = output.readFromCache(readOptions.get(), progress);

        if (progress && progress->collectStats())
            progress->stats("pager.readCache") = OE_GET_TIMER(readCache);
    }

    bool fromCache = node.valid();

    canceled = canceled || (progress && progress->isCanceled());

    if (!node.valid() && !canceled)
    {

        // fetch the style for this LOD:
        std::string styleName = Stringify() << tileKey.getLOD();
        const Style* style = _session->styles() ? _session->styles()->getStyle(styleName) : 0L;

        // Create a cursor to iterator over the feature data:
        Query query;
        query.tileKey() = tileKey;
        
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor(query, progress);
        if (cursor.valid() && cursor->hasMore() && !canceled)
        {
           osg::CVSpan UpdateTick(series, 4, "buildFromScratch");
           
           osg::ref_ptr<BuildingFactory> factory = new BuildingFactory();

            factory->setSession(_session.get());
            factory->setCatalog(_catalog.get());
            factory->setOutputSRS(_session->getMapSRS());

            // Prepare the terrain envelope, for clamping.
            // TODO: review the LOD selection..
            OE_START_TIMER(envelope);

            osg::ref_ptr<ElevationEnvelope> envelope;

            osg::ref_ptr<ElevationPool> pool;
            if (_elevationPool.lock(pool))
            {
                envelope = pool->createEnvelope(
                    _session->getMapSRS(),      // SRS of input features
                    tileKey.getLOD());          // LOD at which to clamp

                if (progress && progress->collectStats())
                    progress->stats("pager.envelope") = OE_GET_TIMER(envelope);

                if (!envelope.valid())
                {
                    // if this happens, it means that the clamper most likely lost its connection
                    // to the underlying map for some reason (Map closed, e.g.). In this case we
                    // should just cancel the tile operation.
                    OE_INFO << LC << "Failed to create clamping envelope for " << tileKey.str() << "\n";
                }
            }
            canceled = canceled || !envelope.valid();

            while (cursor->hasMore() && !canceled)
            {
                Feature* feature = cursor->nextFeature();
                numFeatures++;
                
                BuildingVector buildings;
                if (!factory->create(feature, tileKey.getExtent(), envelope.get(), style, buildings, readOptions.get(), progress))
                {
                    canceled = true;
                }

                if (!canceled && !buildings.empty())
                {
                    if (output.getLocalToWorld().isIdentity())
                    {
                        output.setLocalToWorld(buildings.front()->getReferenceFrame());
                    }

                    // for indexing, if enabled:
                    output.setCurrentFeature(feature);

                    _compiler->setUsage(_filterUsage);

                    if (!_compiler->compile(buildings, output, readOptions.get(), progress))
                    {
                        canceled = true;
                    }
                }
            }

            if (!canceled)
            {
                // set the distance at which details become visible.
                osg::BoundingSphere tileBound = getBounds(tileKey);
                output.setRange(tileBound.radius() * getRangeFactor());
                node = output.createSceneGraph(_session.get(), _compilerSettings, readOptions.get(), progress);

                osg::MatrixTransform * mt = dynamic_cast<osg::MatrixTransform *> (node.get());
                if (mt)
                {
                   osg::ref_ptr<osg::Group> oqn;
                   if (osgEarth::OcclusionQueryNodeFactory::_occlusionFactory) {
                      oqn = osgEarth::OcclusionQueryNodeFactory::_occlusionFactory->createQueryNode();
                   }
                   if (oqn.get()) 
                   {
                      oqn->setName("BuildingPager::oqn");
                      //oqn.get()->setDebugDisplay(true);
                      while (mt->getNumChildren()) {
                         oqn.get()->addChild(mt->getChild(0));
                         mt->removeChild(mt->getChild(0));
                      }
                      mt->addChild(oqn.get());
                   }

                }
            }
            else
            {
                //OE_INFO << LC << "Tile " << tileKey.str() << " was canceled " << progress->message() << "\n";
            }
        }

        // This can go here now that we can serialize DIs and TBOs.
        if (node.valid() && !canceled)
        {
            OE_START_TIMER(postProcess);
            osg::CVSpan UpdateTick(series2, 4, "postProcess");

            // apply render symbology, if it exists.
            if (style)
                applyRenderSymbology(node.get(), *style);

            output.postProcess(node.get(), _compilerSettings, progress);

            if (progress && progress->collectStats())
                progress->stats("pager.postProcess") = OE_GET_TIMER(postProcess);
        }

        if (node.valid() && cacheWritesEnabled(readOptions.get()) && !canceled)
        {
            OE_START_TIMER(writeCache);
            
            osg::CVSpan UpdateTick(series2, 4, "writeToCache");

            output.writeToCache(node.get(), readOptions.get(), progress);

            if (progress && progress->collectStats())
                progress->stats("pager.writeCache") = OE_GET_TIMER(writeCache);
        }
    }

    Registry::instance()->endActivity(activityName);

    double totalTime = OE_GET_TIMER(total);

    // STATS:
    if ( progress && progress->collectStats() && !progress->stats().empty() && (fromCache || numFeatures > 0))
    {
        Analyzer analyzer;
        analyzer.analyze(node.get(), progress, numFeatures, totalTime, tileKey);
    }

    if (canceled)
    {
        OE_INFO << LC << "Building tile " << tileKey.str() << " - canceled" << std::endl;
        return 0L;
    }
    else
    {

        return node.release();
    }
}

void
BuildingPager::applyRenderSymbology(osg::Node* node, const Style& style) const
{
    const RenderSymbol* render = style.get<RenderSymbol>();
    if ( render )
    {
        if ( render->depthTest().isSet() )
        {
            node->getOrCreateStateSet()->setMode(
                GL_DEPTH_TEST,
                (render->depthTest() == true? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

        if ( render->backfaceCulling().isSet() )
        {
            node->getOrCreateStateSet()->setMode(
                GL_CULL_FACE,
                (render->backfaceCulling() == true? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

#ifndef OSG_GLES2_AVAILABLE
        if ( render->clipPlane().isSet() )
        {
            GLenum mode = GL_CLIP_DISTANCE0 + render->clipPlane().value();
            node->getOrCreateStateSet()->setMode(mode, 1);
        }
#endif

        if ( render->order().isSet() || render->renderBin().isSet() )
        {
            osg::StateSet* ss = node->getOrCreateStateSet();
            int binNumber = render->order().isSet() ? (int)render->order()->eval() : ss->getBinNumber();
            std::string binName =
                render->renderBin().isSet() ? render->renderBin().get() :
                ss->useRenderBinDetails() ? ss->getBinName() : "DepthSortedBin";
            ss->setRenderBinDetails(binNumber, binName);
        }

        if ( render->minAlpha().isSet() )
        {
            DiscardAlphaFragments().install( node->getOrCreateStateSet(), render->minAlpha().value() );
        }
        

        if ( render->transparent() == true )
        {
            osg::StateSet* ss = node->getOrCreateStateSet();
            ss->setRenderingHint( ss->TRANSPARENT_BIN );
        }
    }
}