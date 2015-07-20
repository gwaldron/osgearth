/* --*-c++-*-- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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

#include <osgEarthFeatures/FeatureModelGraph>
#include <osgEarthFeatures/CropFilter>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/Session>

#include <osgEarth/Map>
#include <osgEarth/Capabilities>
#include <osgEarth/Clamping>
#include <osgEarth/ClampableNode>
#include <osgEarth/CullingUtils>
#include <osgEarth/DrapeableNode>
#include <osgEarth/ElevationLOD>
#include <osgEarth/ElevationQuery>
#include <osgEarth/FadeEffect>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>

#include <osg/CullFace>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>

#include <algorithm>
#include <iterator>

#define LC "[FeatureModelGraph] " << getName()

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#undef USE_PROXY_NODE_FOR_TESTING
#define OE_TEST OE_NULL
//#define OE_TEST OE_NOTICE

namespace
{
    // callback to force features onto the high-latency queue.
    struct HighLatencyFileLocationCallback : public osgDB::FileLocationCallback
    {
        Location fileLocation(const std::string& filename, const osgDB::Options* options)
        {
            return REMOTE_FILE;
        }

        bool useFileCache() const { return false; }
    };
}

//---------------------------------------------------------------------------

// pseudo-loader for paging in feature tiles for a FeatureModelGraph.

namespace
{
    UID                               _uid         = 0;
    Threading::ReadWriteMutex         _fmgMutex;
    typedef std::map<UID, osg::observer_ptr<FeatureModelGraph> > FMGRegistry;
    FMGRegistry _fmgRegistry;

    static std::string s_makeURI( UID uid, unsigned lod, unsigned x, unsigned y ) 
    {
        std::stringstream buf;
        buf << uid << "." << lod << "_" << x << "_" << y << ".osgearth_pseudo_fmg";
        std::string str;
        str = buf.str();
        return str;
    }

    osg::Group* createPagedNode(const osg::BoundingSphered& bs, 
                                const std::string& uri, 
                                float minRange, 
                                float maxRange, 
                                float priOffset, 
                                float priScale,
                                RefNodeOperationVector* postMergeOps,
                                osgDB::FileLocationCallback* flc)
    {
#ifdef USE_PROXY_NODE_FOR_TESTING
        osg::ProxyNode* p = new osg::ProxyNode();
        p->setCenter( bs.center() );
        p->setRadius( bs.radius() );
        p->setFileName( 0, uri );
#else
        PagedLODWithNodeOperations* p = new PagedLODWithNodeOperations(postMergeOps);
        p->setCenter( bs.center() );
        p->setRadius( bs.radius() ); //maxRange + bs.radius() );
        //p->setRadius(-1);
        p->setFileName( 0, uri );
        p->setRange( 0, minRange, maxRange );
        p->setPriorityOffset( 0, priOffset );
        p->setPriorityScale( 0, priScale );
#endif

        // force onto the high-latency thread pool.
        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setFileLocationCallback( flc );
        p->setDatabaseOptions( options );

        return p;
    }
}


/**
 * A pseudo-loader for paged feature tiles.
 */
struct osgEarthFeatureModelPseudoLoader : public osgDB::ReaderWriter
{
    osgEarthFeatureModelPseudoLoader()
    {
        supportsExtension( "osgearth_pseudo_fmg", "Feature model pseudo-loader" );
    }

    const char* className()
    { // override
        return "osgEarth Feature Model Pseudo-Loader";
    }

    ReadResult readNode(const std::string& uri, const Options* options) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension(uri) ) )
            return ReadResult::FILE_NOT_HANDLED;

        UID uid;
        unsigned lod, x, y;
        sscanf( uri.c_str(), "%u.%d_%d_%d.%*s", &uid, &lod, &x, &y );

        osg::ref_ptr<FeatureModelGraph> graph = getGraph(uid);
        if ( graph.valid() )
        {
            // Take a reference on the map to avoid map destruction during thread operation
            osg::ref_ptr<const Map> map = graph->getSession()->getMap();
            if (map.valid() == true)
            {
                Registry::instance()->startActivity(uri);
                osg::Node* node = graph->load( lod, x, y, uri );
                Registry::instance()->endActivity(uri);
                return ReadResult(node);
            }
        }

        return ReadResult::ERROR_IN_READING_FILE;
    }

    static UID registerGraph( FeatureModelGraph* graph )
    {
        Threading::ScopedWriteLock lock( _fmgMutex );
        UID key = ++_uid;
        _fmgRegistry[key] = graph;
        OE_TEST << "Registered FMG " << key << std::endl;
        return key;
    }

    static void unregisterGraph( UID uid )
    {
        Threading::ScopedWriteLock lock( _fmgMutex );
        _fmgRegistry.erase( uid );
        OE_TEST << "UNregistered FMG " << uid << std::endl;
    }

    static FeatureModelGraph* getGraph( UID uid ) 
    {
        Threading::ScopedReadLock lock( _fmgMutex );
        FMGRegistry::const_iterator i = _fmgRegistry.find( uid );
        return i != _fmgRegistry.end() ? i->second.get() : 0L;
    }

    /** User data structure for traversing the feature graph. */
    struct CullUserData : public osg::Referenced
    {
        double _cameraElevation;
    };
};

REGISTER_OSGPLUGIN(osgearth_pseudo_fmg, osgEarthFeatureModelPseudoLoader);


namespace
{    
    GeoExtent
    s_getTileExtent( unsigned lod, unsigned tileX, unsigned tileY, const GeoExtent& fullExtent )
    {
        double w = fullExtent.width();
        double h = fullExtent.height();
        for( unsigned i=0; i<lod; ++i ) {
            w *= 0.5;
            h *= 0.5;
        }
        return GeoExtent(
            fullExtent.getSRS(),
            fullExtent.xMin() + w * (double)tileX,
            fullExtent.yMin() + h * (double)tileY,
            fullExtent.xMin() + w * (double)(tileX+1),
            fullExtent.yMin() + h * (double)(tileY+1) );
    }


    struct SetupFading : public NodeOperation
    {
        void operator()( osg::Node* node )
        {
            osg::Uniform* u = FadeEffect::createStartTimeUniform();
            u->set( (float)osg::Timer::instance()->time_s() );
            node->getOrCreateStateSet()->addUniform( u );
        }
    };
}


//---------------------------------------------------------------------------

FeatureModelGraph::FeatureModelGraph(Session*                         session,
                                     const FeatureModelSourceOptions& options,
                                     FeatureNodeFactory*              factory,
                                     ModelSource*                     modelSource,
                                     RefNodeOperationVector*          preMergeOperations,
                                     RefNodeOperationVector*          postMergeOperations) :
_session            ( session ),
_options            ( options ),
_factory            ( factory ),
_modelSource        ( modelSource ),
_preMergeOperations ( preMergeOperations ),
_postMergeOperations( postMergeOperations ),
_dirty              ( false ),
_pendingUpdate      ( false ),
_overlayInstalled   ( 0L ),
_overlayPlaceholder ( 0L ),
_clampable          ( 0L ),
_drapeable          ( 0L ),
_overlayChange      ( OVERLAY_NO_CHANGE )
{
    ctor();
}

FeatureModelGraph::FeatureModelGraph(Session*                         session,
                                     const FeatureModelSourceOptions& options,
                                     FeatureNodeFactory*              factory,
                                     RefNodeOperationVector*          preMergeOperations,
                                     RefNodeOperationVector*          postMergeOperations) :
_session            ( session ),
_options            ( options ),
_factory            ( factory ),
_modelSource        ( 0L ),
_preMergeOperations ( preMergeOperations ),
_postMergeOperations( postMergeOperations ),
_dirty              ( false ),
_pendingUpdate      ( false ),
_overlayInstalled   ( 0L ),
_overlayPlaceholder ( 0L ),
_clampable          ( 0L ),
_drapeable          ( 0L ),
_overlayChange      ( OVERLAY_NO_CHANGE )
{
    ctor();
}

void
FeatureModelGraph::ctor()
{
    _uid = osgEarthFeatureModelPseudoLoader::registerGraph( this );

    // an FLC that queues feature data on the high-latency thread.
    _defaultFileLocationCallback = new HighLatencyFileLocationCallback();

    // set up the callback queues for pre- and post-merge operations.

    // per-merge ops run in the pager thread:
    if ( !_preMergeOperations.valid())
        _preMergeOperations = new RefNodeOperationVector();

    // post-merge ops run in the update traversal:
    if ( !_postMergeOperations.valid() )
        _postMergeOperations = new RefNodeOperationVector();

    // install the stylesheet in the session if it doesn't already have one.
    if ( !_session->styles() )
        _session->setStyles( _options.styles().get() );

    if ( !_session->getFeatureSource() )
    {
        OE_WARN << LC << "ILLEGAL: Session must have a feature source" << std::endl;
        return;
    }

    // Set up a shared resource cache for the session. A session-wide cache means
    // that all the paging threads that load data from this FMG will load resources
    // from a single cache; e.g., once a texture is loaded in one thread, the same
    // StateSet will be used across the entire Session. That also means that StateSets
    // in the ResourceCache can potentially also be in the live graph; so you should
    // take care in dealing with them in a multi-threaded environment.
    if ( !_session->getResourceCache() && _options.sessionWideResourceCache() == true )
    {
        _session->setResourceCache( new ResourceCache(_session->getDBOptions()) );
    }
    
    // Calculate the usable extent (in both feature and map coordinates) and bounds.
    const Profile* mapProfile = _session->getMapInfo().getProfile();
    const FeatureProfile* featureProfile = _session->getFeatureSource()->getFeatureProfile();

    // Bail out if the feature profile is bad
    if ( !featureProfile || !featureProfile->getExtent().isValid() )
    {
        // warn or allow?
        return;
    }

    // the part of the feature extent that will fit on the map (in map coords):
    _usableMapExtent = mapProfile->clampAndTransformExtent( 
        featureProfile->getExtent(), 
        &_featureExtentClamped );

    // same, back into feature coords:
    _usableFeatureExtent = _usableMapExtent.transform( featureProfile->getSRS() );

    // world-space bounds of the feature layer
    _fullWorldBound = getBoundInWorldCoords( _usableMapExtent, 0L );
    
    // whether to request tiles from the source (if available). if the source is tiled, but the
    // user manually specified schema levels, don't use the tiles.
    _useTiledSource = featureProfile->getTiled();


    // compute an appropriate tileSizeFactor for a tiled source if a max range was set but no tilesize factor
    if (_options.layout().isSet() && (_options.layout()->maxRange().isSet() || _options.maxRange().isSet()))
    {
        // select the max range either from the Layout or from the model layer options.
        float userMaxRange = FLT_MAX;
        if ( _options.layout()->maxRange().isSet() )
            userMaxRange = *_options.layout()->maxRange();
        if ( _options.maxRange().isSet() )
            userMaxRange = std::min(userMaxRange, *_options.maxRange());
        
        if ( featureProfile->getTiled() )
        {
            // Cannot change the tile size of a tiled data source.
            if (_options.layout()->tileSize().isSet() )
            {
                OE_WARN << LC << getName()
                    << ": Illegal: you cannot set a tile size on a pre-tiled feature source. Ignoring.\n";
            }

            if ( !_options.layout()->tileSizeFactor().isSet() )
            {
                // So automatically compute the tileSizeFactor based on the max range
                double width, height;
                featureProfile->getProfile()->getTileDimensions(featureProfile->getFirstLevel(), width, height);

                MapFrame mapf = _session->createMapFrame();


                GeoExtent ext(featureProfile->getSRS(),
                    featureProfile->getExtent().west(),
                    featureProfile->getExtent().south(),
                    featureProfile->getExtent().west() + width,
                    featureProfile->getExtent().south() + height);
                osg::BoundingSphered bounds = getBoundInWorldCoords( ext, &mapf );

                float tileSizeFactor = userMaxRange / bounds.radius();
                //The tilesize factor must be at least 1.0 to avoid culling the tile when you are within it's bounding sphere. 
                tileSizeFactor = osg::maximum( tileSizeFactor, 1.0f);
                OE_INFO << LC << "Computed a tilesize factor of " << tileSizeFactor << " with max range setting of " <<  userMaxRange << std::endl;
                _options.layout()->tileSizeFactor() = tileSizeFactor * 1.5; // approx sqrt(2)
            }
        }
    }


    if ( _options.layout().isSet() && _options.layout()->getNumLevels() > 0 )
    {
        // the user provided a custom levels setup, so don't use the tiled source (which
        // provides its own levels setup)
        _useTiledSource = false;

        // If the user asked for a particular tile size, give it to them!
        if (_options.layout()->tileSize().isSet() &&
            _options.layout()->tileSize() > 0.0 )
        {
            float maxRange = FLT_MAX;
            maxRange = _options.maxRange().getOrUse(maxRange);
            maxRange = _options.layout()->maxRange().getOrUse(maxRange);
            maxRange = std::min( maxRange, _options.layout()->getLevel(0)->maxRange().get() );
        
            _options.layout()->tileSizeFactor() = maxRange / _options.layout()->tileSize().get();

            OE_INFO << LC << "Tile size = " << (*_options.layout()->tileSize()) << " ==> TRF = " << 
                (*_options.layout()->tileSizeFactor()) << "\n";
        }

        // for each custom level, calculate the best LOD match and store it in the level
        // layout data. We will use this information later when constructing the SG in
        // the pager.
        for( unsigned i = 0; i < _options.layout()->getNumLevels(); ++i )
        {
            const FeatureLevel* level = _options.layout()->getLevel( i );
            unsigned lod = _options.layout()->chooseLOD( *level, _fullWorldBound.radius() );
            _lodmap.resize( lod+1, 0L );
            _lodmap[lod] = level;

            OE_INFO << LC << _session->getFeatureSource()->getName() 
                << ": F.Level max=" << level->maxRange().get() << ", min=" << level->minRange().get()
                << ", LOD=" << lod
                << std::endl;
        }
    }


    // Compute the feature levels up front for tiled sources.
    if (featureProfile->getTiled() && _useTiledSource)
    {    
        // Get the max range of the root level
        MapFrame mapf = _session->createMapFrame();
        osg::BoundingSphered bounds = getBoundInWorldCoords( featureProfile->getExtent(), &mapf );
        double maxRange = bounds.radius() * *_options.layout()->tileSizeFactor();

        _lodmap.resize(featureProfile->getMaxLevel() + 1);
         
        // Compute the max range of all the feature levels.  Each subsequent level if half of the parent.
        for (unsigned int i = 0; i < featureProfile->getMaxLevel()+1; i++)
        {
            OE_INFO << LC << "Computed max range " << maxRange << " for lod " << i << std::endl;
            FeatureLevel* level = new FeatureLevel(0.0, maxRange);
            _lodmap[i] = level;
            maxRange /= 2.0;
        }
    }

    // Apply some default state. The options properties let you override the
    // defaults, but we'll set some reasonable state if they are not set.

    osg::StateSet* stateSet = getOrCreateStateSet();

    // Set up backface culling. If the option is unset, enable it by default
    // since shadowing requires it and it's a decent general-purpose setting
    if ( _options.backfaceCulling().isSet() )
        stateSet->setMode( GL_CULL_FACE, *_options.backfaceCulling() ? 1 : 0 );
    else
        stateSet->setMode( GL_CULL_FACE, 1 );

    // Set up alpha blending. Enable it by default if not specified.
    if ( _options.alphaBlending().isSet() )
        stateSet->setMode( GL_BLEND, *_options.alphaBlending() ? 1 : 0 );
    else
        stateSet->setMode( GL_BLEND, 1 );

    // Set up lighting, only if the option is set
    if ( _options.enableLighting().isSet() )
        stateSet->setMode( GL_LIGHTING, *_options.enableLighting() ? 1 : 0 );

    // If the user requests fade-in, install a post-merge operation that will set the 
    // proper fade time for paged nodes.
    if ( _options.fading().isSet() )
    {
        _postMergeOperations->mutex().writeLock();
        _postMergeOperations->push_back( new SetupFading() );
        _postMergeOperations->mutex().writeUnlock();
        OE_INFO << LC << "Added fading post-merge operation" << std::endl;
    }

    ADJUST_EVENT_TRAV_COUNT( this, 1 );

    redraw();
}

FeatureModelGraph::~FeatureModelGraph()
{
    osgEarthFeatureModelPseudoLoader::unregisterGraph( _uid );
}

void
FeatureModelGraph::dirty()
{
    _dirty = true;
}

std::ostream& operator << (std::ostream& in, const osg::Vec3d& v) { in << v.x() << ", " << v.y() << ", " << v.z(); return in; }

osg::BoundingSphered
FeatureModelGraph::getBoundInWorldCoords(const GeoExtent& extent,
                                         const MapFrame*  mapf ) const
{
    osg::Vec3d center, corner;
    GeoExtent workingExtent;

    if ( !extent.isValid() )
    {
        return osg::BoundingSphered();
    }

    if ( extent.getSRS()->isEquivalentTo( _usableMapExtent.getSRS() ) )
    {
        workingExtent = extent;
    }
    else
    {
        workingExtent = extent.transform( _usableMapExtent.getSRS() ); // safe.
    }

    workingExtent.getCentroid( center.x(), center.y() );
    
    double centerZ = 0.0;    
    if ( mapf )
    {
        // Use an appropriate resolution for this extents width
        double resolution = workingExtent.width();
        ElevationQuery query( *mapf );
        query.setFallBackOnNoData( true );
        GeoPoint p( mapf->getProfile()->getSRS(), center, ALTMODE_ABSOLUTE );
        query.getElevation( p, center.z(), resolution );
        centerZ = center.z();
    }    

    corner.x() = workingExtent.xMin();
    corner.y() = workingExtent.yMin();
    corner.z() = 0;

    if ( _session->getMapInfo().isGeocentric() )
    {
#if 0
        // Convert the extent to lat/long and center it on the equator; this will ensure
        // that all tiles in the same LOD have the same bounding radius.
        GeoExtent eq = workingExtent.transform( workingExtent.getSRS()->getGeographicSRS() );

        GeoExtent equatorialExtent(
            eq.getSRS(),
            eq.west(),
            -eq.height()/2.0,
            eq.east(),
            eq.height()/2.0 );
        
        GeoPoint centerPoint( workingExtent.getSRS(), center, ALTMODE_ABSOLUTE );
        centerPoint.toWorld( center );

        return osg::BoundingSphered( center, equatorialExtent.getBoundingGeoCircle().getRadius() );
#else
        
        /*
        GeoPoint centerPoint( workingExtent.getSRS(), center, ALTMODE_ABSOLUTE );
        centerPoint.toWorld( center );
        double radius = workingExtent.getBoundingGeoCircle().getRadius();
        //OE_WARN << LC << "Extent=" << workingExtent.toString() << "; center=" << center << "; radius=" << radius << "\n";
        return osg::BoundingSphered( center, radius );  
        */

        // Compute the bounding sphere by sampling points along the extent.
        int samples = 6;

        double xSample = workingExtent.width() / (double)samples;
        double ySample = workingExtent.height() / (double)samples;

        osg::BoundingSphered bs;
        for (unsigned int c = 0; c < samples+1; c++)
        {
            double x = workingExtent.xMin() + (double)c * xSample;
            for (unsigned int r = 0; r < samples+1; r++)
            {
                double y = workingExtent.yMin() + (double)r * ySample;
                osg::Vec3d world;
                GeoPoint(workingExtent.getSRS(), x, y, 0, ALTMODE_ABSOLUTE).toWorld(world);
                bs.expandBy(world);
            }
        }
        return bs;
              

        /*
        // Compute the bounding sphere by sampling the corners.
        osg::Vec3d sw, se, ne, nw, e, w, s, n;
        GeoPoint(workingExtent.getSRS(), workingExtent.west(), workingExtent.south(), 0, ALTMODE_ABSOLUTE).toWorld(sw);
        GeoPoint(workingExtent.getSRS(), workingExtent.east(), workingExtent.south(), 0, ALTMODE_ABSOLUTE).toWorld(se);
        GeoPoint(workingExtent.getSRS(), workingExtent.east(), workingExtent.north(), 0, ALTMODE_ABSOLUTE).toWorld(ne);
        GeoPoint(workingExtent.getSRS(), workingExtent.west(), workingExtent.north(), 0, ALTMODE_ABSOLUTE).toWorld(nw);
        GeoPoint(workingExtent.getSRS(), workingExtent.west(), center.y(),            0, ALTMODE_ABSOLUTE).toWorld(w);
        GeoPoint(workingExtent.getSRS(), workingExtent.east(), center.y(),            0, ALTMODE_ABSOLUTE).toWorld(e);
        GeoPoint(workingExtent.getSRS(), center.x(),           workingExtent.north(), 0, ALTMODE_ABSOLUTE).toWorld(n);
        GeoPoint(workingExtent.getSRS(), center.x(),           workingExtent.south(), 0, ALTMODE_ABSOLUTE).toWorld(s);
      
        osg::BoundingSphered bs;
        bs.expandBy(center);
        bs.expandBy(sw);
        bs.expandBy(se);
        bs.expandBy(ne);
        bs.expandBy(nw);
        bs.expandBy(w);
        bs.expandBy(e);
        bs.expandBy(n);
        bs.expandBy(s);
       
        */ 
#endif
    }

    if (workingExtent.getSRS()->isGeographic() &&
        ( workingExtent.width() >= 90 || workingExtent.height() >= 90 ) )
    {
        return osg::BoundingSphered( osg::Vec3d(0,0,0), 2*center.length() );
    }

    return osg::BoundingSphered( center, (center-corner).length() );
}

osg::Node*
FeatureModelGraph::setupPaging()
{
    // calculate the bounds of the full data extent:
    MapFrame mapf = _session->createMapFrame();
    osg::BoundingSphered bs = getBoundInWorldCoords( _usableMapExtent, &mapf );

    const FeatureProfile* featureProfile = _session->getFeatureSource()->getFeatureProfile();

    optional<float> maxRangeOverride;

    if (_options.layout()->maxRange().isSet() || _options.maxRange().isSet())
    {
        // select the max range either from the Layout or from the model layer options.
        float userMaxRange = FLT_MAX;
        if ( _options.layout()->maxRange().isSet() )
            userMaxRange = *_options.layout()->maxRange();
        if ( _options.maxRange().isSet() )
            userMaxRange = std::min(userMaxRange, *_options.maxRange());
        
        if ( !featureProfile->getTiled() )
        {
            // user set a max_range, but we'd not tiled. Just override the top level plod.
            maxRangeOverride = userMaxRange;
        }
    }

    // calculate the max range for the top-level PLOD:
    // TODO: a user-specified maxRange is actually an altitude, so this is not
    //       strictly correct anymore!
    float maxRange = 
        maxRangeOverride.isSet() ? *maxRangeOverride :
        bs.radius() * _options.layout()->tileSizeFactor().value();

    // build the URI for the top-level paged LOD:
    std::string uri = s_makeURI( _uid, 0, 0, 0 );

    // bulid the top level Paged LOD:
    osg::Group* pagedNode = createPagedNode( 
        bs, 
        uri, 
        0.0f, 
        maxRange, 
        *_options.layout()->priorityOffset(), 
        *_options.layout()->priorityScale(),
        _postMergeOperations.get(),
        _defaultFileLocationCallback.get() );

    return pagedNode;
}


/**
 * Called by the pseudo-loader, this method attempts to load a single tile of features.
 */
osg::Node*
FeatureModelGraph::load( unsigned lod, unsigned tileX, unsigned tileY, const std::string& uri )
{
    OE_DEBUG << LC
        << "load: " << lod << "_" << tileX << "_" << tileY << std::endl;

    osg::Group* result = 0L;
    
    if ( _useTiledSource )
    {       
        // A "tiled" source has a pre-generted tile hierarchy, but no range information.
        // We will calcluate the LOD ranges here, as a function of the tile radius and the
        // "tile size factor" ... see below.
        osg::Group* geometry =0L;
        const FeatureProfile* featureProfile = _session->getFeatureSource()->getFeatureProfile();

        if ( (int)lod >= featureProfile->getFirstLevel() )
        {
            // The extent of this tile:
            GeoExtent tileExtent = s_getTileExtent( lod, tileX, tileY, _usableFeatureExtent );

            // Calculate the bounds of this new tile:
            MapFrame mapf = _session->createMapFrame();
            osg::BoundingSphered tileBound = getBoundInWorldCoords( tileExtent, &mapf );

            // Apply the tile range multiplier to calculate a max camera range. The max range is
            // the geographic radius of the tile times the multiplier.
            float tileFactor = _options.layout().isSet() ? _options.layout()->tileSizeFactor().get() : 15.0f;            
            double maxRange =  tileBound.radius() * tileFactor;
            FeatureLevel level( 0, maxRange );
            //OE_NOTICE << "(" << lod << ": " << tileX << ", " << tileY << ")" << std::endl;
            //OE_NOTICE << "  extent = " << tileExtent.width() << "x" << tileExtent.height() << std::endl;
            //OE_NOTICE << "  tileFactor = " << tileFactor << " maxRange=" << maxRange << " radius=" << tileBound.radius() << std::endl;
            
            // Construct a tile key that will be used to query the source for this tile.
#if 0
            unsigned int w, h;
            featureProfile->getProfile()->getNumTiles(lod, w, h);
            tileY = h - tileY - 1;
#endif

            TileKey key(lod, tileX, tileY, featureProfile->getProfile());
            geometry = buildLevel( level, tileExtent, &key );
            result = geometry;
        }

        // check whether more levels exist below the current level.
        if ( (int)lod < featureProfile->getMaxLevel() )
        {
            // yes, so build some pagedlods to bring in the next level.
            osg::ref_ptr<osg::Group> group = new osg::Group();

            // calculate the LOD of the next level:
            if ( lod+1 != ~0 )
            {
                // only build sub-pagedlods if we are expecting subtiles at some point:
                if ( geometry != 0L || (int)lod < featureProfile->getFirstLevel() )
                {
                    MapFrame mapf = _session->createMapFrame();
                    buildSubTilePagedLODs( lod, tileX, tileY, &mapf, group.get() );
                    group->addChild( geometry );
                }

                result = group.release();
            }   
        }
    }

    else if ( !_options.layout().isSet() || _options.layout()->getNumLevels() == 0 )
    {
        // This is a non-tiled data source that has NO level details. In this case, 
        // we simply want to load all features at once and make them visible at
        // maximum camera range.

        FeatureLevel all( 0.0f, FLT_MAX );
        result = buildLevel( all, GeoExtent::INVALID, 0 );
    }

    else if ( (int)lod < _lodmap.size() )
    {
        // This path computes the SG for a model graph with explicity-defined levels of
        // detail. We already calculated the LOD level map in setupPaging(). If the
        // current LOD points to an actual FeatureLevel, we build the geometry for that
        // level in the tile.

        osg::Group* geometry = 0L;
        const FeatureLevel* level = _lodmap[lod];
        if ( level )
        {
            // There exists a real data level at this LOD. So build the geometry that will
            // represent this tile.
            GeoExtent tileExtent = 
                lod > 0 ?
                s_getTileExtent( lod, tileX, tileY, _usableFeatureExtent ) :
                _usableFeatureExtent;
                
            geometry = buildLevel( *level, tileExtent, 0 );
            result = geometry;
        }

        if ( lod < _lodmap.size()-1 )
        {
            // There are more populated levels below this one. So build the subtile
            // PagedLODs that will load them.
            osg::ref_ptr<osg::Group> group = new osg::Group();

            MapFrame mapf = _session->createMapFrame();
            buildSubTilePagedLODs( lod, tileX, tileY, &mapf, group.get() );

            if ( geometry )
                group->addChild( geometry );

            result = group.release();
        }
    }

    if ( !result )
    {
        // If the read resulting in nothing, create an empty group so that the read
        // (technically) succeeds and the pager won't try to load the null child
        // over and over.
        result = new osg::Group();
    }
    else
    {
        // For some unknown reason, this breaks when I insert an LOD. -gw
        //RemoveEmptyGroupsVisitor::run( result );
    }

    if ( result->getNumChildren() == 0 )
    {
        // if the result group contains no data, blacklist it so we never try to load it again.
        Threading::ScopedWriteLock exclusiveLock( _blacklistMutex );
        _blacklist.insert( uri );
        OE_DEBUG << LC << "Blacklisting: " << uri << std::endl;
    }

    // Done - run the pre-merge operations.
    runPreMergeOperations(result);

    return result;
}


void
FeatureModelGraph::buildSubTilePagedLODs(unsigned        parentLOD,
                                         unsigned        parentTileX,
                                         unsigned        parentTileY,
                                         const MapFrame* mapf,
                                         osg::Group*     parent)
{
    unsigned subtileLOD = parentLOD + 1;
    unsigned subtileX = parentTileX * 2;
    unsigned subtileY = parentTileY * 2;

    
    // Find the next level with data:
    const FeatureLevel* flevel = 0L;
    
    for(unsigned lod=subtileLOD; lod<_lodmap.size() && !flevel; ++lod)
    {
        flevel = _lodmap[lod];
    }

    // should not happen (or this method would never have been called in teh first place) but
    // check anyway.
    if ( !flevel )
    {
        OE_INFO << LC << "INTERNAL: buildSubTilePagedLODs called but no further levels exist\n";
        return;
    }
    
    // make a paged LOD for each subtile:
    for( unsigned u = subtileX; u <= subtileX + 1; ++u )
    {
        for( unsigned v = subtileY; v <= subtileY + 1; ++v )
        {
            GeoExtent subtileFeatureExtent = s_getTileExtent( subtileLOD, u, v, _usableFeatureExtent );
            osg::BoundingSphered subtile_bs = getBoundInWorldCoords( subtileFeatureExtent, mapf );
      
            // Calculate the maximum camera range for the LOD.
            float maxRange;

            
            if ( flevel && flevel->maxRange().isSet() )
            {
                // User set it expressly
                maxRange = flevel->maxRange().get();
                if ( maxRange < FLT_MAX )
                    maxRange += subtile_bs.radius();
            }
            
            else
            {
                // Calculate it based on the tile size factor.
                maxRange = subtile_bs.radius() * _options.layout()->tileSizeFactor().value();
            }

            std::string uri = s_makeURI( _uid, subtileLOD, u, v );

            // check the blacklist to make sure we haven't unsuccessfully tried
            // this URI before
            bool blacklisted = false;
            {
                Threading::ScopedReadLock sharedLock( _blacklistMutex );
                blacklisted = _blacklist.find( uri ) != _blacklist.end();
            }

            if ( !blacklisted )
            {
                OE_DEBUG << LC << "    " << uri
                    << std::fixed
                    << "; center = " << subtile_bs.center().x() << "," << subtile_bs.center().y() << "," << subtile_bs.center().z()
                    << "; radius = " << subtile_bs.radius()
                    << "; maxrange = " << maxRange
                    << std::endl;

                osg::Group* pagedNode = createPagedNode( 
                    subtile_bs, 
                    uri, 
                    0.0f, maxRange, 
                    *_options.layout()->priorityOffset(), 
                    *_options.layout()->priorityScale(),
                    _postMergeOperations.get(),
                    _defaultFileLocationCallback.get() );

                parent->addChild( pagedNode );
            }
        }
    }
}

/**
 * Builds geometry for feature data at a particular level, and constrained by an extent.
 * The extent is either (a) expressed in "extent" literally, as is the case in a non-tiled
 * data source, or (b) expressed implicitly by a TileKey, which is the case for a tiled
 * data source.
 */
osg::Group*
FeatureModelGraph::buildLevel( const FeatureLevel& level, const GeoExtent& extent, const TileKey* key )
{
    // set up for feature indexing if appropriate:
    osg::ref_ptr<osg::Group> group;
    FeatureSourceIndexNode* index = 0L;

    FeatureSource* featureSource = _session->getFeatureSource();

    if (featureSource)
    {
        const FeatureProfile* fp = featureSource->getFeatureProfile();

        if ( _featureIndex.valid() )
        {
            index = new FeatureSourceIndexNode( _featureIndex.get() );
            group = index;
        }
    }

    if ( !group.valid() )
    {
        group = new osg::Group();
    }

    // form the baseline query, which does a spatial query based on the working extent.
    Query query;
    if ( extent.isValid() )
        query.bounds() = extent.bounds();

    // add a tile key to the query if there is one, to support TFS-style queries
    if ( key )
        query.tileKey() = *key;

    query.setMap( _session->getMap() );

    // does the level have a style name set?
    if ( level.styleName().isSet() )
    {
        osg::Node* node = 0L;
        const Style* style = _session->styles()->getStyle( *level.styleName(), false );
        if ( style )
        {
            // found a specific style to use.
            node = createStyleGroup( *style, query, index );
            if ( node )
                group->addChild( node );
        }
        else
        {
            const StyleSelector* selector = _session->styles()->getSelector( *level.styleName() );
            if ( selector )
            {
                buildStyleGroups( selector, query, index, group.get() );
            }
        }
    }

    else
    {
        Style defaultStyle;

        if ( _session->styles()->selectors().size() == 0 )
        {
            // attempt to glean the style from the feature source name:
            defaultStyle = *_session->styles()->getStyle( 
                *_session->getFeatureSource()->getFeatureSourceOptions().name() );
        }

        osg::Node* node = build( defaultStyle, query, extent, index );
        if ( node )
            group->addChild( node );
    }

    if ( group->getNumChildren() > 0 )
    {
        // account for a min-range here. Do not address the max-range here; that happens
        // above when generating paged LOD nodes, etc.
        float minRange = level.minRange().get();
        if ( minRange > 0.0f )
        {
            ElevationLOD* lod = new ElevationLOD( _session->getMapSRS() );
            lod->setMinElevation( minRange );
            lod->addChild( group.get() );
            group = lod;
        }

        // install a cluster culler.
        if ( _session->getMapInfo().isGeocentric() && _options.clusterCulling() == true )
        {
            const FeatureProfile* featureProfile = _session->getFeatureSource()->getFeatureProfile();
            const GeoExtent& ccExtent = extent.isValid() ? extent : featureProfile->getExtent();
            if ( ccExtent.isValid() )
            {
                // if the extent is more than 90 degrees, bail
                GeoExtent geodeticExtent = ccExtent.transform( ccExtent.getSRS()->getGeographicSRS() );
                if ( geodeticExtent.width() < 90.0 && geodeticExtent.height() < 90.0 )
                {
                    // get the geocentric tile center:
                    osg::Vec3d tileCenter;
                    ccExtent.getCentroid( tileCenter.x(), tileCenter.y() );

                    osg::Vec3d centerECEF;
                    ccExtent.getSRS()->transform( tileCenter, _session->getMapSRS()->getECEF(), centerECEF );
                    //ccExtent.getSRS()->transformToECEF( tileCenter, centerECEF );

                    osg::NodeCallback* ccc = ClusterCullingFactory::create2( group.get(), centerECEF );
                    if ( ccc )
                        group->addCullCallback( ccc );
                }
            }
        }

        return group.release();
    }

    else
    {
        return 0L;
    }
}


osg::Group*
FeatureModelGraph::build(const Style&         defaultStyle, 
                         const Query&         baseQuery, 
                         const GeoExtent&     workingExtent,
                         FeatureIndexBuilder* index)
{
    osg::ref_ptr<osg::Group> group = new osg::Group();

    FeatureSource* source = _session->getFeatureSource();

    // case: each feature has an embedded style.
    if ( source->hasEmbeddedStyles() )
    {
        const FeatureProfile* featureProfile = source->getFeatureProfile();

        // each feature has its own style, so use that and ignore the style catalog.
        osg::ref_ptr<FeatureCursor> cursor = source->createFeatureCursor( baseQuery );

        while( cursor.valid() && cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );
                osg::ref_ptr<FeatureCursor> cursor = new FeatureListCursor(list);

                FilterContext context( _session.get(), featureProfile, workingExtent, index );

                // note: gridding is not supported for embedded styles.
                osg::ref_ptr<osg::Node> node;

                // Get the Group that parents all features of this particular style. Note, this
                // might be NULL if the factory does not support style groups.
                osg::Group* styleGroup = getOrCreateStyleGroupFromFactory( *feature->style() );
                if ( styleGroup )
                {
                    if ( !group->containsNode( styleGroup ) )
                    {
                        group->addChild( styleGroup );
                    }
                }

                if ( _factory->createOrUpdateNode( cursor.get(), *feature->style(), context, node ) )
                {
                    if ( node.valid() )
                    {
                        if ( styleGroup )
                            styleGroup->addChild( node.get() );
                        else
                            group->addChild( node.get() );
                    }
                }
            }
        }
    }

    // case: features are externally styled.
    else
    {
        const StyleSheet* styles = _session->styles();

        // if the stylesheet has selectors, use them to sort the features into style groups. Then create
        // a create a node for each style group.
        if ( styles->selectors().size() > 0 )
        {
            for( StyleSelectorList::const_iterator i = styles->selectors().begin(); i != styles->selectors().end(); ++i )
            {
                // pull the selected style...
                const StyleSelector& sel = *i;

                // if the selector uses an expression to select the style name, then we must perform the
                // query and then SORT the features into style groups.
                if ( sel.styleExpression().isSet() )
                {
                    // merge the selector's query into the existing query
                    Query combinedQuery = baseQuery.combineWith( *sel.query() );
                    combinedQuery.setMap( _session->getMap() );

                    // query, sort, and add each style group to th parent:
                    queryAndSortIntoStyleGroups( combinedQuery, *sel.styleExpression(), index, group );
                }

                // otherwise, all feature returned by this query will have the same style:
                else if ( !_useTiledSource )
                {
                    // combine the selection style with the incoming base style:
                    Style selectedStyle = *styles->getStyle( sel.getSelectedStyleName() );
                    Style combinedStyle = defaultStyle.combineWith( selectedStyle );

                    // .. and merge it's query into the existing query
                    Query combinedQuery = baseQuery.combineWith( *sel.query() );
                    combinedQuery.setMap( _session->getMap() );

                    // then create the node.
                    osg::Group* styleGroup = createStyleGroup( combinedStyle, combinedQuery, index );

                    if ( styleGroup && !group->containsNode(styleGroup) )
                        group->addChild( styleGroup );
                }

                // Tried to apply a selector query to a tiled source, which is illegal because
                // you cannot run an SQL expression on pre-tiled data (like TFS).
                else
                {
                    OE_WARN << LC 
                        << "Illegal: you cannot use a selector SQL query with a tiled feature source. "
                        << "Consider using a JavaScript style expression instead."
                        << std::endl;
                }
            }
        }

        // if no selectors are present, render all the features with a single style.
        else
        {
            Style combinedStyle = defaultStyle;

            // if there's no base style defined, choose a "default" style from the stylesheet.
            if ( defaultStyle.empty() )
                combinedStyle = *styles->getDefaultStyle();

            osg::Group* styleGroup = createStyleGroup( combinedStyle, baseQuery, index );

            if ( styleGroup && !group->containsNode(styleGroup) )
                group->addChild( styleGroup );
        }
    }

    return group->getNumChildren() > 0 ? group.release() : 0L;
}


/**
 * Builds a collection of style groups by processing a StyleSelector.
 */
void
FeatureModelGraph::buildStyleGroups(const StyleSelector* selector,
                                    const Query&         baseQuery,
                                    FeatureIndexBuilder* index,
                                    osg::Group*          parent)
{
    OE_TEST << LC << "buildStyleGroups: " << selector->name() << std::endl;

    // if the selector uses an expression to select the style name, then we must perform the
    // query and then SORT the features into style groups.
    if ( selector->styleExpression().isSet() )
    {
        // merge the selector's query into the existing query
        Query combinedQuery = baseQuery.combineWith( *selector->query() );
        combinedQuery.setMap( _session->getMap() );

        // query, sort, and add each style group to the parent:
        queryAndSortIntoStyleGroups( combinedQuery, *selector->styleExpression(), index, parent );
    }

    // otherwise, all feature returned by this query will have the same style:
    else
    {
        // combine the selection style with the incoming base style:
        const Style* selectedStyle = _session->styles()->getStyle(selector->getSelectedStyleName());
        Style style;
        if ( selectedStyle )
            style = *selectedStyle;

        // .. and merge it's query into the existing query
        Query combinedQuery = baseQuery.combineWith( *selector->query() );
        combinedQuery.setMap( _session->getMap() );

        // then create the node.
        osg::Node* node = createStyleGroup( style, combinedQuery, index );
        if ( node && !parent->containsNode(node) )
            parent->addChild( node );
    }
}


/**
 * Querys the feature source;
 * Visits each feature and uses the Style Expression to resolve its style class;
 * Sorts the features into bins based on style class;
 * Compiles each bin into a separate style group;
 * Adds the resulting style groups to the provided parent.
 */
void
FeatureModelGraph::queryAndSortIntoStyleGroups(const Query&            query,
                                               const StringExpression& styleExpr,
                                               FeatureIndexBuilder*    index,
                                               osg::Group*             parent)
{
    // the profile of the features
    const FeatureProfile* featureProfile = _session->getFeatureSource()->getFeatureProfile();

    // get the extent of the full set of feature data:
    const GeoExtent& extent = featureProfile->getExtent();
    
    // query the feature source:
    osg::ref_ptr<FeatureCursor> cursor = _session->getFeatureSource()->createFeatureCursor( query );
    if ( !cursor.valid() )
        return;

    // establish the working bounds and a context:
    Bounds bounds = query.bounds().isSet() ? *query.bounds() : extent.bounds();
    FilterContext context( _session.get(), featureProfile, GeoExtent(featureProfile->getSRS(), bounds), index );
    StringExpression styleExprCopy( styleExpr );

    // visit each feature and run the expression to sort it into a bin.
    std::map<std::string, FeatureList> styleBins;
    while( cursor->hasMore() )
    {
        osg::ref_ptr<Feature> feature = cursor->nextFeature();
        if ( feature.valid() )
        {
            const std::string& styleString = feature->eval( styleExprCopy, &context );
            styleBins[styleString].push_back( feature.get() );
        }
    }

    // next create a style group per bin.
    for( std::map<std::string,FeatureList>::iterator i = styleBins.begin(); i != styleBins.end(); ++i )
    {
        const std::string& styleString = i->first;
        FeatureList&       workingSet  = i->second;

        // resolve the style:
        Style combinedStyle;

        // if the style string begins with an open bracket, it's an inline style definition.
        if ( styleString.length() > 0 && styleString.at(0) == '{' )
        {
            Config conf( "style", styleString );
            conf.setReferrer( styleExpr.uriContext().referrer() );
            conf.set( "type", "text/css" );
            combinedStyle = Style(conf);
        }

        // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
        // style in this case: for style expressions, the user must be explicity about 
        // default styling; this is because there is no other way to exclude unwanted
        // features.
        else
        {
            const Style* selectedStyle = _session->styles()->getStyle(styleString, false);
            if ( selectedStyle )
                combinedStyle = *selectedStyle;
        }

        // if there is a valid style, create the node and add it. (Otherwise we will skip
        // the feature.)
        if ( !combinedStyle.empty() )
        {
            osg::Group* styleGroup = createStyleGroup(combinedStyle, workingSet, context);
            if ( styleGroup )
                parent->addChild( styleGroup );
        }
    }
}


osg::Group*
FeatureModelGraph::createStyleGroup(const Style&         style, 
                                    FeatureList&         workingSet, 
                                    const FilterContext& contextPrototype)
{
    osg::Group* styleGroup = 0L;

    OE_DEBUG << LC << "Created style group \"" << style.getName() << "\"\n";

    FilterContext context(contextPrototype);

    // First Crop the feature set to the working extent.
    // Note: There is an obscure edge case that can happen is a feature's centroid
    // falls exactly on the crop extent boundary. In that case the feature can
    // show up in more than one tile. It's rare and not trivial to mitigate so for now
    // we have decided to do nothing. :)
    CropFilter crop( 
        _options.layout().isSet() && _options.layout()->cropFeatures() == true ? 
        CropFilter::METHOD_CROPPING : CropFilter::METHOD_CENTROID );

    unsigned sizeBefore = workingSet.size();

    context = crop.push( workingSet, context );

    unsigned sizeAfter = workingSet.size();

    OE_DEBUG << LC << "Cropped out " << sizeBefore-sizeAfter << " features\n";

    // next, if the usable extent is less than the full extent (i.e. we had to clamp the feature
    // extent to fit on the map), calculate the extent of the features in this tile and 
    // crop to the map extent if necessary. (Note, if cropFeatures was set to true, this is
    // already done)
    if ( _featureExtentClamped && _options.layout().isSet() && _options.layout()->cropFeatures() == false )
    {
        context.extent() = _usableFeatureExtent;
        CropFilter crop2( CropFilter::METHOD_CROPPING );
        context = crop2.push( workingSet, context );
    }

    // finally, compile the features into a node.
    if ( workingSet.size() > 0 )
    {
        osg::ref_ptr<osg::Node> node;
        osg::ref_ptr<FeatureCursor> newCursor = new FeatureListCursor(workingSet);

        if ( _factory->createOrUpdateNode( newCursor.get(), style, context, node ) )
        {
            if ( !styleGroup )
                styleGroup = getOrCreateStyleGroupFromFactory( style );

            // if it returned a node, add it. (it doesn't necessarily have to)
            if ( node.valid() )
                styleGroup->addChild( node.get() );
        }
    }

    return styleGroup;
}


osg::Group*
FeatureModelGraph::createStyleGroup(const Style&         style, 
                                    const Query&         query, 
                                    FeatureIndexBuilder* index)
{
    osg::Group* styleGroup = 0L;

    // the profile of the features
    const FeatureProfile* featureProfile = _session->getFeatureSource()->getFeatureProfile();

    // get the extent of the full set of feature data:
    const GeoExtent& extent = featureProfile->getExtent();
    
    // query the feature source:
    osg::ref_ptr<FeatureCursor> cursor = _session->getFeatureSource()->createFeatureCursor( query );

    if ( cursor.valid() && cursor->hasMore() )
    {
        Bounds cellBounds =
            query.bounds().isSet() ? *query.bounds() : extent.bounds();

        FilterContext context( _session.get(), featureProfile, GeoExtent(featureProfile->getSRS(), cellBounds), index );

        // start by culling our feature list to the working extent. By default, this is done by
        // checking feature centroids. But the user can override this to crop feature geometry to
        // the cell boundaries.
        FeatureList workingSet;
        cursor->fill( workingSet );

        styleGroup = createStyleGroup(style, workingSet, context);
    }


    return styleGroup;
}


void
FeatureModelGraph::checkForGlobalStyles( const Style& style )
{
    const AltitudeSymbol* alt = style.get<AltitudeSymbol>();
    if ( alt )
    {
        if (alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN || 
            alt->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN)
        {
            if ( alt->technique() == AltitudeSymbol::TECHNIQUE_GPU && !_clampable )
            {
                _clampable = new ClampableNode( 0L );
                _overlayChange = OVERLAY_INSTALL_CLAMPABLE;
            }

            else if ( alt->technique() == AltitudeSymbol::TECHNIQUE_DRAPE && !_drapeable )
            {
                _drapeable = new DrapeableNode( 0L );
                _overlayChange = OVERLAY_INSTALL_DRAPEABLE;
            }
        }
    }
    
    const RenderSymbol* render = style.get<RenderSymbol>();

    if ( _clampable )
    {
        // if we're using extrusion, don't perform depth offsetting:
        const ExtrusionSymbol* extrusion = style.get<ExtrusionSymbol>();
        if ( extrusion )
        {
            DepthOffsetOptions d = _clampable->getDepthOffsetOptions();
            d.enabled() = false;
            _clampable->setDepthOffsetOptions( d );
        }

        // check for explicit depth offset render settings (note, this could
        // override the automatic disable put in place by the presence of an
        // ExtrusionSymbol above)
        if ( render && render->depthOffset().isSet() )
        {
            _clampable->setDepthOffsetOptions(*render->depthOffset());
        }
    }

    else 
    {
        if ( render && render->depthOffset().isSet() )
        {
            _depthOffsetAdapter.setGraph( this );
            _depthOffsetAdapter.setDepthOffsetOptions( *render->depthOffset() );
        }

        // apply render order when draping:
        if ( _drapeable && render && render->order().isSet() )
        {
            _drapeable->setRenderOrder( render->order()->eval() );
        }

        if ( render && render->renderBin().isSet() )
        {
            osg::StateSet* ss = getOrCreateStateSet();
            ss->setRenderBinDetails(ss->getBinNumber(), render->renderBin().get());
        }
    }
}


osg::Group*
FeatureModelGraph::getOrCreateStyleGroupFromFactory(const Style& style)
{
    osg::Group* styleGroup = _factory->getOrCreateStyleGroup( style, _session.get() );

    // Check the style and see if we need to active GPU clamping. GPU clamping
    // is currently all-or-nothing for a single FMG.
    checkForGlobalStyles( style );

    return styleGroup;
}


void
FeatureModelGraph::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.EVENT_VISITOR )
    {
        if (!_pendingUpdate && 
             (_dirty ||
              _session->getFeatureSource()->outOfSyncWith(_featureSourceRev) ||
              (_modelSource.valid() && _modelSource->outOfSyncWith(_modelSourceRev))))
        {
            _pendingUpdate = true;
            ADJUST_UPDATE_TRAV_COUNT( this, 1 );
        }

        else if ( _overlayChange != OVERLAY_NO_CHANGE )
        {
            ADJUST_UPDATE_TRAV_COUNT( this, 1 );
        }
    }

    else if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        if ( _pendingUpdate )
        {
            redraw();
            _pendingUpdate = false;
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        }

        else if ( _overlayChange != OVERLAY_NO_CHANGE )
        {
            changeOverlay();
            _overlayChange = OVERLAY_NO_CHANGE;
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        }
    }

    osg::Group::traverse(nv);
}

void
FeatureModelGraph::runPreMergeOperations(osg::Node* node)
{
   if ( _preMergeOperations.valid() )
   {
      _preMergeOperations->mutex().readLock();
      for( NodeOperationVector::iterator i = _preMergeOperations->begin(); i != _preMergeOperations->end(); ++i )
      {
         i->get()->operator()( node );
      }
      _preMergeOperations->mutex().readUnlock();
   }
}

void
FeatureModelGraph::runPostMergeOperations(osg::Node* node)
{
   if ( _postMergeOperations.valid() )
   {
      _postMergeOperations->mutex().readLock();
      for( NodeOperationVector::iterator i = _postMergeOperations->begin(); i != _postMergeOperations->end(); ++i )
      {
         i->get()->operator()( node );
      }
      _postMergeOperations->mutex().readUnlock();
   }
}


void
FeatureModelGraph::changeOverlay()
{
    if (_overlayChange == OVERLAY_INSTALL_CLAMPABLE &&
        _clampable                                  && 
        _clampable != _overlayInstalled )
    {
        runPostMergeOperations( _clampable );
        osgEarth::replaceGroup( _overlayInstalled, _clampable );
        _overlayInstalled   = _clampable;
        _drapeable          = 0L;
        _overlayPlaceholder = 0L;
        OE_INFO << LC << "Installed clampable decorator on layer " << getName() << std::endl;
    }

    else if (
        _overlayChange == OVERLAY_INSTALL_DRAPEABLE && 
        _drapeable                                  && 
        _drapeable != _overlayInstalled )
    {
        runPostMergeOperations( _drapeable );
        osgEarth::replaceGroup( _overlayInstalled, _drapeable );
        _overlayInstalled   = _drapeable;
        _overlayPlaceholder = 0L;
        _clampable          = 0L;
        OE_INFO << LC << "Installed drapeable decorator on layer " << getName() << std::endl;
    }

    else if (
        _overlayChange == OVERLAY_INSTALL_PLACEHOLDER && 
        _overlayPlaceholder                           && 
        _overlayPlaceholder != _overlayInstalled)
    {
        runPostMergeOperations( _overlayPlaceholder );
        osgEarth::replaceGroup( _overlayInstalled, _overlayPlaceholder );
        _overlayInstalled = _overlayPlaceholder;
        _clampable        = 0L;
        _drapeable        = 0L;
        OE_INFO << LC << "Installed null decorator on layer " << getName() << std::endl;
    }
}


void
FeatureModelGraph::redraw()
{
    // clear it out
    removeChildren( 0, getNumChildren() );

    // initialize the index if necessary.
    if ( _options.featureIndexing()->enabled() == true )
    {
        _featureIndex = new FeatureSourceIndex(
            _session->getFeatureSource(),
            Registry::objectIndex(),
            _options.featureIndexing().get() );
    }

    // zero out any decorators
    _clampable          = 0L;
    _drapeable          = 0L;
    _overlayPlaceholder = new osg::Group();
    _overlayInstalled   = _overlayPlaceholder;

    osg::Node* node = 0;
    // if there's a display schema in place, set up for quadtree paging.
    if ( _options.layout().isSet() || _useTiledSource )
    {
        node = setupPaging();
    }
    else
    {
        FeatureLevel defaultLevel( 0.0f, FLT_MAX );
        
        //Remove all current children
        node = buildLevel( defaultLevel, GeoExtent::INVALID, 0 );
    }

    float minRange = -FLT_MAX;
    if ( _options.minRange().isSet() ) 
        minRange = std::max(minRange, *_options.minRange());

    if ( _options.layout().isSet() && _options.layout()->minRange().isSet() )
        minRange = std::max(minRange, *_options.layout()->minRange());

    float maxRange = FLT_MAX;
    if ( _options.maxRange().isSet() ) 
        maxRange = std::min(maxRange, *_options.maxRange());

    if ( _options.layout().isSet() && _options.layout()->maxRange().isSet() )
        maxRange = std::min(maxRange, *_options.layout()->maxRange());
    
    //If they've specified a min/max range, setup an LOD
    if ( minRange != -FLT_MAX || maxRange != FLT_MAX )
    {        
        // todo: revisit this, make sure this is still right.
        ElevationLOD *lod = new ElevationLOD(_session->getMapInfo().getSRS(), minRange, maxRange );
        lod->addChild( node );
        node = lod;
    }

    // If we want fading, install fading.
    if ( _options.fading().isSet() )
    {
        FadeEffect* fader = new FadeEffect();
        fader->setFadeDuration( *_options.fading()->duration() );
        fader->setMaxRange( *_options.fading()->maxRange() );
        fader->setAttenuationDistance( *_options.fading()->attenuationDistance() );
        fader->addChild( node );
        node = fader;
    }

    // overlay placeholder. this will make it easier to 
    // replace with a clamper/draper later if necessary
    {
        _overlayInstalled->addChild( node );
        node = _overlayInstalled;
    }

    addChild( node );

    _session->getFeatureSource()->sync( _featureSourceRev );
    if ( _modelSource.valid() )
        _modelSource->sync( _modelSourceRev );

    _dirty = false;
}

void
FeatureModelGraph::setStyles( StyleSheet* styles )
{
    _session->setStyles( styles );
    dirty();
}
