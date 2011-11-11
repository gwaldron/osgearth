/* --*-c++-*-- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/ThreadingUtils>
#include <osgEarth/NodeUtils>
#include <osgEarth/ElevationQuery>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>

#define LC "[FeatureModelGraph] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#undef USE_PROXY_NODE_FOR_TESTING

//---------------------------------------------------------------------------

// pseudo-loader for paging in feature tiles for a FeatureModelGraph.

namespace
{
    UID                               _uid         = 0;
    Threading::ReadWriteMutex         _fmgMutex;
    std::map<UID, FeatureModelGraph*> _fmgRegistry;

    static std::string s_makeURI( UID uid, unsigned lod, unsigned x, unsigned y ) 
    {
        std::stringstream buf;
        buf << uid << "." << lod << "_" << x << "_" << y << ".osgearth_pseudo_fmg";
        std::string str = buf.str();
        return str;
    }

    osg::Group* createPagedNode( const osg::BoundingSphered& bs, const std::string& uri, float minRange, float maxRange )
    {
#ifdef USE_PROXY_NODE_FOR_TESTING
        osg::ProxyNode* p = new osg::ProxyNode();
        p->setCenter( bs.center() );
        p->setRadius( bs.radius() );
        p->setFileName( 0, uri );
#else
        osg::PagedLOD* p = new osg::PagedLOD();
        p->setCenter( bs.center() );
        p->setRadius( bs.radius() );
        p->setFileName( 0, uri );
        p->setRange( 0, minRange, maxRange );
#endif
        return p;
    }
}

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

        FeatureModelGraph* graph = getGraph(uid);
        if ( graph )
            return ReadResult( graph->load( lod, x, y, uri ) );
        else
            return ReadResult::ERROR_IN_READING_FILE;
    }

    static UID registerGraph( FeatureModelGraph* graph )
    {
        Threading::ScopedWriteLock lock( _fmgMutex );
        UID key = ++_uid;
        _fmgRegistry[key] = graph;
        return key;
    }

    static void unregisterGraph( UID uid )
    {
        Threading::ScopedWriteLock lock( _fmgMutex );
        _fmgRegistry.erase( uid );
    }

    static FeatureModelGraph* getGraph( UID uid ) 
    {
        Threading::ScopedReadLock lock( _fmgMutex );
        std::map<UID, FeatureModelGraph*>::const_iterator i = _fmgRegistry.find( uid );
        return i != _fmgRegistry.end() ? i->second : 0L;
    }
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
}


//---------------------------------------------------------------------------

FeatureModelGraph::FeatureModelGraph(FeatureSource*                   source,
                                     const FeatureModelSourceOptions& options,
                                     FeatureNodeFactory*              factory,
                                     Session*                         session) :
_source   ( source ),
_options  ( options ),
_factory  ( factory ),
_session  ( session ),
_dirty    ( false )
{
    _uid = osgEarthFeatureModelPseudoLoader::registerGraph( this );

    // install the stylesheet in the session if it doesn't already have one.
    if ( !session->styles() )
        session->setStyles( _options.styles().get() );

    // initialize lighting on the graph, if necessary.
    osg::StateSet* stateSet = getOrCreateStateSet();

    if ( _options.enableLighting().isSet() )
        stateSet->setMode( GL_LIGHTING, *_options.enableLighting() ? 1 : 0 );
    
    // Calculate the usable extent (in both feature and map coordinates) and bounds.
    const Profile* mapProfile = session->getMapInfo().getProfile();

    // the part of the feature extent that will fit on the map (in map coords):
    _usableMapExtent = mapProfile->clampAndTransformExtent( 
        _source->getFeatureProfile()->getExtent(), 
        &_featureExtentClamped );

    // same, back into feature coords:
    _usableFeatureExtent = _usableMapExtent.transform( _source->getFeatureProfile()->getSRS() );

    // world-space bounds of the feature layer
    _fullWorldBound = getBoundInWorldCoords( _usableMapExtent, 0L );

    // whether to request tiles from the source (if available). if the source is tiled, but the
    // user manually specified schema levels, don't use the tiles.
    _useTiledSource = _source->getFeatureProfile()->getTiled();

    if ( options.levels().isSet() && options.levels()->getNumLevels() > 0 )
    {
        // the user provided a custom levels setup, so don't use the tiled source (which
        // provides its own levels setup)
        _useTiledSource = false;

        // for each custom level, calculate the best LOD match and store it in the level
        // layout data. We will use this information later when constructing the SG in
        // the pager.
        for( unsigned i = 0; i < options.levels()->getNumLevels(); ++i )
        {
            const FeatureLevel* level = options.levels()->getLevel( i );
            unsigned lod = options.levels()->chooseLOD( *level, _fullWorldBound.radius() );
            _lodmap.resize( lod+1, 0L );
            _lodmap[lod] = level;

            OE_INFO << LC << source->getName() 
                << ": F.Level max=" << level->maxRange() << ", min=" << level->minRange()
                << ", LOD=" << lod << std::endl;
        }
    }

    setNumChildrenRequiringUpdateTraversal( 1 );

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

osg::BoundingSphered
FeatureModelGraph::getBoundInWorldCoords(const GeoExtent& extent,
                                         const MapFrame*  mapf ) const
{
    osg::Vec3d center, corner;
    //double z = 0.0;
    GeoExtent workingExtent;

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
        query.getElevation( center, mapf->getProfile()->getSRS(), center.z(), resolution );
        centerZ = center.z();
    }    

    corner.x() = workingExtent.xMin();
    corner.y() = workingExtent.yMin();
    corner.z() = 0;

    if ( _session->getMapInfo().isGeocentric() )
    {
        workingExtent.getSRS()->transformToECEF( center, center );
        workingExtent.getSRS()->transformToECEF( corner, corner );
    }

    return osg::BoundingSphered( center, (center-corner).length() );
}

void
FeatureModelGraph::setupPaging()
{
    // calculate the bounds of the full data extent:
    MapFrame mapf = _session->createMapFrame();
    osg::BoundingSphered bs = getBoundInWorldCoords( _usableMapExtent, &mapf );

    // calculate the max range for the top-level PLOD:
    float maxRange = bs.radius() * _options.levels()->tileSizeFactor().value();

    // build the URI for the top-level paged LOD:
    std::string uri = s_makeURI( _uid, 0, 0, 0 );

    // bulid the top level Paged LOD:
    osg::Group* pagedNode = createPagedNode( bs, uri, 0.0f, maxRange );
    this->addChild( pagedNode );
}

osg::Node*
FeatureModelGraph::load( unsigned lod, unsigned tileX, unsigned tileY, const std::string& uri )
{
    OE_DEBUG << LC
        << "load: " << lod << "_" << tileX << "_" << tileY << std::endl;

    osg::Group* result = 0L;
    
    if ( _useTiledSource )
    {       
        // A "tiled" source has a pre-generted tile hierarchy, but no range information.
        // We will be calculating the LOD ranges here.
        osg::Group* geometry =0L;

        if ( lod >= _source->getFeatureProfile()->getFirstLevel() )
        {
            // The extent of this tile:
            GeoExtent tileExtent = s_getTileExtent( lod, tileX, tileY, _usableFeatureExtent );

            // Calculate the bounds of this new tile:
            MapFrame mapf = _session->createMapFrame();
            osg::BoundingSphered tileBound = getBoundInWorldCoords( tileExtent, &mapf );

            // Apply the tile range multiplier to calculate a max camera range. The max range is
            // the geographic radius of the tile times the multiplier.
            float tileFactor = _options.levels().isSet() ? _options.levels()->tileSizeFactor().get() : 15.0f;            
            double maxRange =  tileBound.radius() * tileFactor;
            FeatureLevel level( 0, maxRange );
            //OE_NOTICE << "(" << lod << ": " << tileX << ", " << tileY << ")" << std::endl;
            //OE_NOTICE << "  extent = " << tileExtent.width() << "x" << tileExtent.height() << std::endl;
            //OE_NOTICE << "  tileFactor = " << tileFactor << " maxRange=" << maxRange << " radius=" << tileBound.radius() << std::endl;
            
            // Construct a tile key that will be used to query the source for this tile.
            TileKey key(lod, tileX, tileY, _source->getFeatureProfile()->getProfile());
            geometry = build( level, tileExtent, &key );
            result = geometry;
        }

        if ( lod < _source->getFeatureProfile()->getMaxLevel() )
        {
            // see if there are any more levels. If so, build some pagedlods to bring the
            // next level in.
            //FeatureLevel nextLevel(0, maxRange/2.0);

            osg::ref_ptr<osg::Group> group = new osg::Group();

            // calculate the LOD of the next level:
            if ( lod+1 != ~0 )
            {
                //if ( geometry == 0L )
                //{
                //    OE_WARN << LC << "OK...geometry is null, LOD is = " << lod 
                //        << ", firstLOD = " << _source->getFeatureProfile()->getFirstLevel()
                //        << ", maxLOD = " << _source->getFeatureProfile()->getMaxLevel()
                //        << std::endl;
                //}

                // only build sub-pagedlods if we are expecting subtiles at some point:
                if ( geometry != 0L || lod < _source->getFeatureProfile()->getFirstLevel() )
                {
                    MapFrame mapf = _session->createMapFrame();
                    buildSubTilePagedLODs( lod, tileX, tileY, &mapf, group.get() );
                    group->addChild( geometry );
                }

                //// slap the geometry in there afterwards, if there is any
                //if ( geometry )
                //    group->addChild( geometry );

                result = group.release();
            }   
        }
    }

    else if ( !_options.levels().isSet() || _options.levels()->getNumLevels() == 0 )
    {
        // This is a non-tiled data source that has NO level details. In this case, 
        // we simply want to load all features at once and make them visible at
        // maximum camera range.
        FeatureLevel all( 0.0f, FLT_MAX );
        result = build( all, GeoExtent::INVALID, 0 );
    }

    else if ( lod < _lodmap.size() )
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

            geometry = build( *level, tileExtent, 0 );
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
        RemoveEmptyGroupsVisitor::run( result );
    }

    if ( result->getNumChildren() == 0 )
    {
        // if the result group contains no data, blacklist it so we never try to load it again.
        Threading::ScopedWriteLock exclusiveLock( _blacklistMutex );
        _blacklist.insert( uri );
        OE_DEBUG << LC << "Blacklisting: " << uri << std::endl;
    }

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

    // make a paged LOD for each subtile:
    for( unsigned u = subtileX; u <= subtileX + 1; ++u )
    {
        for( unsigned v = subtileY; v <= subtileY + 1; ++v )
        {
            GeoExtent subtileFeatureExtent = s_getTileExtent( subtileLOD, u, v, _usableFeatureExtent );
            osg::BoundingSphered subtile_bs = getBoundInWorldCoords( subtileFeatureExtent, mapf );

            // Camera range for the PLODs. This should always be sufficient because
            // the max range of a FeatureLevel below this will, by definition, have a max range
            // less than or equal to this number -- based on how the LODs were chosen in 
            // setupPaging.
            float maxRange = subtile_bs.radius() * _options.levels()->tileSizeFactor().value();

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
                    << std::endl;

                osg::Group* pagedNode = createPagedNode( subtile_bs, uri, 0.0f, maxRange );
                parent->addChild( pagedNode );
            }
        }
    }
}

osg::Group*
FeatureModelGraph::build( const FeatureLevel& level, const GeoExtent& extent, const TileKey* key )
{
    osg::ref_ptr<osg::Group> group = new osg::Group();

    // form the baseline query, which does a spatial query based on the working extent.
    Query query;
    if ( extent.isValid() )
        query.bounds() = extent.bounds();

    // add a tile key to the query if there is one, to support TFS-style queries
    if ( key )
        query.tileKey() = *key;

    // now, go through any level-based selectors.
    const StyleSelectorVector& levelSelectors = level.selectors();
    
    // if there are none, just build once with the default style and query.
    if ( levelSelectors.size() == 0 )
    {
        // attempt to glean the style from the feature source name:
        const Style style = *_session->styles()->getStyle( *_source->getFeatureSourceOptions().name() );

        osg::Node* node = build( style, query, extent );
        if ( node )
            group->addChild( node );
    }

    else
    {
        for( StyleSelectorVector::const_iterator i = levelSelectors.begin(); i != levelSelectors.end(); ++i )
        {
            const StyleSelector& selector = *i;

            // fetch the selector's style:
            const Style* selectorStyle = _session->styles()->getStyle( selector.getSelectedStyleName() );

            // combine the selector's query, if it has one:
            Query selectorQuery = 
                selector.query().isSet() ? query.combineWith( *selector.query() ) : query;

            osg::Node* node = build( *selectorStyle, selectorQuery, extent );
            if ( node )
                group->addChild( node );
        }
    }

    if ( group->getNumChildren() > 0 )
    {
        // account for a min-range here.
        if ( level.minRange() > 0.0f )
        {
            osg::LOD* lod = new osg::LOD();
            lod->addChild( group.get(), level.minRange(), FLT_MAX );
            group = lod;
        }

        if ( _session->getMapInfo().isGeocentric() && _options.clusterCulling() == true )
        {
            const GeoExtent& ccExtent = extent.isValid() ? extent : _source->getFeatureProfile()->getExtent();
            if ( ccExtent.isValid() )
            {
                // if the extent is more than 90 degrees, bail
                GeoExtent geodeticExtent = ccExtent.transform( ccExtent.getSRS()->getGeographicSRS() );
                if ( geodeticExtent.width() < 90.0 && geodeticExtent.height() < 90.0 )
                {
#if 1
                    // get the geocentric tile center:
                    osg::Vec3d tileCenter;
                    ccExtent.getCentroid( tileCenter.x(), tileCenter.y() );
                    osg::Vec3d centerECEF;
                    ccExtent.getSRS()->transformToECEF( tileCenter, centerECEF );

                    osg::NodeCallback* ccc = ClusterCullerFactory::create( group.get(), centerECEF );
                    if ( ccc )
                        group->addCullCallback( ccc );
#endif
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
FeatureModelGraph::build( const Style& baseStyle, const Query& baseQuery, const GeoExtent& workingExtent )
{
    osg::ref_ptr<osg::Group> group = new osg::Group();

    if ( _source->hasEmbeddedStyles() )
    {
        const FeatureProfile* profile = _source->getFeatureProfile();

        // each feature has its own style, so use that and ignore the style catalog.
        osg::ref_ptr<FeatureCursor> cursor = _source->createFeatureCursor( baseQuery );
        while( cursor.valid() && cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );
                osg::ref_ptr<FeatureCursor> cursor = new FeatureListCursor(list);

                FilterContext context( _session.get(), _source->getFeatureProfile(), workingExtent );

                // note: gridding is not supported for embedded styles.
                osg::ref_ptr<osg::Node> node;

                // Get the Group that parents all features of this particular style. Note, this
                // might be NULL if the factory does not support style groups.
                osg::Group* styleGroup = _factory->getOrCreateStyleGroup(*feature->style(), _session.get());
                if ( styleGroup )
                {
                    if ( !group->containsNode( styleGroup ) )
                        group->addChild( styleGroup );
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

    else
    {
        const StyleSheet* styles = _session->styles();

        // if we have selectors, sort the features into style groups and create a node for each group.
        if ( styles->selectors().size() > 0 )
        {
            for( StyleSelectorList::const_iterator i = styles->selectors().begin(); i != styles->selectors().end(); ++i )
            {
                // pull the selected style...
                const StyleSelector& sel = *i;

                // combine the selection style with the incoming base style:
                Style selectedStyle = *styles->getStyle( sel.getSelectedStyleName() );
                Style combinedStyle = baseStyle.combineWith( selectedStyle );

                // .. and merge it's query into the existing query
                Query combinedQuery = baseQuery.combineWith( *sel.query() );

                // then create the node.
                osg::Group* styleGroup = createNodeForStyle( combinedStyle, combinedQuery );
                if ( styleGroup && !group->containsNode(styleGroup) )
                    group->addChild( styleGroup );
            }
        }

        // otherwise, render all the features with a single style
        else
        {
            Style combinedStyle = baseStyle;

            // if there's no base style defined, choose a "default" style from the stylesheet.
            if ( baseStyle.empty() )
                combinedStyle = *styles->getDefaultStyle();

            osg::Group* styleGroup = createNodeForStyle( combinedStyle, baseQuery );
            if ( styleGroup && !group->containsNode(styleGroup) )
                group->addChild( styleGroup );
        }
    }

    return group->getNumChildren() > 0 ? group.release() : 0L;
}

osg::Group*
FeatureModelGraph::createNodeForStyle(const Style& style, const Query& query)
{
    osg::Group* styleGroup = 0L;

    // the profile of the features
    const FeatureProfile* profile = _source->getFeatureProfile();

    // get the extent of the full set of feature data:
    const GeoExtent& extent = profile->getExtent();
    
    // query the feature source:
    osg::ref_ptr<FeatureCursor> cursor = _source->createFeatureCursor( query );

    if ( cursor.valid() && cursor->hasMore() )
    {
        Bounds cellBounds =
            query.bounds().isSet() ? *query.bounds() : extent.bounds();

        FilterContext context( _session.get(), profile, GeoExtent(profile->getSRS(), cellBounds) );

        // start by culling our feature list to the working extent. By default, this is done by
        // checking feature centroids. But the user can override this to crop feature geometry to
        // the cell boundaries.
        FeatureList workingSet;
        cursor->fill( workingSet );

        CropFilter crop( 
            _options.levels().isSet() && _options.levels()->cropFeatures() == true ? 
            CropFilter::METHOD_CROPPING : CropFilter::METHOD_CENTROID );
        context = crop.push( workingSet, context );

        // next, if the usable extent is less than the full extent (i.e. we had to clamp the feature
        // extent to fit on the map), calculate the extent of the features in this tile and 
        // crop to the map extent if necessary. (Note, if cropFeatures was set to true, this is
        // already done)
        if ( _featureExtentClamped && _options.levels().isSet() && _options.levels()->cropFeatures() == false )
        {
            context.extent() = _usableFeatureExtent;
            CropFilter crop2( CropFilter::METHOD_CROPPING );
            context = crop2.push( workingSet, context );
        }

        if ( workingSet.size() > 0 )
        {
            // next ask the implementation to construct OSG geometry for the cell features.
            osg::ref_ptr<osg::Node> node;

            osg::ref_ptr<FeatureCursor> newCursor = new FeatureListCursor(workingSet);
			FeatureSource * oldFeatureSource = _session->getFeatureSource();
			_session->setFeatureSource(_source.get());

            if ( _factory->createOrUpdateNode( newCursor.get(), style, context, node ) )
            {
                if ( !styleGroup )
                    styleGroup = _factory->getOrCreateStyleGroup( style, _session.get() );

                // if it returned a node, add it. (it doesn't necessarily have to)
                if ( node.valid() )
                    styleGroup->addChild( node.get() );
            }
			_session->setFeatureSource(oldFeatureSource);
        }

        CacheStats stats = context.resourceCache()->getSkinStats();
        OE_DEBUG << LC << "Resource Cache skins: "
            << " num=" << stats._entries << ", max=" << stats._maxEntries
            << ", queries=" << stats._queries << ", hits=" << (100.0f*stats._hitRatio) << "%"
            << std::endl;

    }


    return styleGroup;
}

void
FeatureModelGraph::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        if (_source->outOfSyncWith(_revision) || _dirty)
        {
            redraw();
        }
    }
    osg::Group::traverse(nv);
}

void
FeatureModelGraph::redraw()
{
    removeChildren( 0, getNumChildren() );
    // if there's a display schema in place, set up for quadtree paging.
    if ( _options.levels().isSet() || _useTiledSource ) //_source->getFeatureProfile()->getTiled() )
    {
        setupPaging();
    }
    else
    {
        FeatureLevel defaultLevel( 0.0f, FLT_MAX );
        
        //Remove all current children        
        osg::Node* node = build( defaultLevel, GeoExtent::INVALID, 0 );
        if ( node )
            addChild( node );
    }

    _source->sync( _revision );
    _dirty = false;
}

void
FeatureModelGraph::setStyles( StyleSheet* styles )
{
    _session->setStyles( styles );
    dirty();
}
