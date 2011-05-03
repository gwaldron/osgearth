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
#include <osg/PagedLOD>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>

#define LC "[FeatureModelGraph] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

// pseudo-loader for paging in feature tiles for a FeatureModelGraph.

namespace
{
    UID _uid = 0;
    OpenThreads::Mutex _fmgMutex;
    std::map<UID, FeatureModelGraph*> _fmgRegistry;
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
        unsigned levelIndex, x, y;
        sscanf( uri.c_str(), "%u.%d_%d_%d.%*s", &uid, &levelIndex, &x, &y );

        //OE_INFO << LC << "Page in: " << uri << std::endl;

        FeatureModelGraph* graph = getGraph(uid);
        if ( graph )
            return ReadResult( graph->load( levelIndex, x, y, uri ) );
        else
            return ReadResult::ERROR_IN_READING_FILE;
    }

    static UID registerGraph( FeatureModelGraph* graph )
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _fmgMutex );
        UID key = ++_uid;
        _fmgRegistry[key] = graph;
        return key;
    }

    static void unregisterGraph( UID uid )
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _fmgMutex );
        _fmgRegistry.erase( uid );
    }

    static FeatureModelGraph* getGraph( UID uid ) 
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _fmgMutex );
        std::map<UID, FeatureModelGraph*>::const_iterator i = _fmgRegistry.find( uid );
        return i != _fmgRegistry.end() ? i->second : 0L;
    }

    static std::string makeURI( UID uid ) 
    {
        std::stringstream buf;
        buf << uid << ".osgearth_pseudo_fmg";
        std::string str = buf.str();
        return str;
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
                                     const StyleSheet&                styles,
                                     Session*                         session ) :
_source ( source ),
_options( options ),
_factory( factory ),
_styles ( styles ),
_session( session )
{
    _uid = osgEarthFeatureModelPseudoLoader::registerGraph( this );

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
    _fullWorldBound = getBoundInWorldCoords( _usableMapExtent );

    // whether to request tiles from the source (if available). if the source is tiled, but the
    // user manually specified schema levels, don't use the tiles.
    _useTiledSource = _source->getFeatureProfile()->getTiled();
    if ( _useTiledSource && options.levels().isSet() && options.levels()->getNumLevels() > 0 )
        _useTiledSource = false;

    // if there's a display schema in place, set up for quadtree paging.
    if ( options.levels().isSet() || _useTiledSource ) //_source->getFeatureProfile()->getTiled() )
    {
        setupPaging();
    }
    else
    {
        FeatureLevel defaultLevel( 0.0f, FLT_MAX );
        osg::Node* node = build( defaultLevel, GeoExtent::INVALID, 0 );
        if ( node )
            this->addChild( node );
    }
}

FeatureModelGraph::~FeatureModelGraph()
{
    osgEarthFeatureModelPseudoLoader::unregisterGraph( _uid );
}

osg::BoundingSphered
FeatureModelGraph::getBoundInWorldCoords( const GeoExtent& extent ) const
{
    osg::Vec3d center, corner;

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
    corner.x() = workingExtent.xMin();
    corner.y() = workingExtent.yMin();

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
    float maxRange = _options.levels().isSet() && _options.levels()->getNumLevels() > 0 ? _options.levels()->getMaxRange() : FLT_MAX;
    if ( maxRange > 0.0f )
    {
        osg::BoundingSphered bs = getBoundInWorldCoords( _usableMapExtent );

        float tileFactor = _options.levels().isSet() ? _options.levels()->tileSizeFactor().get() : 15.0f;

        const FeatureLevel* firstLevel = 0;
        unsigned firstLOD = 0;

        FeatureLevel defaultLevel( 0.0f, FLT_MAX );
        FeatureLevel defaultTiledLevel( 0.0f, bs.radius() * tileFactor );

        if (_options.levels().isSet() && _options.levels()->getNumLevels() > 0)
        {
            firstLevel = _options.levels()->getLevel( 0 );
            firstLOD = _options.levels()->chooseLOD( *firstLevel, bs.radius() );
        }
        else if (_source->getFeatureProfile()->getTiled())
        {            
            firstLOD = 0;
            firstLevel = &defaultTiledLevel;
        }
        else
        {
            firstLOD = 0;
            firstLevel = &defaultLevel;
        }

        osg::Group* group = new osg::Group();
        buildSubTiles( 0, 0, 0, 0, firstLevel, firstLOD, group );

        this->addChild( group );
    }
}

void
FeatureModelGraph::buildSubTiles(unsigned            nextLevelIndex,
                                 unsigned            lod,
                                 unsigned            tileX,
                                 unsigned            tileY,
                                 const FeatureLevel* nextLevel,
                                 unsigned            nextLOD,
                                 osg::Group*         parent)
{
    // calculate how many subtiles there will be:
    unsigned numTiles = 1;
    for( unsigned k = lod; k < nextLOD; ++k )
    {
        tileX *= 2;
        tileY *= 2;
        numTiles *= 2;
    }

    OE_DEBUG << LC 
        << "Building " << numTiles*numTiles << " plods for next level = " << nextLevelIndex
        << ", nextLOD = " << nextLOD
        << std::endl;

    // make a paged LOD for each subtile:
    for( unsigned u = tileX; u < tileX+numTiles; ++u )
    {
        for( unsigned v = tileY; v < tileY+numTiles; ++v )
        {
            GeoExtent subtileFeatureExtent = s_getTileExtent( nextLOD, u, v, _usableFeatureExtent );
            osg::BoundingSphered subtile_bs = getBoundInWorldCoords( subtileFeatureExtent );

            std::stringstream buf;
            buf << _uid << "." << nextLevelIndex << "_" << u << "_" << v << ".osgearth_pseudo_fmg";

            std::string uri = buf.str();

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

                osg::PagedLOD* plod = new osg::PagedLOD();
                plod->setName( uri );
                // We don't really know the exact center/radius beforehand, since we have yet to generate any data,
                // so approximate it by using the tile bounds...
                plod->setCenter  ( subtile_bs.center() );
                plod->setRadius  ( subtile_bs.radius() );
                plod->setFileName( 0, uri );
                plod->setRange   ( 0, 0, nextLevel->maxRange() );
                plod->setPriorityOffset( 0, -(float)nextLOD );

                parent->addChild( plod );
            }
        }
    }

    // tree up the plods to cull a little more efficiently (not sure this actually works since
    // we didn't set radii on the plods)
    //osgUtil::Optimizer optimizer;
    //optimizer.optimize( parent, osgUtil::Optimizer::SPATIALIZE_GROUPS );
}

osg::Node*
FeatureModelGraph::load( unsigned levelIndex, unsigned tileX, unsigned tileY, const std::string& uri )
{
    // note: "level" is not the same as "lod". "level" is an index into the FeatureDisplayLayout
    // levels list, which is sorted by maxRange.

    OE_DEBUG << LC
        << "load: " << levelIndex << "_" << tileX << "_" << tileY << std::endl;

    osg::Group* result = 0L;
    
    if ( _useTiledSource )
    {        
        // Handle a tiled feature source:

        unsigned int lod = levelIndex;
        GeoExtent tileExtent = 
            lod >= 0 ?
            s_getTileExtent( levelIndex, tileX, tileY, _usableFeatureExtent ) : GeoExtent::INVALID;

        osg::BoundingSphered tileBound = getBoundInWorldCoords( tileExtent );

        float tileFactor = _options.levels().isSet() ? _options.levels()->tileSizeFactor().get() : 15.0f;

        double maxRange =  tileBound.radius() * tileFactor;
        FeatureLevel level( 0, maxRange );
        
        TileKey key(lod, tileX, tileY, _source->getFeatureProfile()->getProfile());
        osg::Group* geometry = build( level, tileExtent, &key );
        result = geometry;

        if (lod < _source->getFeatureProfile()->getMaxLevel())
        {
            // see if there are any more levels. If so, build some pagedlods to bring the
            // next one in.
            FeatureLevel nextLevel(0, maxRange/2.0);

            osg::ref_ptr<osg::Group> group = new osg::Group();

            // calculate the LOD of the next level:
            unsigned nextLOD = lod+1;
            if ( nextLOD != ~0 )
            {
                buildSubTiles( levelIndex+1, levelIndex, tileX, tileY, &nextLevel, nextLOD, group.get() );

                // slap the geometry in there afterwards, if there is any
                if ( geometry )
                    group->addChild( geometry );

                result = group.release();
            }   
        }
    }

    else if ( !_options.levels().isSet() || _options.levels()->getNumLevels() == 0 )
    {
        // no levels defined; just load all the features.
        FeatureLevel all( 0.0f, FLT_MAX );
        result = build( all, GeoExtent::INVALID, 0 );
    }

    else
    {
        const FeatureLevel* level = _options.levels()->getLevel( levelIndex );

        if ( level )
        {
            unsigned lod = _options.levels()->chooseLOD( *level, _fullWorldBound.radius() );

            OE_DEBUG << LC 
                << "Choose LOD " << lod << " for level " << levelIndex 
                << std::endl;

            GeoExtent tileExtent = 
                lod > 0 ?
                s_getTileExtent( lod, tileX, tileY, _usableFeatureExtent ) :
                GeoExtent::INVALID;

            osg::Group* geometry = build( *level, tileExtent, 0 );
            result = geometry;

            // see if there are any more levels. If so, build some pagedlods to bring the
            // next one in.
            const FeatureLevel* nextLevel = _options.levels()->getLevel( levelIndex+1 );
            if ( nextLevel )
            {
                osg::ref_ptr<osg::Group> group = new osg::Group();

                // calculate the LOD of the next level:
                unsigned nextLOD = _options.levels()->chooseLOD( *nextLevel, _fullWorldBound.radius() );
                if ( nextLOD != ~0 )
                {
                    buildSubTiles( levelIndex+1, lod, tileX, tileY, nextLevel, nextLOD, group.get() );

                    // slap the geometry in there afterwards, if there is any
                    if ( geometry )
                        group->addChild( geometry );

                    result = group.release();
                }
            }
        }
    }

    // If the read resulting in nothing, do two things. First, blacklist the URI
    // so that the next time we try to create a PagedLOD pointing at this URI, it
    // will find it in the blacklist and not create said PagedLOD. Second, create
    // an empty group so that the read (technically) succeeds and it doesn't try
    // to load the null child over and over.
    if ( !result )
    {
        result = new osg::Group();
    }
    else
    {
        RemoveEmptyGroupsVisitor::run( result );
    }

    if ( result->getNumChildren() == 0 )
    {
        Threading::ScopedWriteLock exclusiveLock( _blacklistMutex );
        _blacklist.insert( uri );

        OE_DEBUG << LC << "Blacklisting: " << uri << std::endl;
    }

    return result;
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
        osg::Node* node = build( Style(), query, extent );
        if ( node )
            group->addChild( node );
    }

    else
    {
        for( StyleSelectorVector::const_iterator i = levelSelectors.begin(); i != levelSelectors.end(); ++i )
        {
            const StyleSelector& selector = *i;

            // fetch the selector's style:
            Style selectorStyle;
            _styles.getStyle( selector.getSelectedStyleName(), selectorStyle );

            // combine the selector's query, if it has one:
            Query selectorQuery = 
                selector.query().isSet() ? query.combineWith( *selector.query() ) : query;

            osg::Node* node = build( selectorStyle, selectorQuery, extent );
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
                    // get the geocentric tile center:
                    osg::Vec3d tileCenter;
                    ccExtent.getCentroid( tileCenter.x(), tileCenter.y() );
                    osg::Vec3d centerECEF;
                    ccExtent.getSRS()->transformToECEF( tileCenter, centerECEF );

                    osg::NodeCallback* ccc = ClusterCullerFactory::create( group.get(), centerECEF );
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
FeatureModelGraph::build( const Style& baseStyle, const Query& baseQuery, const GeoExtent& workingExtent )
{
    osg::ref_ptr<osg::Group> group = new osg::Group();

    if ( _source->hasEmbeddedStyles() )
    {
        const FeatureProfile* profile = _source->getFeatureProfile();

        // each feature has its own style, so use that and ignore the style catalog.
        osg::ref_ptr<FeatureCursor> cursor = _source->createFeatureCursor( baseQuery );
        while( cursor->hasMore() )
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

                osg::Group* styleGroup = _factory->getOrCreateStyleGroup(*feature->style(), _session.get());
                if ( !group->containsNode( styleGroup ) )
                    group->addChild( styleGroup );                

                if ( _factory->createOrUpdateNode( cursor, *feature->style(), context, node ) )
                {
                    if ( node.valid() )
                        styleGroup->addChild( node );
                }
            }
        }
    }

    else
    {
        // if we have selectors, sort the features into style groups and create a node for each group.
        if ( _styles.selectors().size() > 0 )
        {
            for( StyleSelectorList::const_iterator i = _styles.selectors().begin(); i != _styles.selectors().end(); ++i )
            {
                // pull the selected style...
                const StyleSelector& sel = *i;

                // combine the selection style with the incoming base style:
                Style selectedStyle;
                _styles.getStyle( sel.getSelectedStyleName(), selectedStyle );
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
                _styles.getDefaultStyle( combinedStyle );

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

    if ( cursor->hasMore() )
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
            _options.levels()->cropFeatures() == true ? 
            CropFilter::METHOD_CROPPING : CropFilter::METHOD_CENTROID );
        context = crop.push( workingSet, context );

        // next, if the usable extent is less than the full extent (i.e. we had to clamp the feature
        // extent to fit on the map), calculate the extent of the features in this tile and 
        // crop to the map extent if necessary. (Note, if cropFeatures was set to true, this is
        // already done)
        if ( _featureExtentClamped && _options.levels()->cropFeatures() == false )
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

            if ( _factory->createOrUpdateNode( newCursor.get(), style, context, node ) )
            {
                if ( !styleGroup )
                    styleGroup = _factory->getOrCreateStyleGroup( style, _session.get() );

                // if it returned a node, add it. (it doesn't necessarily have to)
                if ( node.valid() )
                    styleGroup->addChild( node.get() );
            }
        }
    }


    return styleGroup;
}
