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
//#include <osgEarthFeatures/FeatureGridder>
#include <osgEarthFeatures/CropFilter>
#include <osgEarth/ThreadingUtils>
#include <osg/ClusterCullingCallback>
#include <osg/PagedLOD>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
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

    // if there's a display schema in place, set up for quadtree paging.
    if ( options.levels().isSet() )
    {
        setupPaging();
    }
    else
    {
        FeatureLevel defaultLevel( 0.0f, FLT_MAX, Query() );
        osg::Node* node = build( defaultLevel, GeoExtent::INVALID );
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

    GeoExtent extentOnMap = extent.transform( _session->getMap().getProfile()->getSRS() );
    extentOnMap.getCentroid( center.x(), center.y() );
    corner.x() = extentOnMap.xMin();
    corner.y() = extentOnMap.yMin();

    if ( _session->getMap().getMapInfo().isGeocentric() )
    {
        extentOnMap.getSRS()->transformToECEF( center, center );
        extentOnMap.getSRS()->transformToECEF( corner, corner );
    }

    return osg::BoundingSphered( center, (center-corner).length() );
}

void
FeatureModelGraph::setupPaging()
{
    float maxRange = _options.levels().isSet() ? _options.levels()->getMaxRange() : FLT_MAX;
    if ( maxRange > 0.0f )
    {
        // find the bounds.
        const GeoExtent& fullExtent = _source->getFeatureProfile()->getExtent();
        osg::BoundingSphered bs = getBoundInWorldCoords( fullExtent );

        const FeatureLevel* firstLevel = _options.levels()->getLevel( 0 );
        unsigned firstLOD = _options.levels()->chooseLOD( *firstLevel, bs.radius() );

        osg::Group* group = new osg::Group();
        buildSubTiles( 0, 0, 0, 0, firstLevel, firstLOD, group );

        this->addChild( group );
    }
}

void
FeatureModelGraph::buildSubTiles(unsigned nextLevelIndex,
                                 unsigned lod,
                                 unsigned tileX,
                                 unsigned tileY,
                                 const FeatureLevel* nextLevel,
                                 unsigned nextLOD,
                                 osg::Group* parent)
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

    const GeoExtent& fullExtent = _source->getFeatureProfile()->getExtent();

    // make a paged LOD for each subtile:
    for( unsigned u = tileX; u < tileX+numTiles; ++u )
    {
        for( unsigned v = tileY; v < tileY+numTiles; ++v )
        {
            GeoExtent subtileExtent = s_getTileExtent( nextLOD, u, v, fullExtent );
            osg::BoundingSphered subtile_bs = getBoundInWorldCoords( subtileExtent );

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
                plod->setCenter  ( subtile_bs.center() );
                plod->setRadius  ( subtile_bs.radius() );
                plod->setFileName( 0, uri );
                plod->setRange   ( 0, nextLevel->minRange(), nextLevel->maxRange() );

                parent->addChild( plod );
            }
        }
    }

    osgUtil::Optimizer optimizer;
    optimizer.optimize( parent, osgUtil::Optimizer::SPATIALIZE_GROUPS );
}

osg::Node*
FeatureModelGraph::load( unsigned levelIndex, unsigned tileX, unsigned tileY, const std::string& uri )
{
    // note: "level" is not the same as "lod". "level" is an index into the FeatureDisplaySchema
    // levels list, which is sorted by maxRange.

    OE_DEBUG << LC
        << "load: " << levelIndex << "_" << tileX << "_" << tileY << std::endl;

    osg::Node* result = 0L;

    // todo: cache this value
    const GeoExtent& fullExtent = _source->getFeatureProfile()->getExtent();
    osg::BoundingSphered fullBound = getBoundInWorldCoords( fullExtent );

    if ( !_options.levels().isSet() )
    {
        // no levels defined; just load all the features.
        FeatureLevel all( 0.0f, FLT_MAX, Query() );
        result = build( all, GeoExtent::INVALID );
    }

    else
    {
        const FeatureLevel* level = _options.levels()->getLevel( levelIndex );

        if ( level )
        {
            unsigned lod = _options.levels()->chooseLOD( *level, fullBound.radius() );

            OE_DEBUG << LC 
                << "Choose LOD " << lod << " for level " << levelIndex 
                << std::endl;

            GeoExtent tileExtent = 
                lod > 0 ?
                s_getTileExtent( lod, tileX, tileY, fullExtent ) :
                GeoExtent::INVALID;

            osg::Node* geometry = build( *level, tileExtent );
            result = geometry;

            // see if there are any more levels. If so, build some pagedlods to bring the
            // next one in.
            const FeatureLevel* nextLevel = _options.levels()->getLevel( levelIndex+1 );
            if ( nextLevel )
            {
                osg::ref_ptr<osg::Group> group = new osg::Group();

                // calculate the LOD of the next level:
                unsigned nextLOD = _options.levels()->chooseLOD( *nextLevel, fullBound.radius() );
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

    if ( !result )
    {
        // If the read resulting in nothing, do two things. First, blacklist the URI
        // so that the next time we try to create a PagedLOD pointing at this URI, it
        // will find it in the blacklist and not create said PagedLOD. Second, create
        // an empty group so that the read (technically) succeeds and it doesn't try
        // to load the null child over and over.

        result = new osg::Group();

        {
            Threading::ScopedWriteLock exclusiveLock( _blacklistMutex );
            _blacklist.insert( uri );
        }
    }

    return result;
}

osg::Node*
FeatureModelGraph::build( const FeatureLevel& level, const GeoExtent& extent )
{
    OE_DEBUG << LC
        << "Build started"
        << std::endl;

    osg::ref_ptr<osg::Group> group = new osg::Group();

    Query localQuery = *level.query();
    if ( extent.isValid() )
    {
        Query spatialQuery;
        spatialQuery.bounds() = extent.bounds();
        localQuery = localQuery.and( spatialQuery );
    }

    OE_DEBUG << LC
        << "local query = " << localQuery.getConfig().toString()
        << std::endl;

    if ( _source->hasEmbeddedStyles() )
    {
        const FeatureProfile* profile = _source->getFeatureProfile();

        // each feature has its own style, so use that and ignore the style catalog.
        osg::ref_ptr<FeatureCursor> cursor = _source->createFeatureCursor( localQuery );
        while( cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );
                osg::ref_ptr<FeatureCursor> cursor = new FeatureListCursor(list);

                FilterContext context( _session.get(), _source->getFeatureProfile(), extent );

                // note: gridding is not supported for embedded styles.
                osg::ref_ptr<osg::Node> node;
                if ( _factory->createOrUpdateNode( cursor, feature->style().get(), context, node ) )
                {
                    group->addChild( node );
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
                Style* style;
                _styles.getStyle( sel.getSelectedStyleName(), style );

                // .. and merge it's query into the existing query
                Query selectorQuery = localQuery.and( *sel.query() );

                // then create the node.
                osg::Group* styleGroup = createNodeForStyle( style, selectorQuery );
                if ( styleGroup )
                    group->addChild( styleGroup );
            }
        }

        // otherwise, render all the features with a single style
        else
        {
            const Style* style = _styles.getDefaultStyle();
            osg::Group* styleGroup = createNodeForStyle( style, localQuery );
            if ( styleGroup )
                group->addChild( styleGroup );
        }
    }

    OE_DEBUG << LC
        << "Build complete"
        << std::endl;

    return group->getNumChildren() > 0 ? group.release() : 0L;
}

osg::Group*
FeatureModelGraph::createNodeForStyle(const Style* style, const Query& query)
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

        // start by cropping our feature list to the working extent.
        FeatureList workingSet;
        cursor->fill( workingSet );
        CropFilter crop( 
            _options.levels()->cropFeatures() == true ? 
            CropFilter::METHOD_CROPPING : CropFilter::METHOD_CENTROID );
        context = crop.push( workingSet, context );

        if ( workingSet.size() > 0 )
        {
            // next ask the implementation to construct OSG geometry for the cell features.
            osg::ref_ptr<osg::Node> node;

            osg::ref_ptr<FeatureCursor> newCursor = new FeatureListCursor(workingSet);

            if ( _factory->createOrUpdateNode( newCursor.get(), style, context, node ) && node.valid() )
            {
                if ( !styleGroup )
                    styleGroup = new osg::Group();

                styleGroup->addChild( node.get() );
            }

            // if the method created a node, and we are building a geocentric map,
            // apply a cluster culler.
            if ( node.valid() )
            {
                const MapInfo& mi = _session->getMapInfo();

                if ( mi.isGeocentric() && _options.clusterCulling() == true )
                {
                    const SpatialReference* mapSRS = mi.getProfile()->getSRS()->getGeographicSRS();
                    GeoExtent cellExtent( extent.getSRS(), cellBounds );
                    GeoExtent mapCellExtent = cellExtent.transform( mapSRS );

                    // get the cell center as ECEF:
                    double cx, cy;
                    mapCellExtent.getCentroid( cx, cy );
                    osg::Vec3d ecefCenter;
                    mapSRS->transformToECEF( osg::Vec3d(cy, cy, 0.0), ecefCenter );

                    // get the cell corner as ECEF:
                    osg::Vec3d ecefCorner;
                    mapSRS->transformToECEF( osg::Vec3d(mapCellExtent.xMin(), mapCellExtent.yMin(), 0.0), ecefCorner );

                    // normal vector at the center of the cell:
                    osg::Vec3d normal = mapSRS->getEllipsoid()->computeLocalUpVector(
                        ecefCenter.x(), ecefCenter.y(), ecefCenter.z() );

                    // the "deviation" determines how far below the tangent plane of the cell your
                    // camera has to be before culling occurs. 0.0 is at the plane; -1.0 is 90deg
                    // below the plane (which means never cull).
                    osg::Vec3d radialVector = ecefCorner - ecefCenter;
                    double radius = radialVector.length();
                    radialVector.normalize();
                    double minDotProduct = radialVector * normal;

                    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
                    ccc->set( ecefCenter, normal, minDotProduct, radius );

                    node->setCullCallback( ccc );

                    OE_DEBUG << LC
                        << "Cell: " << mapCellExtent.toString()
                        << ": centroid = " << cx << "," << cy
                        << "; normal = " << normal.x() << "," << normal.y() << "," << normal.z()
                        << "; dev = " << minDotProduct
                        << "; radius = " << radius
                        << std::endl;
                }
            }
        }
    }

    return styleGroup;
}

#if 0
// if necessary, this method will grid up the features (according to the gridding
// policy) and then proceed to build sorted style groups for each grid cell.
osg::Group*
FeatureModelGraph::gridAndCreateNodeForStyle(const Symbology::Style* style,
                                             const Symbology::Query& query )
{
    osg::Group* styleGroup = 0L;

    // the profile of the features
    const FeatureProfile* profile = _source->getFeatureProfile();

    // get the extent of the full set of feature data:
    const GeoExtent& extent = profile->getExtent();

    // next set up a gridder/cropper:
    FeatureGridder gridder( extent.bounds(), *_options.gridding() );

    // now query the feature source once for each grid cell extent:
    for( unsigned cell = 0; cell < gridder.getNumCells(); ++cell )
    {
        Bounds cellBounds;
        if ( gridder.getCellBounds( cell, cellBounds ) )
        {
            // incorporate the cell bounds into the query:
            Query localQuery = query;
            localQuery.bounds() = query.bounds().isSet()?
                query.bounds()->unionWith( cellBounds ) :
                cellBounds;

            // query the feature source:
            osg::ref_ptr<FeatureCursor> cursor = _source->createFeatureCursor( localQuery );

            // now copy the resulting feature set into a list, converting the data
            // types along the way if a geometry override is in place:
            FeatureList cellFeatures;
            while( cursor->hasMore() )
            {
                Feature* feature = cursor->nextFeature();
                Geometry* geom = feature->getGeometry();
                if ( geom )
                {
                    // apply a type override if requested:
                    if ( _options.geometryTypeOverride().isSet() && _options.geometryTypeOverride() != geom->getComponentType() )
                    {
                        geom = geom->cloneAs( *_options.geometryTypeOverride() );
                        if ( geom )
                            feature->setGeometry( geom );
                    }
                }
                if ( geom )
                {
                    cellFeatures.push_back( feature );
                }
            }

            // cut the features so they fall completely within the cell. Note, we only need to 
            // do this is gridding is enabled.
            if ( gridder.getNumCells() > 1 )
            {
                gridder.cullFeatureListToCell( cell, cellFeatures );
            }

            if ( cellFeatures.size() > 0 )
            {
                // next ask the implementation to construct OSG geometry for the cell features.
                osg::ref_ptr<osg::Node> node;

                if ( _factory->createOrUpdateNode( cellFeatures, profile, style, _session.get(), node ) && node.valid() )
                {
                    if ( !styleGroup )
                        styleGroup = new osg::Group();

                    styleGroup->addChild( node.get() );
                }

                // if the method created a node, and we are building a geocentric map,
                // apply a cluster culler.
                if ( node.valid() )
                {
                    const MapInfo& mi = _session->getMapInfo();

                    if ( mi.isGeocentric() && _options.gridding()->clusterCulling() == true )
                    {
                        const SpatialReference* mapSRS = mi.getProfile()->getSRS()->getGeographicSRS();
                        GeoExtent cellExtent( extent.getSRS(), cellBounds );
                        GeoExtent mapCellExtent = cellExtent.transform( mapSRS );

                        // get the cell center as ECEF:
                        double cx, cy;
                        mapCellExtent.getCentroid( cx, cy );
                        osg::Vec3d ecefCenter;
                        mapSRS->transformToECEF( osg::Vec3d(cy, cy, 0.0), ecefCenter );

                        // get the cell corner as ECEF:
                        osg::Vec3d ecefCorner;
                        mapSRS->transformToECEF( osg::Vec3d(mapCellExtent.xMin(), mapCellExtent.yMin(), 0.0), ecefCorner );

                        // normal vector at the center of the cell:
                        osg::Vec3d normal = mapSRS->getEllipsoid()->computeLocalUpVector(
                            ecefCenter.x(), ecefCenter.y(), ecefCenter.z() );

                        // the "deviation" determines how far below the tangent plane of the cell your
                        // camera has to be before culling occurs. 0.0 is at the plane; -1.0 is 90deg
                        // below the plane (which means never cull).
                        osg::Vec3d radialVector = ecefCorner - ecefCenter;
                        double radius = radialVector.length();
                        radialVector.normalize();
                        double minDotProduct = radialVector * normal;

                        osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
                        ccc->set( ecefCenter, normal, minDotProduct, radius );

                        node->setCullCallback( ccc );

                        OE_NOTICE
                            << "Cell: " << mapCellExtent.toString()
                            << ": centroid = " << cx << "," << cy
                            << "; normal = " << normal.x() << "," << normal.y() << "," << normal.z()
                            << "; dev = " << minDotProduct
                            << "; radius = " << radius
                            << std::endl;
                    }
                }
            }
        }
    }

    // run the SpatializeGroups optimization pass on the result
    if ( styleGroup && _options.gridding()->spatializeGroups() == true )
    {
        //OE_DEBUG << LC << context->getModelSource()->getName() << ": running spatial optimization" << std::endl;
        osgUtil::Optimizer optimizer;
        optimizer.optimize( styleGroup, osgUtil::Optimizer::SPATIALIZE_GROUPS );
    }

    return styleGroup;
}
#endif 