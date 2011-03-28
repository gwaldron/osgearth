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
#include <osgEarthFeatures/FeatureGridder>
#include <osg/BlendFunc>
#include <osg/NodeVisitor>
#include <osg/ClusterCullingCallback>
#include <osgUtil/Optimizer>

#define LC "[FeatureModelGraph] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

FeatureModelGraph::FeatureModelGraph(FeatureSource*                   source,
                                     const FeatureModelSourceOptions& options,
                                     FeatureNodeFactory*              factory) :
_source( source ),
_options( options ),
_factory( factory )
{
    osg::StateSet* stateSet = getOrCreateStateSet();

    if ( _options.enableLighting().isSet() )
        stateSet->setMode( GL_LIGHTING, *_options.enableLighting() ? 1 : 0 );
}

void
FeatureModelGraph::update( Session* session, const StyleSheet& styles )
{
    removeChildren( 0, getNumChildren() );

    if ( _source->hasEmbeddedStyles() )
    {
        const FeatureProfile* profile = _source->getFeatureProfile();

        // each feature has its own style, so use that and ignore the style catalog.
        osg::ref_ptr<FeatureCursor> cursor = _source->createFeatureCursor( Query() );
        while( cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );

                // note: gridding is not supported for embedded styles.
                osg::ref_ptr<osg::Node> node;
                if ( _factory->createOrUpdateNode( list, profile, feature->style().get(), session, node ) )
                {
                    addChild( node );
                }
            }
        }
    }

    else
    {
        // if we have selectors, sort the features into style groups and create a node for each group.
        if ( styles.selectors().size() > 0 )
        {
            for( StyleSelectorList::const_iterator i = styles.selectors().begin(); i != styles.selectors().end(); ++i )
            {
                const StyleSelector& sel = *i;
                Style* style;
                styles.getStyle( sel.getSelectedStyleName(), style );

                osg::Group* styleGroup = gridAndCreateNodeForStyle( style, *sel.query(), session );
                if ( styleGroup )
                    addChild( styleGroup );
            }
        }

        // otherwise, render all the features with a single style
        else
        {
            const Style* style = styles.getDefaultStyle();
            osg::Group* styleGroup = gridAndCreateNodeForStyle( style, Query(), session );
            if ( styleGroup )
                addChild( styleGroup );
        }
    }
}

// if necessary, this method will grid up the features (according to the gridding
// policy) and then proceed to build sorted style groups for each grid cell.
osg::Group*
FeatureModelGraph::gridAndCreateNodeForStyle(const Symbology::Style* style,
                                             const Symbology::Query& query,
                                             Session*                session )
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

                if ( _factory->createOrUpdateNode( cellFeatures, profile, style, session, node ) && node.valid() )
                {
                    if ( !styleGroup )
                        styleGroup = new osg::Group();

                    styleGroup->addChild( node.get() );
                }

                // if the method created a node, and we are building a geocentric map,
                // apply a cluster culler.
                if ( node.valid() )
                {
                    const MapInfo& mi = session->getMapInfo();

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
                        //mapSRS->transformToECEF( cx, cy, 0.0, ecefCenter.x(), ecefCenter.y(), ecefCenter.z() );

                        // get the cell corner as ECEF:
                        osg::Vec3d ecefCorner;
                        mapSRS->transformToECEF( osg::Vec3d(mapCellExtent.xMin(), mapCellExtent.yMin(), 0.0), ecefCorner );
                        //mapSRS->transformToECEF(
                        //    mapCellExtent.xMin(), mapCellExtent.yMin(), 0.0,
                        //    ecefCorner.x(), ecefCorner.y(), ecefCorner.z() );

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
        //OE_INFO << LC << context->getModelSource()->getName() << ": running spatial optimization" << std::endl;
        osgUtil::Optimizer optimizer;
        optimizer.optimize( styleGroup, osgUtil::Optimizer::SPATIALIZE_GROUPS );
    }

    return styleGroup;
}
