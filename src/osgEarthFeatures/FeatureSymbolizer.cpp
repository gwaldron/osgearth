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

#include <osgEarthFeatures/FeatureSymbolizer>
#include <osgEarthFeatures/Feature>
#include <osgEarthSymbology/SymbolicNode>
#include <osg/NodeVisitor>
#include <osg/ClusterCullingCallback>
#include <osgUtil/Optimizer>

#define LC "[FeatureSymbolizer] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

struct FeatureSymbolizerGraphUpdate : public osg::NodeCallback
{
    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        static_cast<FeatureSymbolizerGraph*>( node )->update();
        traverse( node, nv );
    }
};

struct CompileSymbolizersVisitor : public osg::NodeVisitor
{
    CompileSymbolizersVisitor()
        : osg::NodeVisitor( osg::NodeVisitor::UPDATE_VISITOR, osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ) { }

    void apply( osg::Group& group )
    {
        SymbolicNodeBase* sym = dynamic_cast<SymbolicNodeBase*>( &group );
        if ( sym )
        {
            sym->updateSymbology();
        }
        osg::NodeVisitor::traverse( group );
    }
};

FeatureSymbolizerGraph::FeatureSymbolizerGraph( SymbolizerFactory* factory ) :
_factory(factory),
_dirty(true)
{
    // to force the update traversal safely
    this->setUpdateCallback( new FeatureSymbolizerGraphUpdate() );
}

void
FeatureSymbolizerGraph::compile()
{
    update();
    CompileSymbolizersVisitor visitor;
    this->accept( visitor );
}

void
FeatureSymbolizerGraph::update()
{
    if (_dirty && _factory.valid())
    {
        removeChildren(0, getNumChildren());

        //osg::Timer_t start = osg::Timer::instance()->tick();

        FeatureModelSource* model = _factory->getFeatureModelSource();
        // implementation-specific data
        osg::ref_ptr<osg::Referenced> buildData = model->createBuildData();
        osg::ref_ptr<FeatureSymbolizerContext> context = new FeatureSymbolizerContext(model, buildData);

        const optional<StyleCatalog>& styles = model->getFeatureModelOptions().styles();

        // figure out if and how to style the geometry.
        if ( model->getFeatureSource()->hasEmbeddedStyles() )
        {
            // Each feature has its own embedded style data, so use that:
            osg::ref_ptr<FeatureCursor> cursor = model->getFeatureSource()->createFeatureCursor( Query() );
            while( cursor->hasMore() )
            {
                Feature* feature = cursor->nextFeature();
                if ( feature )
                {
                    FeatureList list;
                    list.push_back( feature );
                    // gridding is not supported for embedded styles.
                    osg::Node* node = createSymbolizerNode(feature->style().get().get(), list, context.get());
                    if ( node )
                        addChild( node );
                }
            }
        }
        else if ( styles.isSet() )
        {
            if ( styles->selectors().size() > 0 )
            {
                for( StyleSelectorList::const_iterator i = styles->selectors().begin(); i != styles->selectors().end(); ++i )
                {
                    const StyleSelector& sel = *i;
                    Style* style;
                    styles->getStyle( sel.getSelectedStyleName(), style );
                    osg::Node* node = createGridSymbolizerNode( style, sel.query().value(), context.get());
                    if ( node )
                        addChild( node );
                }
            }
            else
            {
                const Style* style = styles->getDefaultStyle();
                osg::Node* node = createGridSymbolizerNode(style, Query(), context.get());
                if ( node )
                    addChild( node );
            }
        }
        else
        {
            osg::Node* node = createGridSymbolizerNode( new Style, Query(), context.get());
            if ( node )
                addChild( node );
        }
    }
    _dirty = false;
}


osg::Node* FeatureSymbolizerGraph::createGridSymbolizerNode(
    const Symbology::Style* style,
    const Symbology::Query& query,
    FeatureSymbolizerContext* context )
{
    FeatureSymbolicNode* node = new FeatureSymbolicNode();
    
    node->setSymbolizer( new GridFeatureSymbolizer(_factory.get(), query) );
    node->getState()->setContext( context );
    node->getState()->setStyle(style);

    return node;
}

osg::Node* FeatureSymbolizerGraph::createSymbolizerNode(
    const Symbology::Style* style, 
    const FeatureList& features, 
    FeatureSymbolizerContext* ctx )
{
    FeatureSymbolicNode* node = new FeatureSymbolicNode();
    FeatureSymbolizer* symbolizer = new FeatureSymbolizer(_factory.get());
    FeatureContent* input = new FeatureContent(features);

    node->setSymbolizer(symbolizer);
    node->getState()->setContext(ctx);

    node->getState()->setContent(input);
    node->getState()->setStyle(style);

    return node;
}

bool FeatureSymbolizer::compile(FeatureSymbolizerState* state,
                                osg::Group* attachPoint )
{
    if ( !state || !attachPoint || !state->getContent() || !state->getStyle() )
        return false;

    FeatureSymbolizerContext* ctx = dynamic_cast<FeatureSymbolizerContext*>(state->getContext());
    if (!ctx || !_factory.valid())
        return false;
    
    osg::Node* result = _factory->createNodeForStyle(state->getStyle(), state->getContent()->getFeatures(), ctx);
    if (result) {
        attachPoint->removeChildren(0, attachPoint->getNumChildren());
        attachPoint->addChild(result);
        return true;
    }
    return false;
}

//NOTE:
// this symbolizer is currently based on the GeometryContent content type,
// but it doesn't really take any content. It relies on the data source in
// the context object. We should refactor this.
bool GridFeatureSymbolizer::compile(FeatureSymbolizerState* state,
                                    osg::Group* attachPoint)
{
    if ( !state || !attachPoint || !state->getStyle() || !state->getContext() )
        return false;

    FeatureSymbolizerContext* ctx = dynamic_cast<FeatureSymbolizerContext*>(state->getContext());
    if (!ctx)
        return false;
    
    osg::Node* result = gridAndCreateNodeForStyle(state->getStyle(), _query, ctx);
    if (result) {
        attachPoint->removeChildren(0, attachPoint->getNumChildren());
        attachPoint->addChild(result);
        return true;
    }
    return false;
}

osg::Group* GridFeatureSymbolizer::gridAndCreateNodeForStyle(
    const Symbology::Style* style,
    const Symbology::Query& query,
    FeatureSymbolizerContext* context)
{
    osg::Group* styleGroup = 0L;

    // first we need the overall extent of the layer:
    const GeoExtent& extent = context->getModelSource()->getFeatureSource()->getFeatureProfile()->getExtent();
    const FeatureModelSourceOptions& options = context->getModelSource()->getFeatureModelOptions();

    //osg::ref_ptr<GriddingPolicy> gridding = options.gridding().valid() ? options.gridding().get() : new GriddingPolicy();

    // next set up a gridder/cropper:
    FeatureGridder gridder( extent.bounds(), *options.gridding() );

    if ( gridder.getNumCells() > 1 )
    {
        OE_NOTICE
            << context->getModelSource()->getName() << ": grid cells = " << gridder.getNumCells()
            << std::endl;
    }

    // now query the feature source once for each grid cell extent:
    for( int cell=0; cell<gridder.getNumCells(); ++cell )
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
            osg::ref_ptr<FeatureCursor> cursor = context->getModelSource()->getFeatureSource()->createFeatureCursor( localQuery );

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
                    if ( options.geometryTypeOverride().isSet() && options.geometryTypeOverride() != geom->getComponentType() )
                    {
                        geom = geom->cloneAs( options.geometryTypeOverride().value() );
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
                // note: just b/c render* returns NULL does not mean it didn't generate anything.
                // Some implementations might return a group node on the first pass and add children
                // to it on subsequent passes.
                osg::Node* createdNode = 0L;
                osg::Node* nodeToAdd = _factory->createNodeForStyle( style, cellFeatures, context, &createdNode );
                if ( nodeToAdd )
                {
                    if ( !styleGroup )
                        styleGroup = new osg::Group();

                    styleGroup->addChild( nodeToAdd );
                }

                // if the method created a node, apply a cluter culler to it if neceesary:
                if ( createdNode )
                {
                    if ( context->getModelSource()->getMap()->isGeocentric() && options.gridding()->clusterCulling() == true )
                    {
                        const SpatialReference* mapSRS = context->getModelSource()->getMap()->getProfile()->getSRS()->getGeographicSRS();
                        GeoExtent cellExtent( extent.getSRS(), cellBounds );
                        GeoExtent mapCellExtent = cellExtent.transform( mapSRS );

                        double cx, cy;
                        mapCellExtent.getCentroid( cx, cy );

                        osg::Vec3d geoc_center;
                        mapSRS->getEllipsoid()->convertLatLongHeightToXYZ(
                            osg::DegreesToRadians( cy ), osg::DegreesToRadians( cx ), 0,
                            geoc_center.x(), geoc_center.y(), geoc_center.z() );

                        osg::Vec3d geoc_corner;
                        mapSRS->getEllipsoid()->convertLatLongHeightToXYZ(
                            osg::DegreesToRadians( mapCellExtent.yMin() ), osg::DegreesToRadians( mapCellExtent.xMin()), 0,
                            geoc_corner.x(), geoc_corner.y(), geoc_corner.z() );

                        osg::Vec3d normal = mapSRS->getEllipsoid()->computeLocalUpVector(
                            geoc_center.x(), geoc_center.y(), geoc_center.z() );

                        // the "deviation" determines how far below the tangent plane of the cell your
                        // camera has to be before culling occurs. 0.0 is at the plane; -1.0 is 90deg
                        // below the plane (which means never cull).
                        osg::Vec3d radialVector = geoc_corner - geoc_center;
                        double radius = radialVector.length();
                        radialVector.normalize();
                        double minDotProduct = radialVector * normal;

                        osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
                        ccc->set( geoc_center, normal, minDotProduct, radius );

                        createdNode->setCullCallback( ccc );

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
    if ( styleGroup && options.gridding()->spatializeGroups() == true )
    {
        //OE_INFO << LC << context->getModelSource()->getName() << ": running spatial optimization" << std::endl;
        osgUtil::Optimizer optimizer;
        optimizer.optimize( styleGroup, osgUtil::Optimizer::SPATIALIZE_GROUPS );
    }

    return styleGroup;
}




