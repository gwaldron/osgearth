/* -*-c++-*- */
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
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarthFeatures/FeatureSourceNode>
#include <osgEarthFeatures/FeatureSourceMeshConsolidator>
#include <osgEarth/HTTPClient>
#include <osgEarth/ECEF>
#include <osg/AutoTransform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osgUtil/Optimizer>
#include <list>
#include <deque>

#define LC "[SubstituteModelFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
    static osg::Node* s_defaultModel =0L;
}

//------------------------------------------------------------------------

SubstituteModelFilter::SubstituteModelFilter( const Style& style ) :
_style   ( style ),
_cluster ( false ),
_merge   ( true )
{
    //NOP
}

bool
SubstituteModelFilter::process(const FeatureList&           features,                               
                               const MarkerSymbol*          symbol,
                               Session*                     session,
                               osg::Group*                  attachPoint,
                               FilterContext&               context )
{
    bool makeECEF = context.getSession()->getMapInfo().isGeocentric();

    // first, go through the features and build the model cache. Apply the model matrix' scale
    // factor to any AutoTransforms directly (cloning them as necessary)
    std::map< std::pair<URI, float>, osg::ref_ptr<osg::Node> > uniqueModels;
    //std::map< Feature*, osg::ref_ptr<osg::Node> > featureModels;

    StringExpression  uriEx   = *symbol->url();
    NumericExpression scaleEx = *symbol->scale();
    FeatureSource * source = (session!=NULL)?session->getFeatureSource():NULL;

    for( FeatureList::const_iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        // evaluate the marker URI expression:
        StringExpression uriEx = *symbol->url();
        URI markerURI( input->eval(uriEx, &context), uriEx.uriContext() );

        // find the corresponding marker in the cache
        MarkerResource* marker = 0L;
        MarkerCache::Record rec = _markerCache.get( markerURI );
        if ( rec.valid() ) {
            marker = rec.value();
        }
        else {
            marker = new MarkerResource();
            marker->uri() = markerURI;
            _markerCache.insert( markerURI, marker );
        }

        // evalute the scale expression (if there is one)
        float scale = 1.0f;
        osg::Matrixd scaleMatrix;

        if ( symbol->scale().isSet() )
        {
            scale = input->eval( scaleEx, &context );
            if ( scale == 0.0 )
                scale = 1.0;
            scaleMatrix = osg::Matrix::scale( scale, scale, scale );
        }

        // how that we have a marker source, create a node for it
        std::pair<URI,float> key( markerURI, scale );
        osg::ref_ptr<osg::Node>& model = uniqueModels[key];
        if ( !model.valid() )
        {
            model = context.resourceCache()->getMarkerNode( marker );

            if ( scale != 1.0f && dynamic_cast<osg::AutoTransform*>( model.get() ) )
            {
                // clone the old AutoTransform, set the new scale, and copy over its children.
                osg::AutoTransform* oldAT = dynamic_cast<osg::AutoTransform*>(model.get());
                osg::AutoTransform* newAT = osg::clone( oldAT );

                // make a scaler and put it between the new AutoTransform and its kids
                osg::MatrixTransform* scaler = new osg::MatrixTransform(osg::Matrix::scale(scale,scale,scale));
                for( unsigned i=0; i<newAT->getNumChildren(); ++i )
                    scaler->addChild( newAT->getChild(0) );
                newAT->removeChildren(0, newAT->getNumChildren());
                newAT->addChild( scaler );
                model = newAT;
            }
        }

        if ( model.valid() )
        {
            GeometryIterator gi( input->getGeometry(), false );
            while( gi.hasMore() )
            {
                Geometry* geom = gi.next();

                for( unsigned i=0; i<geom->size(); ++i )
                {
                    osg::Matrixd mat;

                    osg::Vec3d point = (*geom)[i];
                    if ( makeECEF )
                    {
                        // the "rotation" element lets us re-orient the instance to ensure it's pointing up. We
                        // could take a shortcut and just use the current extent's local2world matrix for this,
                        // but if the tile is big enough the up vectors won't be quite right.
                        osg::Matrixd rotation;
                        ECEF::transformAndGetRotationMatrix( point, context.profile()->getSRS(), point, rotation );
                        mat = rotation * scaleMatrix * osg::Matrixd::translate( point ) * _world2local;
                    }
                    else
                    {
                        mat = scaleMatrix * osg::Matrixd::translate( point ) * _world2local;
                    }

                    FeatureSourceNode* xform = new FeatureSourceNode(source, input->getFID());
                    xform->setMatrix( mat );
                    xform->setDataVariance( osg::Object::STATIC );
                    xform->addChild( model.get() );
                    attachPoint->addChild( xform );

                    // name the feature if necessary
                    if ( !_featureNameExpr.empty() )
                    {
                        const std::string& name = input->eval( _featureNameExpr, &context);
                        if ( !name.empty() )
                            xform->setName( name );
                    }
                }
            }
        }
    }

    return true;
}




struct ClusterVisitor : public osg::NodeVisitor
{
        ClusterVisitor( const FeatureList& features, const MarkerSymbol* symbol, FeaturesToNodeFilter* f2n, FeatureSourceMultiNode * featureNode, FilterContext& cx )
            : _features   ( features ),
              _symbol     ( symbol ),
              //_modelMatrix( modelMatrix ),
              _f2n        ( f2n ),
              _cx         ( cx ),
              _featureNode(featureNode),
              osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
        {
            //nop
        }

        void apply( osg::Geode& geode )
        {
            bool makeECEF = _cx.getSession()->getMapInfo().isGeocentric();
            const SpatialReference* srs = _cx.profile()->getSRS();

            NumericExpression scaleEx = *_symbol->scale();
            osg::Matrixd scaleMatrix;

            // save the geode's drawables..
            osg::Geode::DrawableList old_drawables = geode.getDrawableList();

            //OE_DEBUG << "ClusterVisitor geode " << &geode << " featureNode=" << _featureNode << " drawables=" << old_drawables.size() << std::endl;

            // ..and clear out the drawables list.
            geode.removeDrawables( 0, geode.getNumDrawables() );
            // ... and remove all drawables from the feature node
            for( osg::Geode::DrawableList::iterator i = old_drawables.begin(); i != old_drawables.end(); i++ )
                _featureNode->removeDrawable(i->get());

            // foreach each drawable that was originally in the geode...
            for( osg::Geode::DrawableList::iterator i = old_drawables.begin(); i != old_drawables.end(); i++ )
            {
                osg::Geometry* originalDrawable = dynamic_cast<osg::Geometry*>( i->get() );
                if ( !originalDrawable )
                    continue;

                // go through the list of input features...
                for( FeatureList::const_iterator j = _features.begin(); j != _features.end(); j++ )
                {
                    const Feature* feature = j->get();

                    if ( _symbol->scale().isSet() )
                    {
                        double scale = feature->eval( scaleEx, &_cx );
                        scaleMatrix.makeScale( scale, scale, scale );
                    }

                    ConstGeometryIterator gi( feature->getGeometry(), false );
                    while( gi.hasMore() )
                    {
                        const Geometry* geom = gi.next();

                        for( Geometry::const_iterator k = geom->begin(); k != geom->end(); ++k )
                        {
                            osg::Vec3d   point = *k;
                            osg::Matrixd mat;

                            if ( makeECEF )
                            {
                                osg::Matrixd rotation;
                                ECEF::transformAndGetRotationMatrix( point, srs, point, rotation );
                                mat = rotation * scaleMatrix * osg::Matrixd::translate(point) * _f2n->world2local();
                            }
                            else
                            {
                                mat = scaleMatrix * osg::Matrixd::translate(point) * _f2n->world2local();
                            }

                            // clone the source drawable once for each input feature.
                            osg::ref_ptr<osg::Geometry> newDrawable = osg::clone( 
                                originalDrawable, 
                                osg::CopyOp::DEEP_COPY_ARRAYS | osg::CopyOp::DEEP_COPY_PRIMITIVES );

                            osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( newDrawable->getVertexArray() );
                            if ( verts )
                            {
                                for( osg::Vec3Array::iterator v = verts->begin(); v != verts->end(); ++v )
                                {
                                    (*v).set( (*v) * mat );
                                }
                                
                                // add the new cloned, translated drawable back to the geode.
                                geode.addDrawable( newDrawable.get() );
                                _featureNode->addDrawable(newDrawable.get(), feature->getFID());
                            }
                        }

                    }
                }
            }

            geode.dirtyBound();

            FeatureSourceMeshConsolidator::run( geode, _featureNode );

            // merge the geometry. Not sure this is necessary
            osgUtil::Optimizer opt;
            opt.optimize( &geode, osgUtil::Optimizer::MERGE_GEOMETRY | osgUtil::Optimizer::SHARE_DUPLICATE_STATE );
            
            osg::NodeVisitor::apply( geode );
        }

    private:
        const FeatureList&    _features;
        FilterContext         _cx;
        const MarkerSymbol*   _symbol;
        //osg::Matrixd          _modelMatrix;
        FeaturesToNodeFilter* _f2n;
        FeatureSourceMultiNode * _featureNode;
    };


typedef std::map< osg::Node*, FeatureList > MarkerToFeatures;

//clustering:
//  troll the external model for geodes. for each geode, create a geode in the target
//  model. then, for each geometry in that geode, replicate it once for each instance of
//  the model in the feature batch and transform the actual verts to a local offset
//  relative to the tile centroid. Finally, reassemble all the geodes and optimize. 
//  hopefully stateset sharing etc will work out. we may need to strip out LODs too.
bool
SubstituteModelFilter::cluster(const FeatureList&           features,
                               const MarkerSymbol*          symbol, 
                               Session*                     session,
                               osg::Group*                  attachPoint,
                               FilterContext&               context )
{    
    MarkerToFeatures markerToFeatures;

    // first, sort the features into buckets, each bucket corresponding to a
    // unique marker.
    for (FeatureList::const_iterator i = features.begin(); i != features.end(); ++i)
    {
        Feature* f = i->get();

        // resolve the URI for the marker:
        StringExpression uriEx( *symbol->url() );
        URI markerURI( f->eval( uriEx, &context ), uriEx.uriContext() );

        // find and load the corresponding marker model. We're using the session-level
        // object store to cache models. This is thread-safe sine we are always going
        // to CLONE the model before using it.
        osg::Node* model = context.getSession()->getObject<osg::Node>( markerURI.full() );
        if ( !model )
        {
            osg::ref_ptr<MarkerResource> mres = new MarkerResource();
            mres->uri() = markerURI;
            model = mres->createNode( context.getSession()->getDBOptions() );
            if ( model )
            {
                context.getSession()->putObject( markerURI.full(), model );
            }
        }

        if ( model )
        {
            MarkerToFeatures::iterator itr = markerToFeatures.find( model );
            if (itr == markerToFeatures.end())
                markerToFeatures[ model ].push_back( f );
            else
                itr->second.push_back( f );
        }
    }

    FeatureSource * source = (session!=NULL)?session->getFeatureSource():NULL;

    //For each model, cluster the features that use that marker
    for (MarkerToFeatures::iterator i = markerToFeatures.begin(); i != markerToFeatures.end(); ++i)
    {
        osg::Node* prototype = i->first;

        // we're using the Session cache since we know we'll be cloning.
        if ( prototype )
        {
            osg::Node* clone = osg::clone( prototype, osg::CopyOp::DEEP_COPY_ALL );

            FeatureSourceMultiNode * featureNode = new FeatureSourceMultiNode(source);

            // ..and apply the clustering to the copy.
            ClusterVisitor cv( i->second, symbol, this, featureNode, context );
            clone->accept( cv );

            featureNode->addChild(clone);
            attachPoint->addChild( featureNode );
        }
    }

    return true;
}

osg::Node*
SubstituteModelFilter::push(FeatureList& features, FilterContext& context)
{
    if ( !isSupported() ) {
        OE_WARN << "SubstituteModelFilter support not enabled" << std::endl;
        return 0L;
    }

    if ( _style.empty() ) {
        OE_WARN << LC << "Empty style; cannot process features" << std::endl;
        return 0L;
    }

    const MarkerSymbol* symbol = _style.getSymbol<MarkerSymbol>();
    if ( !symbol ) {
        OE_WARN << LC << "No MarkerSymbol found in style; cannot process feautres" << std::endl;
        return 0L;
    }

    FilterContext newContext( context );

    computeLocalizers( context );

    osg::Group* group = createDelocalizeGroup();

    bool ok = true;

    if ( _cluster )
    {
        ok = cluster( features, symbol, context.getSession(), group, newContext );
    }

    else
    {
        process( features, symbol, context.getSession(), group, newContext );

#if 0
        // speeds things up a bit, at the expense of creating tons of geometry..

        // this optimizer pass will remove all the MatrixTransform nodes that we
        // used to offset each instance
        osgUtil::Optimizer optimizer;
        optimizer.optimize( group, osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS_DUPLICATING_SHARED_SUBGRAPHS );
#endif

    }

    return group;
}
