/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/ECEF>
#include <osgEarth/VirtualProgram>
#include <osgEarth/DrawInstanced>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Decluttering>
#include <osgEarth/CullingUtils>

#include <osg/AutoTransform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>
#include <osg/AlphaFunc>

#include <osgDB/FileNameUtils>
#include <osgDB/Registry>

#include <osgUtil/Optimizer>
#include <osgUtil/MeshOptimizers>

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

    struct SetSmallFeatureCulling : public osg::NodeCallback
    {
        bool _value;
        SetSmallFeatureCulling(bool value) : _value(value) { }
        void operator()(osg::Node* node, osg::NodeVisitor* nv) {
            Culling::asCullVisitor(nv)->setSmallFeatureCullingPixelSize(-1.0f);
            traverse(node, nv);
        }
    };
}

//------------------------------------------------------------------------

SubstituteModelFilter::SubstituteModelFilter( const Style& style ) :
_style                ( style ),
_cluster              ( false ),
_useDrawInstanced     ( false ),
_merge                ( true ),
_normalScalingRequired( false ),
_instanceCache        ( false )     // cache per object so MT not required
{
    //NOP
}

bool
SubstituteModelFilter::findResource(const URI&            uri,
                                    const InstanceSymbol* symbol,
                                    FilterContext&        context, 
                                    std::set<URI>&        missing,
                                    osg::ref_ptr<InstanceResource>& output )
{
    // be careful about refptrs here since _instanceCache is an LRU.

    InstanceCache::Record rec;
    if ( _instanceCache.get(uri, rec) )
    {
        // found it in the cache:
        output = rec.value().get();
    }
    else if ( _resourceLib.valid() )
    {
        // look it up in the resource library:
        output = _resourceLib->getInstance( uri.base(), context.getDBOptions() );
    }
    else
    {
        // create it on the fly:
        output = symbol->createResource();
        output->uri() = uri;
        _instanceCache.insert( uri, output.get() );
    }

    // failed to find the instance.
    if ( !output.valid() )
    {
        if ( missing.find(uri) == missing.end() )
        {
            missing.insert(uri);
            OE_WARN << LC << "Failed to locate resource: " << uri.full() << std::endl;
        }
    }

    return output.valid();
}

bool
SubstituteModelFilter::process(const FeatureList&           features,
                               const InstanceSymbol*        symbol,
                               Session*                     session,
                               osg::Group*                  attachPoint,
                               FilterContext&               context )
{
    // Establish SRS information:
    bool makeECEF = context.getSession()->getMapInfo().isGeocentric();
    const SpatialReference* targetSRS = context.getSession()->getMapInfo().getSRS();

    // first, go through the features and build the model cache. Apply the model matrix' scale
    // factor to any AutoTransforms directly (cloning them as necessary)
    std::map< std::pair<URI, float>, osg::ref_ptr<osg::Node> > uniqueModels;

    // URI cache speeds up URI creation since it can be slow.
    osgEarth::fast_map<std::string, URI> uriCache;

    // keep track of failed URIs so we don't waste time or warning messages on them
    std::set< URI > missing;

    StringExpression  uriEx   = *symbol->url();
    NumericExpression scaleEx = *symbol->scale();

    const ModelSymbol* modelSymbol = dynamic_cast<const ModelSymbol*>(symbol);
    const IconSymbol*  iconSymbol  = dynamic_cast<const IconSymbol*> (symbol);

    NumericExpression headingEx;
    if ( modelSymbol )
        headingEx = *modelSymbol->heading();

    for( FeatureList::const_iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        // Run a feature pre-processing script.
        if ( symbol->script().isSet() )
        {
            StringExpression scriptExpr(symbol->script().get());
            input->eval( scriptExpr, &context );
        }

		// evaluate the instance URI expression:
		const std::string& st = input->eval(uriEx, &context);
		URI& instanceURI = uriCache[st];
		if(instanceURI.empty()) // Create a map, to reuse URI's, since they take a long time to create
		{
			instanceURI = URI( st, uriEx.uriContext() );
		}

        // find the corresponding marker in the cache
        osg::ref_ptr<InstanceResource> instance;
        if ( !findResource(instanceURI, symbol, context, missing, instance) )
            continue;

        // evalute the scale expression (if there is one)
        float scale = 1.0f;
        osg::Matrixd scaleMatrix;
        if ( symbol->scale().isSet() )
        {
            scale = input->eval( scaleEx, &context );
            if ( scale == 0.0 )
                scale = 1.0;
            if ( scale != 1.0 )
                _normalScalingRequired = true;
            scaleMatrix = osg::Matrix::scale( scale, scale, scale );
        }
        
        osg::Matrixd rotationMatrix;
        if ( modelSymbol && modelSymbol->heading().isSet() )
        {
            float heading = input->eval(headingEx, &context);
            rotationMatrix.makeRotate( osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0,0,1)) );
        }

		// how that we have a marker source, create a node for it
		std::pair<URI,float> key( instanceURI, iconSymbol? scale : 1.0f );//use 1.0 for models, since we don't want unique models based on scaling

        // cache nodes per instance.
        osg::ref_ptr<osg::Node>& model = uniqueModels[key];
        if ( !model.valid() )
        {
            // Always clone the cached instance so we're not processing data that's
            // already in the scene graph. -gw
            context.resourceCache()->cloneOrCreateInstanceNode(instance.get(), model);

            // if icon decluttering is off, install an AutoTransform.
            if ( iconSymbol )
            {
                if ( iconSymbol->declutter() == true )
                {
                    Decluttering::setEnabled( model->getOrCreateStateSet(), true );
                }
                else if ( dynamic_cast<osg::AutoTransform*>(model.get()) == 0L )
                {
                    osg::AutoTransform* at = new osg::AutoTransform();
                    at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
                    at->setAutoScaleToScreen( true );
                    at->addChild( model );
                    model = at;
                }
            }
        }

        if ( model.valid() )
        {
            GeometryIterator gi( input->getGeometry(), false );
            while( gi.hasMore() )
            {
                Geometry* geom = gi.next();

                // if necessary, transform the points to the target SRS:
                if ( !makeECEF && !targetSRS->isEquivalentTo(context.profile()->getSRS()) )
                {
                    context.profile()->getSRS()->transform( geom->asVector(), targetSRS );
                }

                for( unsigned i=0; i<geom->size(); ++i )
                {
                    osg::Matrixd mat;

                    // need to recalcluate expression-based data per-point, not just per-feature!
                    if ( symbol->scale().isSet() )
                    {
                        scale = input->eval(scaleEx, &context);
                        if ( scale == 0.0 )
                            scale = 1.0;
                        if ( scale != 1.0 )
                            _normalScalingRequired = true;
                        scaleMatrix = osg::Matrix::scale( scale, scale, scale );
                    }

                    if ( modelSymbol->heading().isSet() )
                    {
                        float heading = input->eval(headingEx, &context);
                        rotationMatrix.makeRotate( osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0,0,1)) );
                    }

                    osg::Vec3d point = (*geom)[i];
                    if ( makeECEF )
                    {
                        // the "rotation" element lets us re-orient the instance to ensure it's pointing up. We
                        // could take a shortcut and just use the current extent's local2world matrix for this,
                        // but if the tile is big enough the up vectors won't be quite right.
                        osg::Matrixd rotation;
                        ECEF::transformAndGetRotationMatrix( point, context.profile()->getSRS(), point, targetSRS, rotation );
                        mat = rotationMatrix * rotation * scaleMatrix * osg::Matrixd::translate( point ) * _world2local;
                    }
                    else
                    {
                        mat = rotationMatrix * scaleMatrix *  osg::Matrixd::translate( point ) * _world2local;
                    }

                    osg::MatrixTransform* xform = new osg::MatrixTransform();
                    xform->setMatrix( mat );
                    xform->setDataVariance( osg::Object::STATIC );
                    xform->addChild( model.get() );
                    attachPoint->addChild( xform );

                    if ( context.featureIndex() && !_useDrawInstanced )
                    {
                        context.featureIndex()->tagNode( xform, input );
                    }

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

    if ( iconSymbol )
    {
        // activate decluttering for icons if requested
        if ( iconSymbol->declutter() == true )
        {
            Decluttering::setEnabled( attachPoint->getOrCreateStateSet(), true );
        }

        // activate horizon culling if we are in geocentric space
        if ( context.getSession() && context.getSession()->getMapInfo().isGeocentric() )
        {
            HorizonCullingProgram::install( attachPoint->getOrCreateStateSet() );
        }
    }

    // active DrawInstanced if required:
    if ( _useDrawInstanced && Registry::capabilities().supportsDrawInstanced() )
    {
        DrawInstanced::convertGraphToUseDrawInstanced( attachPoint );

        // install a shader program to render draw-instanced.
        DrawInstanced::install( attachPoint->getOrCreateStateSet() );
    }

    return true;
}




struct ClusterVisitor : public osg::NodeVisitor
{
    ClusterVisitor( const FeatureList& features, const InstanceSymbol* symbol, FeaturesToNodeFilter* f2n, FilterContext& cx )
        : _features   ( features ),
          _symbol     ( symbol ),
          _f2n        ( f2n ),
          _cx         ( cx ),
          osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
    {
        _modelSymbol = dynamic_cast<const ModelSymbol*>( symbol );
        if ( _modelSymbol )
            _headingExpr = *_modelSymbol->heading();

        _scaleExpr = *_symbol->scale();

        _makeECEF  = _cx.getSession()->getMapInfo().isGeocentric();
        _srs       = _cx.profile()->getSRS();
        _targetSRS = _cx.getSession()->getMapInfo().getSRS();
    }

    void apply( osg::Geode& geode )
    {
        // save the geode's drawables..
        typedef std::vector<osg::ref_ptr<osg::Drawable> > Drawables;        
        Drawables old_drawables;
        old_drawables.reserve( geode.getNumDrawables() );
        for(unsigned i=0; i<geode.getNumDrawables(); ++i)
            old_drawables.push_back( geode.getDrawable(i) );

        //OE_DEBUG << "ClusterVisitor geode " << &geode << " featureNode=" << _featureNode << " drawables=" << old_drawables.size() << std::endl;

        // ..and clear out the drawables list.
        geode.removeDrawables( 0, geode.getNumDrawables() );

        // foreach each drawable that was originally in the geode...
        for( Drawables::iterator i = old_drawables.begin(); i != old_drawables.end(); i++ )
        {
            osg::Geometry* originalDrawable = dynamic_cast<osg::Geometry*>( i->get() );
            if ( !originalDrawable )
                continue;

            // go through the list of input features...
            for( FeatureList::const_iterator j = _features.begin(); j != _features.end(); j++ )
            {
                Feature* feature = j->get();

                osg::Matrixd scaleMatrix;

                if ( _symbol->scale().isSet() )
                {
                    double scale = feature->eval( _scaleExpr, &_cx );
                    scaleMatrix.makeScale( scale, scale, scale );
                }

                osg::Matrixd rotationMatrix;
                if ( _modelSymbol && _modelSymbol->heading().isSet() )
                {
                    float heading = feature->eval( _headingExpr, &_cx );
                    rotationMatrix.makeRotate( osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0,0,1)) );
                }

                GeometryIterator gi( feature->getGeometry(), false );
                while( gi.hasMore() )
                {
                    Geometry* geom = gi.next();

                    // if necessary, transform the points to the target SRS:
                    if ( !_makeECEF && !_targetSRS->isEquivalentTo(_srs) )
                    {
                        _srs->transform( geom->asVector(), _targetSRS );
                    }

                    for( Geometry::const_iterator k = geom->begin(); k != geom->end(); ++k )
                    {
                        osg::Vec3d   point = *k;
                        osg::Matrixd mat;

                        if ( _makeECEF )
                        {
                            osg::Matrixd rotation;
                            ECEF::transformAndGetRotationMatrix( point, _srs, point, _targetSRS, rotation );
                            mat = rotationMatrix * rotation * scaleMatrix * osg::Matrixd::translate(point) * _f2n->world2local();
                        }
                        else
                        {
                            mat = rotationMatrix * scaleMatrix * osg::Matrixd::translate(point) * _f2n->world2local();
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

                            if ( _cx.featureIndex() )
                                _cx.featureIndex()->tagPrimitiveSets( newDrawable.get(), feature );
                        }
                    }

                }
            }
        }

        geode.dirtyBound();

        MeshConsolidator::run( geode );

        osg::NodeVisitor::apply( geode );
    }

private:
    const FeatureList&      _features;
    FilterContext&          _cx;
    const InstanceSymbol*   _symbol;
    const ModelSymbol*      _modelSymbol;
    FeaturesToNodeFilter*   _f2n;
    NumericExpression       _scaleExpr;
    NumericExpression       _headingExpr;
    bool                    _makeECEF;
    const SpatialReference* _srs;
    const SpatialReference* _targetSRS;
};


//typedef std::map< osg::Node*, FeatureList > MarkerToFeatures;
typedef std::map< osg::ref_ptr<osg::Node>, FeatureList > ModelBins;

//clustering:
//  troll the external model for geodes. for each geode, create a geode in the target
//  model. then, for each geometry in that geode, replicate it once for each instance of
//  the model in the feature batch and transform the actual verts to a local offset
//  relative to the tile centroid. Finally, reassemble all the geodes and optimize. 
//  hopefully stateset sharing etc will work out. we may need to strip out LODs too.
bool
SubstituteModelFilter::cluster(const FeatureList&           features,
                               const InstanceSymbol*        symbol, 
                               Session*                     session,
                               osg::Group*                  attachPoint,
                               FilterContext&               context )
{
    ModelBins modelBins;

    std::set<URI> missing;

    // first, sort the features into buckets, each bucket corresponding to a
    // unique marker.
    for (FeatureList::const_iterator i = features.begin(); i != features.end(); ++i)
    {
        Feature* f = i->get();

        // resolve the URI for the marker:
        StringExpression uriEx( *symbol->url() );
        URI instanceURI( f->eval( uriEx, &context ), uriEx.uriContext() );

        // find and load the corresponding marker model. We're using the session-level
        // object store to cache models. This is thread-safe sine we are always going
        // to CLONE the model before using it.
        osg::ref_ptr<osg::Node> model = context.getSession()->getObject<osg::Node>( instanceURI.full() );
        if ( !model.valid() )
        {
            osg::ref_ptr<InstanceResource> instance;
            if ( !findResource( instanceURI, symbol, context, missing, instance) )
                continue;

            model = instance->createNode( context.getSession()->getDBOptions() );
            if ( model.valid() )
            {
                model = context.getSession()->putObject( instanceURI.full(), model.get(), false );
            }
        }

        if ( model.valid() )
        {
            ModelBins::iterator itr = modelBins.find( model.get() );
            if (itr == modelBins.end())
                modelBins[ model.get() ].push_back( f );
            else
                itr->second.push_back( f );
        }
    }

    // For each model, cluster the features that use that marker
    for (ModelBins::iterator i = modelBins.begin(); i != modelBins.end(); ++i)
    {
        osg::Node* prototype = i->first.get();

        // we're using the Session cache since we know we'll be cloning.
        if ( prototype )
        {
            osg::Node* clone = osg::clone( prototype, osg::CopyOp::DEEP_COPY_ALL );

            // ..and apply the clustering to the copy.
            ClusterVisitor cv( i->second, symbol, this, context );
            clone->accept( cv );

            attachPoint->addChild( clone );
        }
    }

    return true;
}

osg::Node*
SubstituteModelFilter::push(FeatureList& features, FilterContext& context)
{
    if ( !isSupported() )
    {
        OE_WARN << "SubstituteModelFilter support not enabled" << std::endl;
        return 0L;
    }

    if ( _style.empty() )
    {
        OE_WARN << LC << "Empty style; cannot process features" << std::endl;
        return 0L;
    }

    osg::ref_ptr<const InstanceSymbol> symbol = _style.get<InstanceSymbol>();

    // check for deprecated MarkerSymbol type.
    if ( !symbol.valid() )
    {
        if ( _style.has<MarkerSymbol>() )
            symbol = _style.get<MarkerSymbol>()->convertToInstanceSymbol();
    }

    if ( !symbol.valid() )
    {
        OE_WARN << LC << "No appropriate symbol found in stylesheet; aborting." << std::endl;
        return 0L;
    }

    // establish the resource library, if there is one:
    _resourceLib = 0L;

    const StyleSheet* sheet = context.getSession() ? context.getSession()->styles() : 0L;

    if ( symbol->libraryName().isSet() )
    {
        _resourceLib = sheet->getResourceLibrary( symbol->libraryName()->expr() );

        if ( !_resourceLib.valid() )
        {
            OE_WARN << LC << "Unable to load resource library '" << symbol->libraryName()->expr() << "'"
                << "; may not find instance models." << std::endl;
        }
    }

    // reset this marker:
    _normalScalingRequired = false;

    // Compute localization info:
    FilterContext newContext( context );

    computeLocalizers( context );

    osg::Group* group = createDelocalizeGroup();

    // Process the feature set, using clustering if requested
    bool ok = true;
    if ( _cluster )
    {
        ok = cluster( features, symbol, context.getSession(), group, newContext );
    }

    else
    {
        process( features, symbol, context.getSession(), group, newContext );
    }

    // return proper context
    context = newContext;

#if 0
    // TODO: OBE due to shader pipeline
    // see if we need normalized normals
    if ( _normalScalingRequired )
    {
        // TODO: carefully test for this, since GL_NORMALIZE hurts performance in 
        // FFP mode (RESCALE_NORMAL is faster for uniform scaling); and I think auto-normal-scaling
        // is disabled entirely when using shaders. For now I believe we are dropping to FFP
        // when not using instancing...so just check for that
        if ( !_useDrawInstanced )
        {
            group->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
        }
    }
#endif

    return group;
}
