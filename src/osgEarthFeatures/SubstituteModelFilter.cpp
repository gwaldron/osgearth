/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/ECEF>
#include <osgEarth/ShaderComposition>
#include <osgEarth/DrawInstanced>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

#include <osg/AutoTransform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>

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
}

//------------------------------------------------------------------------

SubstituteModelFilter::SubstituteModelFilter( const Style& style ) :
_style                ( style ),
_cluster              ( false ),
_useDrawInstanced     ( false ),
_merge                ( true ),
_normalScalingRequired( false )
{
    //NOP
}

InstanceResource*
SubstituteModelFilter::findResource(const URI&            uri,
                                    const InstanceSymbol* symbol, 
                                    FilterContext&        context, 
                                    std::set<URI>&        missing ) 
{
    // find the corresponding marker in the cache
    InstanceResource* instance = 0L;
    InstanceCache::Record rec = _instanceCache.get( uri );

    if ( rec.valid() )
    {
        // found it in the cache:
        instance = rec.value();
    }
    else if ( _resourceLib.valid() )
    {
        // look it up in the resource library:
        instance = _resourceLib->getInstance( uri.base(), context.getDBOptions() );
    }
    else
    {
        // create it on the fly:
        OE_DEBUG << "New resource (not in the cache!)" << std::endl;
        instance = symbol->createResource();
        instance->uri() = uri;
        _instanceCache.insert( uri, instance );
    }

    // failed to find the instance.
    if ( instance == 0L )
    {
        if ( missing.find(uri) == missing.end() )
        {
            missing.insert(uri);
            OE_WARN << LC << "Failed to locate resource: " << uri.full() << std::endl;
        }
    }

    return instance;
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

        // evaluate the instance URI expression:
        StringExpression uriEx = *symbol->url();
        URI instanceURI( input->eval(uriEx, &context), uriEx.uriContext() );

        // find the corresponding marker in the cache
        InstanceResource* instance = findResource( instanceURI, symbol, context, missing );
        if ( !instance )
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
#if 0
        if ( symbol->orientation().isSet() )
        {
            osg::Vec3d hpr = *symbol->orientation();
            //Rotation in HPR
            //Apply the rotation            
            rotationMatrix.makeRotate( 
                osg::DegreesToRadians(hpr.y()), osg::Vec3(1,0,0),
                osg::DegreesToRadians(hpr.x()), osg::Vec3(0,0,1),
                osg::DegreesToRadians(hpr.z()), osg::Vec3(0,1,0) );
        }
#endif

        if ( modelSymbol && modelSymbol->heading().isSet() )
        {
            float heading = input->eval(headingEx, &context);
            rotationMatrix.makeRotate( osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0,0,1)) );
        }

        // how that we have a marker source, create a node for it
        std::pair<URI,float> key( instanceURI, scale );

        osg::ref_ptr<osg::Node>& model = uniqueModels[key];
        if ( !model.valid() )
        {
            model = context.resourceCache()->getInstanceNode( instance );

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

                // if necessary, transform the points to the target SRS:
                if ( !makeECEF && !targetSRS->isEquivalentTo(context.profile()->getSRS()) )
                {
                    context.profile()->getSRS()->transform( geom->asVector(), targetSRS );
                }

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
                        context.featureIndex()->tagNode( xform, input->getFID() );
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

    if ( _useDrawInstanced && Registry::instance()->getCapabilities().supportsDrawInstanced() )
    {
        //OE_INFO << LC << "Converting to draw-instanced..." << std::endl;

        DrawInstanced::convertGraphToUseDrawInstanced( attachPoint );

        // install a shader program
        VirtualProgram* p = DrawInstanced::createDrawInstancedProgram();
        p->installDefaultColoringShaders( 1 );

        attachPoint->getOrCreateStateSet()->setAttributeAndModes( p, osg::StateAttribute::ON );
    }
    else
    {
        attachPoint->getOrCreateStateSet()->setAttribute( new osg::Program, 0 );
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
        osg::Geode::DrawableList old_drawables = geode.getDrawableList();

        //OE_DEBUG << "ClusterVisitor geode " << &geode << " featureNode=" << _featureNode << " drawables=" << old_drawables.size() << std::endl;

        // ..and clear out the drawables list.
        geode.removeDrawables( 0, geode.getNumDrawables() );

#if 0
        // ... and remove all drawables from the feature node
        for( osg::Geode::DrawableList::iterator i = old_drawables.begin(); i != old_drawables.end(); i++ )
            _featureNode->removeDrawable(i->get());
#endif

        // foreach each drawable that was originally in the geode...
        for( osg::Geode::DrawableList::iterator i = old_drawables.begin(); i != old_drawables.end(); i++ )
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
#if 0
                if ( _symbol->orientation().isSet() )
                {
                    osg::Vec3d hpr = *_symbol->orientation();
                    //Rotation in HPR
                    //Apply the rotation            
                    rotationMatrix.makeRotate( 
                        osg::DegreesToRadians(hpr.y()), osg::Vec3(1,0,0),
                        osg::DegreesToRadians(hpr.x()), osg::Vec3(0,0,1),
                        osg::DegreesToRadians(hpr.z()), osg::Vec3(0,1,0) );            
                }
#endif
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
                            ECEF::transformAndGetRotationMatrix( point, _srs, point, rotation );
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
                                _cx.featureIndex()->tagPrimitiveSets( newDrawable.get(), feature->getFID() );
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
    //ModelToFeatures modelToFeatures;

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
            InstanceResource* instance = findResource( instanceURI, symbol, context, missing );
            if ( !instance )
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

    return group;
}
