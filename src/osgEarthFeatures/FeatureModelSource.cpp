/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureModelGraph>
#include <osgEarth/SpatialReference>
#include <osgEarth/ShaderFactory>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osg/Notify>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[FeatureModelSource] "

//------------------------------------------------------------------------

FeatureModelSourceOptions::FeatureModelSourceOptions( const ConfigOptions& options ) :
ModelSourceOptions ( options ),
_lit               ( true ),
_maxGranularity_deg( 1.0 ),
_mergeGeometry     ( false ),
_clusterCulling    ( true ),
_backfaceCulling   ( true ),
_alphaBlending     ( true )
{
    fromConfig( _conf );
}

void
FeatureModelSourceOptions::fromConfig( const Config& conf )
{
    conf.getObjIfSet( "features", _featureOptions );
    _featureSource = conf.getNonSerializable<FeatureSource>("feature_source");

    conf.getObjIfSet( "styles",           _styles );
    conf.getObjIfSet( "layout",           _layout );
    conf.getObjIfSet( "paging",           _layout ); // backwards compat.. to be deprecated
    conf.getObjIfSet( "cache_policy",     _cachePolicy );
    conf.getObjIfSet( "fading",           _fading );
    conf.getObjIfSet( "feature_name",     _featureNameExpr );
    conf.getObjIfSet( "feature_indexing", _featureIndexing );

    conf.getIfSet( "lighting",         _lit );
    conf.getIfSet( "max_granularity",  _maxGranularity_deg );
    conf.getIfSet( "merge_geometry",   _mergeGeometry );
    conf.getIfSet( "cluster_culling",  _clusterCulling );
    conf.getIfSet( "backface_culling", _backfaceCulling );
    conf.getIfSet( "alpha_blending",   _alphaBlending );

}

Config
FeatureModelSourceOptions::getConfig() const
{
    Config conf = ModelSourceOptions::getConfig();

    conf.updateObjIfSet( "features", _featureOptions );    
    if (_featureSource.valid())
    {
        conf.addNonSerializable("feature_source", _featureSource.get());
    }
    conf.updateObjIfSet( "styles",           _styles );
    conf.updateObjIfSet( "layout",           _layout );
    conf.updateObjIfSet( "cache_policy",     _cachePolicy );
    conf.updateObjIfSet( "fading",           _fading );
    conf.updateObjIfSet( "feature_name",     _featureNameExpr );
    conf.updateObjIfSet( "feature_indexing", _featureIndexing );

    conf.updateIfSet( "lighting",         _lit );
    conf.updateIfSet( "max_granularity",  _maxGranularity_deg );
    conf.updateIfSet( "merge_geometry",   _mergeGeometry );
    conf.updateIfSet( "cluster_culling",  _clusterCulling );
    conf.updateIfSet( "backface_culling", _backfaceCulling );
    conf.updateIfSet( "alpha_blending",   _alphaBlending );

    return conf;
}

//------------------------------------------------------------------------

FeatureModelSource::FeatureModelSource( const FeatureModelSourceOptions& options ) :
ModelSource( options ),
_options   ( options )
{
    //nop
}

void
FeatureModelSource::setFeatureSource( FeatureSource* source )
{
    if ( !_features.valid() )
    {
        _features = source;
    }
    else
    {
        OE_WARN << LC << "Illegal: cannot set a feature source after one is already set" << std::endl;
    }
}

void 
FeatureModelSource::initialize(const osgDB::Options* dbOptions)
{
    ModelSource::initialize( dbOptions );
    
    // the data source from which to pull features:
    if ( _options.featureSource().valid() )
    {
        _features = _options.featureSource().get();
    }
    else if ( _options.featureOptions().isSet() )
    {
        _features = FeatureSourceFactory::create( _options.featureOptions().value() );
        if ( !_features.valid() )
        {
            OE_WARN << LC << "No valid feature source provided!" << std::endl;
        }
    }

    // initialize the feature source if it exists:
    if ( _features.valid() )
    {
        _features->initialize( dbOptions );
    }
    else
    {
        OE_WARN << LC << "No FeatureSource; nothing will be rendered (" << getName() << ")" << std::endl;
    }
}

osg::Node*
FeatureModelSource::createNodeImplementation(const Map*            map,
                                             const osgDB::Options* dbOptions,
                                             ProgressCallback*     progress )
{
    // user must provide a valid map.
    if ( !map )
    {
        OE_WARN << LC << "NULL Map is illegal when building feature data." << std::endl;
        return 0L;
    }

    // make sure the feature source initialized properly:
    if ( !_features.valid() || !_features->getFeatureProfile() )
    {
        OE_WARN << LC << "Invalid feature source" << std::endl;
        return 0L;
    }

    // create a feature node factory:
    FeatureNodeFactory* factory = createFeatureNodeFactory();
    if ( !factory )
    {
        OE_WARN << LC << "Unable to create a feature node factory!" << std::endl;
        return 0L;
    }

    // Session holds data that's shared across the life of the FMG
    Session* session = new Session( map, _options.styles().get(), _features.get(), dbOptions );

    // Graph that will render feature models. May included paged data.
    FeatureModelGraph* graph = new FeatureModelGraph( session, _options, factory );

    // install any post-merge operations on the FMG so it can call them during paging:
    const NodeOperationVector& ops = postProcessors();
    for( NodeOperationVector::const_iterator i = ops.begin(); i != ops.end(); ++i )
    {
        graph->addPostMergeOperation( i->get() );
    }

    // then run the ops on the staring graph:
    firePostProcessors( graph );

    return graph;
}



//------------------------------------------------------------------------


osg::Group*
FeatureNodeFactory::getOrCreateStyleGroup(const Style& style,
                                          Session*     session)
{
    osg::Group* group = new osg::Group();

    // apply necessary render styles.
    const RenderSymbol* render = style.get<RenderSymbol>();
    if ( render )
    {
        if ( render->depthTest().isSet() )
        {
            group->getOrCreateStateSet()->setMode(
                GL_DEPTH_TEST, 
                (render->depthTest() == true ? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

        if ( render->lighting().isSet() )
        {
            osg::StateSet* stateset = group->getOrCreateStateSet();

            stateset->setMode(
                GL_LIGHTING,
                (render->lighting() == true ? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );

            if ( Registry::capabilities().supportsGLSL() )
            {
                stateset->addUniform( Registry::shaderFactory()->createUniformForGLMode(
                    GL_LIGHTING, render->lighting().value()));
            }
        }

        if ( render->backfaceCulling().isSet() )
        {
            group->getOrCreateStateSet()->setMode(
                GL_CULL_FACE,
                (render->backfaceCulling() == true ? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }
    }

    return group;
}


//------------------------------------------------------------------------


GeomFeatureNodeFactory::GeomFeatureNodeFactory( const GeometryCompilerOptions& options ) : 
_options( options ) 
{ 
    //nop
}

bool GeomFeatureNodeFactory::createOrUpdateNode(
    FeatureCursor*            features,
    const Style&              style,
    const FilterContext&      context,
    osg::ref_ptr<osg::Node>&  node )
{
    GeometryCompiler compiler( _options );
    node = compiler.compile( features, style, context );
    return node.valid();
}
