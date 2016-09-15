/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/ShaderUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/DrapeableNode>
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
_clusterCulling    ( true ),
_backfaceCulling   ( true ),
_alphaBlending     ( true ),
_sessionWideResourceCache( true )
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
    conf.getIfSet( "cluster_culling",  _clusterCulling );
    conf.getIfSet( "backface_culling", _backfaceCulling );
    conf.getIfSet( "alpha_blending",   _alphaBlending );
    
    conf.getIfSet( "session_wide_resource_cache", _sessionWideResourceCache );
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
    conf.updateIfSet( "cluster_culling",  _clusterCulling );
    conf.updateIfSet( "backface_culling", _backfaceCulling );
    conf.updateIfSet( "alpha_blending",   _alphaBlending );
    
    conf.updateIfSet( "session_wide_resource_cache", _sessionWideResourceCache );

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

Status
FeatureModelSource::initialize(const osgDB::Options* readOptions)
{
    if (readOptions)
        setReadOptions(readOptions);
    
    // the data source from which to pull features:
    if ( _options.featureSource().valid() )
    {
        _features = _options.featureSource().get();
    }
    else if ( _options.featureOptions().isSet() )
    {
        _features = FeatureSourceFactory::create( _options.featureOptions().value() );
    }

    if (!_features.valid())
        return Status::Error(Status::ServiceUnavailable, "Failed to create a feature driver");

    // open the feature source if it exists:
    const Status& featuresStatus = _features->open(_readOptions.get());
    if (featuresStatus.isError())
        return featuresStatus;

    // Try to fill the DataExtent list using the FeatureProfile
    const FeatureProfile* featureProfile = _features->getFeatureProfile();
    if (featureProfile == NULL)
        return Status::Error("Failed to establish a feature profile");

    if (featureProfile->getProfile() != NULL)
    {
        // Use specified profile's GeoExtent
        getDataExtents().push_back(DataExtent(featureProfile->getProfile()->getExtent()));
    }
    else if (featureProfile->getExtent().isValid() == true)
    {
        // Use FeatureProfile's GeoExtent
        getDataExtents().push_back(DataExtent(featureProfile->getExtent()));
    }

    return Status::OK();
}

void
FeatureModelSource::setReadOptions(const osgDB::Options* readOptions)
{
    _readOptions = Registry::cloneOrCreateOptions(readOptions);
    
    // for texture atlas support
    _readOptions->setObjectCacheHint(osgDB::Options::CACHE_IMAGES);

    if (_features.valid())
    {
        _features->setReadOptions(_readOptions.get());
    }
}

osg::Node*
FeatureModelSource::createNodeImplementation(const Map*        map,
                                             ProgressCallback* progress )
{
    // trivial bailout.
    if (!getStatus().isOK())
        return 0L;

    // user must provide a valid map.
    if ( !map )
    {
        OE_WARN << LC << "NULL Map is illegal when building feature data." << std::endl;
        return 0L;
    }

    // make sure the feature source initialized properly:
    if ( !_features.valid() || !_features->getFeatureProfile() )
    {
        return 0L;
    }

    // create a feature node factory:
    FeatureNodeFactory* factory = createFeatureNodeFactory();
    if ( !factory )
    {
        OE_WARN << LC << "Unable to create a feature node factory!" << std::endl;
        setStatus(Status::Error(Status::ServiceUnavailable, "Failed to create a feature node factory"));
        return 0L;
    }

    // Session holds data that's shared across the life of the FMG
    Session* session = new Session( 
        map, 
        _options.styles().get(), 
        _features.get(), 
        _readOptions.get() );

    // Name the session (for debugging purposes)
    session->setName( this->getName() );

    // Graph that will render feature models. May included paged data.
    FeatureModelGraph* graph = new FeatureModelGraph( 
       session,
       _options,
       factory,
       this,
       _preMergeOps.get(),
       _postMergeOps.get() );

    graph->setName( session->getName() );

    // then run the ops on the staring graph:
    firePostProcessors( graph );

    return graph;
}



//------------------------------------------------------------------------


osg::Group*
FeatureNodeFactory::getOrCreateStyleGroup(const Style& style,
                                          Session*     session)
{
    osg::Group* group = 0L;

    // If we're draping, the style group will be a DrapeableNode.
    const AltitudeSymbol* alt = style.get<AltitudeSymbol>();
    if (alt &&
        alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN &&
        alt->technique() == AltitudeSymbol::TECHNIQUE_DRAPE )
    {
        group = new DrapeableNode();
    }

    // Otherwise, a normal group.
    if ( !group )
    {
        group = new osg::Group();
    }

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

#ifndef OSG_GLES2_AVAILABLE
        if ( render->clipPlane().isSet() )
        {
            GLenum mode = GL_CLIP_PLANE0 + (render->clipPlane().value());
            group->getOrCreateStateSet()->setMode(mode, 1);
        }
#endif

        if ( render->minAlpha().isSet() )
        {
            DiscardAlphaFragments().install( group->getOrCreateStateSet(), render->minAlpha().value() );
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
