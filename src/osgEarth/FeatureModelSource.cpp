/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/FeatureModelSource>
#include <osgEarth/FeatureModelGraph>
#include <osgEarth/SpatialReference>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ShaderUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/DrapeableNode>
#include <osgEarth/ClampableNode>
#include <osgEarth/GLUtils>
#include <osgEarth/Progress>
#include <osg/Notify>

using namespace osgEarth;

#ifndef GL_CLIP_DISTANCE0
#define GL_CLIP_DISTANCE0 0x3000
#endif

#define LC "[FeatureModelSource] "

//........................................................................

FeatureModelOptions::FeatureModelOptions(const ConfigOptions& co) :
_lit               ( true ),
_maxGranularity_deg( 1.0 ),
_clusterCulling    ( false ),
_backfaceCulling   ( true ),
_alphaBlending     ( true ),
_sessionWideResourceCache( true ),
_nodeCaching(false)
{
    fromConfig(co.getConfig());
}

void
FeatureModelOptions::fromConfig(const Config& conf)
{
    LayerReference<StyleSheet>::get(conf, "styles", _styleSheetLayer, _styleSheet);

    conf.get( "layout",           _layout );
    conf.get( "fading",           _fading );
    conf.get( "feature_name",     _featureNameExpr );
    conf.get( "feature_indexing", _featureIndexing );

    conf.get( "lighting",         _lit );
    conf.get( "max_granularity",  _maxGranularity_deg );
    conf.get( "cluster_culling",  _clusterCulling );
    conf.get( "backface_culling", _backfaceCulling );
    conf.get( "alpha_blending",   _alphaBlending );
    conf.get( "node_caching",     _nodeCaching );
    
    conf.get( "session_wide_resource_cache", _sessionWideResourceCache );
}

Config
FeatureModelOptions::getConfig() const
{
    Config conf;

    LayerReference<StyleSheet>::set(conf, "styles", styleSheetLayer(), styleSheet());

    conf.set( "layout",           _layout );
    conf.set( "fading",           _fading );
    conf.set( "feature_name",     _featureNameExpr );
    conf.set( "feature_indexing", _featureIndexing );

    conf.set( "lighting",         _lit );
    conf.set( "max_granularity",  _maxGranularity_deg );
    conf.set( "cluster_culling",  _clusterCulling );
    conf.set( "backface_culling", _backfaceCulling );
    conf.set( "alpha_blending",   _alphaBlending );
    conf.set( "node_caching",     _nodeCaching );
    
    conf.set( "session_wide_resource_cache", _sessionWideResourceCache );

    return conf;
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

    else if (alt &&
        alt->clamping() == alt->CLAMP_TO_TERRAIN &&
        alt->technique() == alt->TECHNIQUE_GPU)
    {
        group = new ClampableNode();
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

            GLUtils::setLighting(
                stateset,
                (render->lighting() == true ? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

        if ( render->backfaceCulling().isSet() )
        {
            group->getOrCreateStateSet()->setMode(
                GL_CULL_FACE,
                (render->backfaceCulling() == true ? osg::StateAttribute::ON : osg::StateAttribute::OFF) | osg::StateAttribute::OVERRIDE );
        }

#if !(defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE))
        if ( render->clipPlane().isSet() )
        {
            GLenum mode = GL_CLIP_DISTANCE0 + (render->clipPlane().value());
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
    osg::ref_ptr<osg::Node>&  node,
    const Query&              query
)
{
    GeometryCompiler compiler( _options );
    node = compiler.compile( features, style, context );
    return node.valid();
}
