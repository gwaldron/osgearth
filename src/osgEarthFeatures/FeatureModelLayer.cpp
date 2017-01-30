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
#include <osgEarthFeatures/FeatureModelLayer>
#include <osgEarthFeatures/FeatureModelGraph>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[FeatureModelLayer] "

REGISTER_OSGEARTH_LAYER(feature_model, FeatureModelLayer);

//...........................................................................

FeatureModelLayerOptions::FeatureModelLayerOptions( const ConfigOptions& options ) :
LayerOptions       ( options ),
_lit               ( true ),
_maxGranularity_deg( 1.0 ),
_clusterCulling    ( true ),
_backfaceCulling   ( true ),
_alphaBlending     ( true ),
_sessionWideResourceCache( true ),
_nodeCaching(false)
{
    fromConfig( _conf );
}

void
FeatureModelLayerOptions::fromConfig(const Config& conf)
{
    conf.getIfSet("feature_source", _featureSourceLayer);

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
    conf.getIfSet( "node_caching",     _nodeCaching );
    
    conf.getIfSet( "session_wide_resource_cache", _sessionWideResourceCache );
}

Config
FeatureModelLayerOptions::getConfig() const
{
    Config conf = LayerOptions::getConfig();
    conf.key() = "feature_model";

    conf.updateIfSet("feature_source", _featureSourceLayer);

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
    conf.updateIfSet( "node_caching",     _nodeCaching );
    
    conf.updateIfSet( "session_wide_resource_cache", _sessionWideResourceCache );

    return conf;
}

//...........................................................................

FeatureModelLayer::FeatureModelLayer(const FeatureModelLayerOptions& options) :
Layer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

FeatureModelLayer::~FeatureModelLayer()
{
    //NOP
}

bool
FeatureModelLayer::setFeatureSourceLayer(FeatureSourceLayer* layer)
{
    setFeatureSource(layer ? layer->getFeatureSource() : 0L);
    return false;
}

void
FeatureModelLayer::setFeatureSource(FeatureSource* source)
{
    _featureSource = source;
    //TODO
}

const Status&
FeatureModelLayer::open()
{
    return Layer::open();
}

void
FeatureModelLayer::addedToMap(const Map* map)
{
    if (options().featureSourceLayer().isSet())
    {
        _featureSourceLayerListener.listen(
            map,
            options().featureSourceLayer().get(),
            this,
            &FeatureModelLayer::setFeatureSourceLayer);
    }

    _fmg = new FeatureModelGraph(
        session,
        options,
        factory);
}

void
FeatureModelLayer::removedFromMap(const Map* map)
{
    _featureSourceLayerListener.clear();
}
