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
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureModelGraph>
#include <osgEarth/SpatialReference>
#include <osg/Notify>
#include <osg/Timer>
#include <osg/LOD>
#include <osg/ClusterCullingCallback>
#include <osgUtil/Optimizer>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[FeatureModelSource] "

//------------------------------------------------------------------------

FeatureModelSourceOptions::FeatureModelSourceOptions( const ConfigOptions& options ) :
ModelSourceOptions( options ),
_geomTypeOverride( Geometry::TYPE_UNKNOWN ),
_lit( true ),
_maxGranularity_deg( 5.0 ),
_mergeGeometry( false ),
_clusterCulling( true )
{
    fromConfig( _conf );
}

void
FeatureModelSourceOptions::fromConfig( const Config& conf )
{
    conf.getObjIfSet( "features", _featureOptions );
    //if ( conf.hasChild("features") )
    //    _featureOptions->merge( conf.child("features") );
    _featureSource = conf.getNonSerializable<FeatureSource>("feature_source");

    conf.getObjIfSet( "styles", _styles );
    conf.getObjIfSet( "layout", _levels );
    conf.getObjIfSet( "paging", _levels ); // backwards compat.. to be deprecated
    conf.getObjIfSet( "gridding", _gridding ); // to be deprecated
    conf.getObjIfSet( "feature_name", _featureNameExpr );
    conf.getIfSet( "lighting", _lit );
    conf.getIfSet( "max_granularity", _maxGranularity_deg );
    conf.getIfSet( "merge_geometry", _mergeGeometry );
    conf.getIfSet( "cluster_culling", _clusterCulling );

    std::string gt = conf.value( "geometry_type" );
    if ( gt == "line" || gt == "lines" || gt == "linestring" )
        _geomTypeOverride = Geometry::TYPE_LINESTRING;
    else if ( gt == "point" || gt == "pointset" || gt == "points" )
        _geomTypeOverride = Geometry::TYPE_POINTSET;
    else if ( gt == "polygon" || gt == "polygons" )
        _geomTypeOverride = Geometry::TYPE_POLYGON;
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
    //conf.updateObjIfSet( "feature_source", _featureSource);
    conf.updateObjIfSet( "gridding", _gridding ); // to be deprecated
    conf.updateObjIfSet( "styles", _styles );
    conf.updateObjIfSet( "layout", _levels );

    conf.updateIfSet( "lighting", _lit );
    conf.updateIfSet( "max_granularity", _maxGranularity_deg );
    conf.updateIfSet( "merge_geometry", _mergeGeometry );
    conf.updateIfSet( "cluster_culling", _clusterCulling );


    if ( _geomTypeOverride.isSet() ) {
        if ( _geomTypeOverride == Geometry::TYPE_LINESTRING )
            conf.update( "geometry_type", "line" );
        else if ( _geomTypeOverride == Geometry::TYPE_POINTSET )
            conf.update( "geometry_type", "point" );
        else if ( _geomTypeOverride == Geometry::TYPE_POLYGON )
            conf.update( "geometry_type", "polygon" );
    }

    return conf;
}

//------------------------------------------------------------------------

FeatureModelSource::FeatureModelSource( const FeatureModelSourceOptions& options ) :
ModelSource( options ),
_options( options )
{
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
            OE_WARN << "FeatureModelSource - no valid feature source provided" << std::endl;
        }
    }
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
FeatureModelSource::initialize( const std::string& referenceURI, const osgEarth::Map* map )
{
    ModelSource::initialize( referenceURI, map );

    if ( _features.valid() )
    {
        _features->initialize( referenceURI );
    }
    else
    {
        OE_WARN << LC << "No FeatureSource; nothing will be rendered (" << getName() << ")" << std::endl;
    }

    _map = map;
}

osg::Node*
FeatureModelSource::createNode( ProgressCallback* progress )
{
    if ( !_factory.valid() )
        _factory = createFeatureNodeFactory();

    if ( !_factory.valid() )
        return 0L;

    if ( !_features.valid() || !_features->getFeatureProfile() )
    {
        OE_WARN << LC << "Invalid feature source" << std::endl;
        return 0L;
    }

    FeatureModelGraph* graph = new FeatureModelGraph( 
        _features.get(), 
        _options, 
        _factory.get(),
        *_options.styles(),
        new Session( _map.get() ) );

    return graph;
}

//------------------------------------------------------------------------
GeomFeatureNodeFactory::GeomFeatureNodeFactory( const GeometryCompilerOptions& options )
            : _options( options ) 
{ 
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