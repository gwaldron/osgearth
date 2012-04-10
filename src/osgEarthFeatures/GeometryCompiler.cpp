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
#include "GeometryCompiler"
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/AltitudeFilter>
#include <osgEarthFeatures/CentroidFilter>
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/ScatterFilter>
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarthFeatures/TessellateOperator>
#include <osg/MatrixTransform>
#include <osg/Timer>
#include <osgDB/WriteFile>

#define LC "[GeometryCompiler] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//-----------------------------------------------------------------------

namespace
{
    osg::ref_ptr<PointSymbol>   s_defaultPointSymbol   = new PointSymbol();
    osg::ref_ptr<LineSymbol>    s_defaultLineSymbol    = new LineSymbol();
    osg::ref_ptr<PolygonSymbol> s_defaultPolygonSymbol = new PolygonSymbol();
}

//-----------------------------------------------------------------------

GeometryCompilerOptions::GeometryCompilerOptions( const ConfigOptions& conf ) :
ConfigOptions      ( conf ),
_maxGranularity_deg( 1.0 ),
_mergeGeometry     ( false ),
_clustering        ( true ),
_ignoreAlt         ( false )
{
    fromConfig(_conf);
}

void
GeometryCompilerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet   ( "max_granularity",  _maxGranularity_deg );
    conf.getIfSet   ( "merge_geometry",   _mergeGeometry );
    conf.getIfSet   ( "clustering",       _clustering );
    conf.getObjIfSet( "feature_name",     _featureNameExpr );
    conf.getIfSet   ( "ignore_altitude",  _ignoreAlt );
    conf.getIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.getIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
}

Config
GeometryCompilerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.addIfSet   ( "max_granularity",  _maxGranularity_deg );
    conf.addIfSet   ( "merge_geometry",   _mergeGeometry );
    conf.addIfSet   ( "clustering",       _clustering );
    conf.addObjIfSet( "feature_name",     _featureNameExpr );
    conf.addIfSet   ( "ignore_altitude",  _ignoreAlt );
    conf.addIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.addIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    return conf;
}

void
GeometryCompilerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//-----------------------------------------------------------------------

GeometryCompiler::GeometryCompiler()
{
    //nop
}

GeometryCompiler::GeometryCompiler( const GeometryCompilerOptions& options ) :
_options( options )
{
    //nop
}

osg::Node*
GeometryCompiler::compile(Geometry*             geometry,
                          const Style&          style,
                          const FilterContext&  context)
{
    osg::ref_ptr<Feature> f = new Feature(geometry, 0L); // no SRS!
    return compile(f.get(), style, context);
}

osg::Node*
GeometryCompiler::compile(Geometry*             geometry,
                          const FilterContext&  context)
{
    return compile( geometry, Style(), context );
}

osg::Node*
GeometryCompiler::compile(Feature*              feature,
                          const Style&          style,
                          const FilterContext&  context)
{
    FeatureList workingSet;
    workingSet.push_back(feature);
    return compile(workingSet, style, context);
}

osg::Node*
GeometryCompiler::compile(Feature*              feature,
                          const FilterContext&  context)
{
    return compile(feature, *feature->style(), context);
}

osg::Node*
GeometryCompiler::compile(FeatureCursor*        cursor,
                          const Style&          style,
                          const FilterContext&  context)

{
    // start by making a working copy of the feature set
    FeatureList workingSet;
    cursor->fill( workingSet );

    return compile(workingSet, style, context);
}

osg::Node*
GeometryCompiler::compile(FeatureList&          workingSet,
                          const Style&          style,
                          const FilterContext&  context)
{
#ifdef PROFILING
    osg::Timer_t p_start = osg::Timer::instance()->tick();
    unsigned p_features = workingSet.size();
#endif

    osg::ref_ptr<osg::Group> resultGroup = new osg::Group();

    // create a filter context that will track feature data through the process
    FilterContext sharedCX = context;
    if ( !sharedCX.extent().isSet() && sharedCX.profile() )
    {
        sharedCX.extent() = sharedCX.profile()->getExtent();
    }

    // only localize coordinates if the map is geocentric AND the extent is
    // less than 180 degrees.
    bool localize = false;
    if ( sharedCX.isGeoreferenced() )
    {
        const MapInfo& mi = sharedCX.getSession()->getMapInfo();
        GeoExtent workingExtent = sharedCX.extent()->transform( sharedCX.profile()->getSRS()->getGeographicSRS() );
        localize = mi.isGeocentric() && workingExtent.width() < 180.0;
    }

    // go through the Style and figure out which filters to use.
    const MarkerSymbol*    marker    = style.get<MarkerSymbol>();
    const PointSymbol*     point     = style.get<PointSymbol>();
    const LineSymbol*      line      = style.get<LineSymbol>();
    const PolygonSymbol*   polygon   = style.get<PolygonSymbol>();
    const ExtrusionSymbol* extrusion = style.get<ExtrusionSymbol>();
    const AltitudeSymbol*  altitude  = style.get<AltitudeSymbol>();
    const TextSymbol*      text      = style.get<TextSymbol>();

    // check whether we need tessellation:
    if ( line && line->tessellation().isSet() )
    {
        TemplateFeatureFilter<TessellateOperator> filter;
        filter.setNumPartitions( *line->tessellation() );
        sharedCX = filter.push( workingSet, sharedCX );
    }

    // if the style was empty, use some defaults based on the geometry type of the
    // first feature.
    if ( !point && !line && !polygon && !marker && !extrusion && !text && workingSet.size() > 0 )
    {
        Feature* first = workingSet.begin()->get();
        Geometry* geom = first->getGeometry();
        if ( geom )
        {
            switch( geom->getComponentType() )
            {
            case Geometry::TYPE_LINESTRING:
            case Geometry::TYPE_RING:
                line = s_defaultLineSymbol.get(); break;
            case Geometry::TYPE_POINTSET:
                point = s_defaultPointSymbol.get(); break;
            case Geometry::TYPE_POLYGON:
                polygon = s_defaultPolygonSymbol.get(); break;
            case Geometry::TYPE_UNKNOWN: break;
            case Geometry::TYPE_MULTI: break;
            }
        }
    }

    if (_options.resampleMode().isSet())
    {
        ResampleFilter resample;
        resample.resampleMode() = *_options.resampleMode();        
        if (_options.resampleMaxLength().isSet())
        {
            resample.maxLength() = *_options.resampleMaxLength();
        }                   
        sharedCX = resample.push( workingSet, sharedCX );        
    }    
    
    bool altRequired =
        _options.ignoreAltitudeSymbol() != true &&
        altitude && (
            altitude->clamping() != AltitudeSymbol::CLAMP_NONE ||
            altitude->verticalOffset().isSet() ||
            altitude->verticalScale().isSet() );

    // model substitution
    if ( marker )
    {
        // use a separate filter context since we'll be munging the data
        FilterContext markerCX = sharedCX;

        if ( marker->placement() == MarkerSymbol::PLACEMENT_RANDOM   ||
             marker->placement() == MarkerSymbol::PLACEMENT_INTERVAL )
        {
            ScatterFilter scatter;
            scatter.setDensity( *marker->density() );
            scatter.setRandom( marker->placement() == MarkerSymbol::PLACEMENT_RANDOM );
            scatter.setRandomSeed( *marker->randomSeed() );
            markerCX = scatter.push( workingSet, markerCX );
        }
        else if ( marker->placement() == MarkerSymbol::PLACEMENT_CENTROID )
        {
            CentroidFilter centroid;
            centroid.push( workingSet, markerCX );
        }

        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            markerCX = clamp.push( workingSet, markerCX );

            // don't set this; we changed the input data.
            //altRequired = false;
        }

        SubstituteModelFilter sub( style );
        if ( marker->scale().isSet() )
        {
            //Turn on GL_NORMALIZE so lighting works properly
            resultGroup->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON );
            //sub.setModelMatrix( osg::Matrixd::scale( *marker->scale() ) );
        }

        sub.setClustering( *_options.clustering() );
        if ( _options.featureName().isSet() )
            sub.setFeatureNameExpr( *_options.featureName() );

        osg::Node* node = sub.push( workingSet, markerCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    // extruded geometry
    if ( extrusion )
    {
        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            altRequired = false;
        }

        ExtrudeGeometryFilter extrude;
        extrude.setStyle( style );

        // apply per-feature naming if requested.
        if ( _options.featureName().isSet() )
            extrude.setFeatureNameExpr( *_options.featureName() );

        osg::Node* node = extrude.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    // simple geometry
    else if ( point || line || polygon )
    {
        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            altRequired = false;
        }

        BuildGeometryFilter filter( style );
        if ( _options.maxGranularity().isSet() )
            filter.maxGranularity() = *_options.maxGranularity();
        if ( _options.geoInterp().isSet() )
            filter.geoInterp() = *_options.geoInterp();
        if ( _options.mergeGeometry().isSet() )
            filter.mergeGeometry() = *_options.mergeGeometry();
        if ( _options.featureName().isSet() )
            filter.featureName() = *_options.featureName();

        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    if ( text )
    {
        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            altRequired = false;
        }

        BuildTextFilter filter( style );
        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    resultGroup->getOrCreateStateSet()->setMode( GL_BLEND, 1 );

    //osgDB::writeNodeFile( *(resultGroup.get()), "out.osg" );

#ifdef PROFILING
    osg::Timer_t p_end = osg::Timer::instance()->tick();
    OE_INFO << LC
        << "features = " << p_features <<
        << " ,time = " << osg::Timer::instance()->delta_s(p_start, p_end) << " s." << std::endl;
#endif

    return resultGroup.release();
}
