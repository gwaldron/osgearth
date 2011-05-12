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
#include "GeomCompiler"
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/ClampFilter>
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/ScatterFilter>
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osg/MatrixTransform>

#define LC "[GeomCompiler] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

GeomCompiler::GeomCompiler( const FeatureGeomModelOptions& options ) :
_options( options )
{
    //nop
}

osg::Node*
GeomCompiler::compile(FeatureCursor*        cursor,
                      const Style&          style,
                      const FilterContext&  context)

{
    if ( !context.profile() ) {
        OE_WARN << LC << "Valid feature profile required" << std::endl;
        return 0L;
    }

    if ( style.empty() ) {
        OE_WARN << LC << "Non-empty style required" << std::endl;
        return 0L;
    }

    osg::ref_ptr<osg::Group> resultGroup = new osg::Group();

    // start by making a working copy of the feature set
    FeatureList workingSet;
    cursor->fill( workingSet );

    // create a filter context that will track feature data through the process
    FilterContext cx = context;
    if ( !cx.extent().isSet() )
        cx.extent() = cx.profile()->getExtent();

    // only localize coordinates if the map if geocentric AND the extent is
    // less than 180 degrees.
    const MapInfo& mi = cx.getSession()->getMapInfo();
    GeoExtent workingExtent = cx.extent()->transform( cx.profile()->getSRS()->getGeographicSRS() );
    bool localize = mi.isGeocentric() && workingExtent.width() < 180.0;

    // go through the Style and figure out which filters to use.
    const MarkerSymbol*    marker    = style.get<MarkerSymbol>();
    const PointSymbol*     point     = style.get<PointSymbol>();
    const LineSymbol*      line      = style.get<LineSymbol>();
    const PolygonSymbol*   polygon   = style.get<PolygonSymbol>();
    const ExtrusionSymbol* extrusion = style.get<ExtrusionSymbol>();
    const AltitudeSymbol*  altitude  = style.get<AltitudeSymbol>();
    const TextSymbol*      text      = style.get<TextSymbol>();
    
    // transform the features into the map profile
    TransformFilter xform( mi.getProfile()->getSRS(), mi.isGeocentric() );   
    xform.setLocalizeCoordinates( localize );
    if ( altitude && altitude->verticalOffset().isSet() )
        xform.setMatrix( osg::Matrixd::translate(0, 0, *altitude->verticalOffset()) );
    cx = xform.push( workingSet, cx );

    bool clampRequired =
        altitude && altitude->clamping() != AltitudeSymbol::CLAMP_NONE;

    // model substitution
    if ( marker )
    {
        if ( marker->placement() == MarkerSymbol::PLACEMENT_RANDOM   ||
             marker->placement() == MarkerSymbol::PLACEMENT_INTERVAL )
        {
            ScatterFilter scatter;
            scatter.setDensity( *marker->density() );
            scatter.setRandom( marker->placement() == MarkerSymbol::PLACEMENT_RANDOM );
            scatter.setRandomSeed( *marker->randomSeed() );
            cx = scatter.push( workingSet, cx );
        }

        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setIgnoreZ( altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
            cx = clamp.push( workingSet, cx );
            clampRequired = false;
        }

        SubstituteModelFilter sub( style );
        sub.setClustering( *_options.clustering() );
        if ( marker->scale().isSet() )
            sub.setModelMatrix( osg::Matrixd::scale( *marker->scale() ) );

        cx = sub.push( workingSet, cx );

        osg::Node* node = sub.getNode();
        if ( node )
            resultGroup->addChild( node );
    }

    // extruded geometry
    if ( extrusion && ( line || polygon ) )
    {
        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setIgnoreZ( altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
            cx = clamp.push( workingSet, cx );
            clampRequired = false;
        }

        ExtrudeGeometryFilter extrude;
        if ( extrusion )
        {
            if ( extrusion->height().isSet() )
                extrude.setExtrusionHeight( *extrusion->height() );
            if ( extrusion->heightExpression().isSet() )
                extrude.setExtrusionExpr( *extrusion->heightExpression() );

            extrude.setFlatten( *extrusion->flatten() );
        }
        if ( polygon )
        {
            extrude.setColor( polygon->fill()->color() );
        }

        osg::Node* node = extrude.push( workingSet, cx );
        if ( node )
            resultGroup->addChild( node );
    }

    // simple geometry
    else if ( point || line || polygon )
    {
        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setIgnoreZ( altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
            cx = clamp.push( workingSet, cx );
            clampRequired = false;
        }

        BuildGeometryFilter filter( style );
        if ( _options.maxGranularity().isSet() )
            filter.maxGranularity() = *_options.maxGranularity();
        if ( _options.mergeGeometry().isSet() )
            filter.mergeGeometry() = *_options.mergeGeometry();
        cx = filter.push( workingSet, cx );

        osg::Node* node = filter.getNode();
        if ( node )
            resultGroup->addChild( node );
    }

    if ( text )
    {
        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setIgnoreZ( altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
            cx = clamp.push( workingSet, cx );
            clampRequired = false;
        }

        BuildTextFilter filter( style );
        cx = filter.push( workingSet, cx );

        osg::Node* node = filter.takeNode();
        if ( node )
            resultGroup->addChild( node );
    }

    //else // insufficient symbology
    //{
    //    OE_WARN << LC << "Insufficient symbology; no geometry created" << std::endl;
    //}

    // install the localization transform if necessary.
    if ( cx.hasReferenceFrame() )
    {
        osg::MatrixTransform* delocalizer = new osg::MatrixTransform( cx.inverseReferenceFrame() );
        delocalizer->addChild( resultGroup.get() );
        resultGroup = delocalizer;
    }

    resultGroup->getOrCreateStateSet()->setMode( GL_BLEND, 1 );

    return resultGroup.release();
}
