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

GeomCompiler::GeomCompiler( Session* session, const FeatureGeomModelOptions& options ) :
_session( session ),
_options( options )
{
    //nop
}

osg::Node*
GeomCompiler::compile(FeatureCursor*        cursor,
                      const FeatureProfile* featureProfile,
                      const Style*          style)
{
    if ( !featureProfile ) {
        OE_WARN << LC << "Valid feature profile required" << std::endl;
        return 0L;
    }

    if ( !style ) {
        OE_WARN << LC << "Valid style required" << std::endl;
        return 0L;
    }

    osg::ref_ptr<osg::Node> result;

    // start by making a working copy of the feature set
    FeatureList workingSet;
    cursor->fill( workingSet );

    // create a filter context that will track feature data through the process
    FilterContext cx( _session.get() );
    cx.profile() = featureProfile;

    // only localize coordinates if the map if geocentric AND the extent is
    // less than 180 degrees.
    const MapInfo& mi = _session->getMapInfo();
    GeoExtent geoExtent = featureProfile->getExtent().transform( featureProfile->getSRS()->getGeographicSRS() );
    bool localize = mi.isGeocentric() && geoExtent.width() < 180.0;

    // go through the Style and figure out which filters to use.
    const ModelSymbol*     model     = style->getSymbol<ModelSymbol>();
    const PointSymbol*     point     = style->getSymbol<PointSymbol>();
    const LineSymbol*      line      = style->getSymbol<LineSymbol>();
    const PolygonSymbol*   polygon   = style->getSymbol<PolygonSymbol>();
    const ExtrusionSymbol* extrusion = style->getSymbol<ExtrusionSymbol>();
    const AltitudeSymbol*  altitude  = style->getSymbol<AltitudeSymbol>();
    
    // transform the features into the map profile
    TransformFilter xform( mi.getProfile()->getSRS(), mi.isGeocentric() );   
    xform.setLocalizeCoordinates( localize );
    if ( altitude && altitude->verticalOffset().isSet() )
        xform.setMatrix( osg::Matrixd::translate(0, 0, *altitude->verticalOffset()) );
    cx = xform.push( workingSet, cx );

    // model substitution
    if ( model )
    {
        if ( model->placement() == ModelSymbol::PLACEMENT_RANDOM   ||
             model->placement() == ModelSymbol::PLACEMENT_INTERVAL )
        {
            ScatterFilter scatter;
            scatter.setDensity( *model->density() );
            scatter.setRandom( model->placement() == ModelSymbol::PLACEMENT_RANDOM );
            scatter.setRandomSeed( *model->randomSeed() );
            cx = scatter.push( workingSet, cx );
        }

        if ( altitude && altitude->clamping() != AltitudeSymbol::CLAMP_NONE )
        {
            ClampFilter clamp;
            clamp.setIgnoreZ( altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
            cx = clamp.push( workingSet, cx );
        }

        SubstituteModelFilter sub( style );
        sub.setClustering( *_options.clustering() );
        if ( model->scale().isSet() )
            sub.setModelMatrix( osg::Matrixd::scale( *model->scale() ) );

        cx = sub.push( workingSet, cx );
        result = sub.getNode();
    }

    // extruded geometry
    else if ( extrusion && ( line || polygon ) )
    {
        if ( altitude && altitude->clamping() != AltitudeSymbol::CLAMP_NONE )
        {
            ClampFilter clamp;
            clamp.setIgnoreZ( altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
            cx = clamp.push( workingSet, cx );
        }

        ExtrudeGeometryFilter extrude;
        if ( extrusion )
        {
            extrude.setExtrusionHeight( *extrusion->height() );
            extrude.setFlatten( *extrusion->flatten() );
        }
        if ( polygon )
        {
            extrude.setColor( polygon->fill()->color() );
        }

        result = extrude.push( workingSet, cx );
    }

    // simple geometry
    else if ( point || line || polygon )
    {
        if ( altitude && altitude->clamping() != AltitudeSymbol::CLAMP_NONE )
        {
            ClampFilter clamp;
            clamp.setIgnoreZ( altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN );
            cx = clamp.push( workingSet, cx );
        }

        BuildGeometryFilter filter( style );
        if ( _options.maxGranularity().isSet() )
            filter.maxGranularity() = *_options.maxGranularity();
        if ( _options.mergeGeometry().isSet() )
            filter.mergeGeometry() = *_options.mergeGeometry();
        cx = filter.push( workingSet, cx );
        result = filter.getNode();
    }

    else // insufficient symbology
    {
        OE_WARN << LC << "Insufficient symbology; no geometry created" << std::endl;
    }
    
    // install the localization transform if necessary.
    if ( cx.hasReferenceFrame() )
    {
        osg::MatrixTransform* delocalizer = new osg::MatrixTransform( cx.inverseReferenceFrame() );
        delocalizer->addChild( result.get() );
        result = delocalizer;
    }

    return result.release();
}
