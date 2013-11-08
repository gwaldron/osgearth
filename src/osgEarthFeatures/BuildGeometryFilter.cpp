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
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/PolygonizeLines>
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/Utils>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osg/Point>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osgDB/WriteFile>
#include <osg/Version>

#define LC "[BuildGeometryFilter] "

#define OE_TEST OE_NULL

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

BuildGeometryFilter::BuildGeometryFilter( const Style& style ) :
_style        ( style ),
_maxAngle_deg ( 1.0 ),
_geoInterp    ( GEOINTERP_RHUMB_LINE )
{
    //nop
}

osg::Geode*
BuildGeometryFilter::processPolygons(FeatureList& features, const FilterContext& context)
{
    osg::Geode* geode = new osg::Geode();

    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS = 0L;

    // set up the reference system info:
    if ( context.isGeoreferenced() )
    {
        makeECEF   = context.getSession()->getMapInfo().isGeocentric();
        featureSRS = context.extent()->getSRS();
        mapSRS     = context.getSession()->getMapInfo().getProfile()->getSRS();
    }

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        GeometryIterator parts( input->getGeometry(), false );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip geometry that is invalid for a polygon
            if ( part->size() < 3 )
                continue;

            // access the polygon symbol, and bail out if there isn't one
            const PolygonSymbol* poly =
                input->style().isSet() && input->style()->has<PolygonSymbol>() ? input->style()->get<PolygonSymbol>() :
                _style.get<PolygonSymbol>();
            if ( !poly )
                continue;

            // resolve the color:
            osg::Vec4f primaryColor = poly->fill()->color();
            
            osg::ref_ptr<osg::Geometry> osgGeom = new osg::Geometry();
            osgGeom->setUseVertexBufferObjects( true );
            osgGeom->setUseDisplayList( false );

            // are we embedding a feature name?
            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
                osgGeom->setName( name );
            }

            // build the geometry:
            buildPolygon(part, featureSRS, mapSRS, makeECEF, true, osgGeom);

            osg::Vec3Array* allPoints = static_cast<osg::Vec3Array*>(osgGeom->getVertexArray());
            
            // subdivide the mesh if necessary to conform to an ECEF globe:
            if ( makeECEF )
            {
                double threshold = osg::DegreesToRadians( *_maxAngle_deg );
                OE_DEBUG << "Running mesh subdivider with threshold " << *_maxAngle_deg << std::endl;

                MeshSubdivider ms( _world2local, _local2world );
                //ms.setMaxElementsPerEBO( INT_MAX );
                if ( input->geoInterp().isSet() )
                    ms.run( *osgGeom, threshold, *input->geoInterp() );
                else
                    ms.run( *osgGeom, threshold, *_geoInterp );
            }

            // assign the primary color array. PER_VERTEX required in order to support
            // vertex optimization later
            osg::Vec4Array* colors = new osg::Vec4Array;
            colors->assign( osgGeom->getVertexArray()->getNumElements(), primaryColor );
            osgGeom->setColorArray( colors );
            osgGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

            geode->addDrawable( osgGeom );

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
                context.featureIndex()->tagPrimitiveSets( osgGeom, input );
        }
    }
    
    return geode;
}


osg::Geode*
BuildGeometryFilter::processPolygonizedLines(FeatureList&         features, 
                                             bool                 twosided,
                                             const FilterContext& context)
{
    osg::Geode* geode = new osg::Geode();

    // establish some referencing
    bool                    makeECEF   = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS     = 0L;

    if ( context.isGeoreferenced() )
    {
        makeECEF   = context.getSession()->getMapInfo().isGeocentric();
        featureSRS = context.extent()->getSRS();
        mapSRS     = context.getSession()->getMapInfo().getProfile()->getSRS();
    }

    // iterate over all features.
    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* input = i->get();

        // extract the required line symbol; bail out if not found.
        const LineSymbol* line =
            input->style().isSet() && input->style()->has<LineSymbol>() ? input->style()->get<LineSymbol>() :
            _style.get<LineSymbol>();
        if ( !line )
            continue;

        // The operator we'll use to make lines into polygons.
        PolygonizeLinesOperator polygonizer( *line->stroke() );

        // iterate over all the feature's geometry parts. We will treat
        // them as lines strings.
        GeometryIterator parts( input->getGeometry(), true );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // if the underlying geometry is a ring (or a polygon), close it so the
            // polygonizer will generate a closed loop.
            Ring* ring = dynamic_cast<Ring*>(part);
            if ( ring )
                ring->close();

            // skip invalid geometry
            if ( part->size() < 2 )
                continue;

            // transform the geometry into the target SRS and localize it about 
            // a local reference point.
            osg::ref_ptr<osg::Vec3Array> verts   = new osg::Vec3Array();
            osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
            transformAndLocalize( part->asVector(), featureSRS, verts.get(), normals.get(), mapSRS, _world2local, makeECEF );

            // turn the lines into polygons.
            osg::Geometry* geom = polygonizer( verts.get(), normals.get(), twosided );
            if ( geom )
            {
                geode->addDrawable( geom );
            }

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
                context.featureIndex()->tagPrimitiveSets( geom, input );
        }

        polygonizer.installShaders( geode->getOrCreateStateSet() );
    }
    return geode;
}


osg::Geode*
BuildGeometryFilter::processLines(FeatureList& features, const FilterContext& context)
{
    osg::Geode* geode = new osg::Geode();

    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS = 0L;

    // set up referencing information:
    if ( context.isGeoreferenced() )
    {
        makeECEF   = context.getSession()->getMapInfo().isGeocentric();
        featureSRS = context.extent()->getSRS();
        mapSRS     = context.getSession()->getMapInfo().getProfile()->getSRS();
    }

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        GeometryIterator parts( input->getGeometry(), true );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip invalid geometry for lines.
            if ( part->size() < 2 )
                continue;

            // extract the required line symbol; bail out if not found.
            const LineSymbol* line = 
                input->style().isSet() && input->style()->has<LineSymbol>() ? input->style()->get<LineSymbol>() :
                _style.get<LineSymbol>();
            if ( !line )
                continue;

            // if the underlying geometry is a ring (or a polygon), use a line loop; otherwise
            // use a line strip.
            GLenum primMode = dynamic_cast<Ring*>(part) ? GL_LINE_LOOP : GL_LINE_STRIP;

            // resolve the color:
            osg::Vec4f primaryColor = line->stroke()->color();
            
            osg::ref_ptr<osg::Geometry> osgGeom = new osg::Geometry();
            osgGeom->setUseVertexBufferObjects( true );
            osgGeom->setUseDisplayList( false );

            // embed the feature name if requested. Warning: blocks geometry merge optimization!
            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
                osgGeom->setName( name );
            }

            // build the geometry:
            osg::Vec3Array* allPoints = new osg::Vec3Array();

            transformAndLocalize( part->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

            osgGeom->addPrimitiveSet( new osg::DrawArrays(primMode, 0, allPoints->getNumElements()) );
            osgGeom->setVertexArray( allPoints );

            if ( input->style().isSet() )
            {
                //TODO: re-evaluate this. does it hinder geometry merging?
                applyLineSymbology( osgGeom->getOrCreateStateSet(), line );
            }
            
            // subdivide the mesh if necessary to conform to an ECEF globe:
            if ( makeECEF && !line->tessellation().isSetTo(0) )
            {
                double threshold = osg::DegreesToRadians( *_maxAngle_deg );
                OE_DEBUG << "Running mesh subdivider with threshold " << *_maxAngle_deg << std::endl;

                MeshSubdivider ms( _world2local, _local2world );
                //ms.setMaxElementsPerEBO( INT_MAX );
                if ( input->geoInterp().isSet() )
                    ms.run( *osgGeom, threshold, *input->geoInterp() );
                else
                    ms.run( *osgGeom, threshold, *_geoInterp );
            }

            // assign the primary color (PER_VERTEX required for later optimization)
            osg::Vec4Array* colors = new osg::Vec4Array;
            colors->assign( osgGeom->getVertexArray()->getNumElements(), primaryColor );
            osgGeom->setColorArray( colors );
            osgGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

            geode->addDrawable( osgGeom );

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
                context.featureIndex()->tagPrimitiveSets( osgGeom, input );
        }
    }
    
    return geode;
}


osg::Geode*
BuildGeometryFilter::processPoints(FeatureList& features, const FilterContext& context)
{
    osg::Geode* geode = new osg::Geode();

    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS = 0L;

    // set up referencing information:
    if ( context.isGeoreferenced() )
    {
        makeECEF   = context.getSession()->getMapInfo().isGeocentric();
        featureSRS = context.extent()->getSRS();
        mapSRS     = context.getSession()->getMapInfo().getProfile()->getSRS();
    }

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        GeometryIterator parts( input->getGeometry(), true );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip invalid geometry for lines.
            if ( part->size() < 2 )
                continue;

            // extract the required line symbol; bail out if not found.
            const PointSymbol* point =
                input->style().isSet() && input->style()->has<PointSymbol>() ? input->style()->get<PointSymbol>() :
                _style.get<PointSymbol>();
            if ( !point )
                continue;

            // resolve the color:
            osg::Vec4f primaryColor = point->fill()->color();
            
            osg::ref_ptr<osg::Geometry> osgGeom = new osg::Geometry();
            osgGeom->setUseVertexBufferObjects( true );
            osgGeom->setUseDisplayList( false );

            // embed the feature name if requested. Warning: blocks geometry merge optimization!
            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
                osgGeom->setName( name );
            }

            // build the geometry:
            osg::Vec3Array* allPoints = new osg::Vec3Array();

            transformAndLocalize( part->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

            osgGeom->addPrimitiveSet( new osg::DrawArrays(GL_POINTS, 0, allPoints->getNumElements()) );
            osgGeom->setVertexArray( allPoints );

            if ( input->style().isSet() )
            {
                //TODO: re-evaluate this. does it hinder geometry merging?
                applyPointSymbology( osgGeom->getOrCreateStateSet(), point );
            }

            // assign the primary color (PER_VERTEX required for later optimization)
            osg::Vec4Array* colors = new osg::Vec4Array;
            colors->assign( osgGeom->getVertexArray()->getNumElements(), primaryColor );
            osgGeom->setColorArray( colors );
            osgGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

            geode->addDrawable( osgGeom );

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
                context.featureIndex()->tagPrimitiveSets( osgGeom, input );
        }
    }
    
    return geode;
}

// builds and tessellates a polygon (with or without holes)
void
BuildGeometryFilter::buildPolygon(Geometry*               ring,
                                  const SpatialReference* featureSRS,
                                  const SpatialReference* mapSRS,
                                  bool                    makeECEF,
                                  bool                    tessellate,
                                  osg::Geometry*          osgGeom)
{
    if ( !ring->isValid() )
        return;

    int totalPoints = ring->getTotalPointCount();
    osg::Vec3Array* allPoints = new osg::Vec3Array();
    transformAndLocalize( ring->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

    GLenum mode = GL_LINE_LOOP;
    osgGeom->addPrimitiveSet( new osg::DrawArrays( mode, 0, ring->size() ) );

    Polygon* poly = dynamic_cast<Polygon*>(ring);
    if ( poly )
    {
        int offset = ring->size();

        for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
        {
            Geometry* hole = h->get();
            if ( hole->isValid() )
            {
                transformAndLocalize( hole->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

                osgGeom->addPrimitiveSet( new osg::DrawArrays( mode, offset, hole->size() ) );
                offset += hole->size();
            }            
        }
    }
    osgGeom->setVertexArray( allPoints );

    if ( tessellate )
    {
        osgUtil::Tessellator tess;
        tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
        tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
        tess.retessellatePolygons( *osgGeom );
    }

    //// Normal computation.
    //// Not completely correct, but better than no normals at all. TODO: update this
    //// to generate a proper normal vector in ECEF mode.
    ////
    //// We cannot accurately rely on triangles from the tessellation, since we could have
    //// very "degraded" triangles (close to a line), and the normal computation would be bad.
    //// In this case, we would have to average the normal vector over each triangle of the polygon.
    //// The Newell's formula is simpler and more direct here.
    //osg::Vec3 normal( 0.0, 0.0, 0.0 );
    //for ( size_t i = 0; i < poly->size(); ++i )
    //{
    //    osg::Vec3 pi = (*poly)[i];
    //    osg::Vec3 pj = (*poly)[ (i+1) % poly->size() ];
    //    normal[0] += ( pi[1] - pj[1] ) * ( pi[2] + pj[2] );
    //    normal[1] += ( pi[2] - pj[2] ) * ( pi[0] + pj[0] );
    //    normal[2] += ( pi[0] - pj[0] ) * ( pi[1] + pj[1] );
    //}
    //normal.normalize();

    //osg::Vec3Array* normals = new osg::Vec3Array();
    //normals->push_back( normal );
    //osgGeom->setNormalArray( normals );
    //osgGeom->setNormalBinding( osg::Geometry::BIND_OVERALL );
}


osg::Node*
BuildGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    osg::ref_ptr<osg::Group> result = new osg::Group();

    computeLocalizers( context );

    const LineSymbol*    line  = _style.get<LineSymbol>();
    const PolygonSymbol* poly  = _style.get<PolygonSymbol>();
    const PointSymbol*   point = _style.get<PointSymbol>();

    // bin the feautres into polygons, lines, polygonized lines, and points.
    FeatureList polygons;
    FeatureList lines;
    FeatureList polygonizedLines;
    FeatureList points;

    for(FeatureList::iterator i = input.begin(); i != input.end(); ++i)
    {
        Feature* f = i->get();

        // first consider the overall style:
        bool has_polysymbol     = poly != 0L;
        bool has_linesymbol     = line != 0L && line->stroke()->widthUnits() == Units::PIXELS;
        bool has_polylinesymbol = line != 0L && line->stroke()->widthUnits() != Units::PIXELS;
        bool has_pointsymbol    = point != 0L;

        // if the featue has a style set, that overrides:
        if ( f->style().isSet() )
        {
            has_polysymbol     = has_polysymbol     || (f->style()->has<PolygonSymbol>());
            has_linesymbol     = has_linesymbol     || (f->style()->has<LineSymbol>() && f->style()->get<LineSymbol>()->stroke()->widthUnits() == Units::PIXELS);
            has_polylinesymbol = has_polylinesymbol || (f->style()->has<LineSymbol>() && f->style()->get<LineSymbol>()->stroke()->widthUnits() != Units::PIXELS);
            has_pointsymbol    = has_pointsymbol    || (f->style()->has<PointSymbol>());
        }

        // if no style is set, use the geometry type:
        if ( !has_polysymbol && !has_linesymbol && !has_polylinesymbol && !has_pointsymbol && f->getGeometry() )
        {
            switch( f->getGeometry()->getComponentType() )
            {
            case Geometry::TYPE_LINESTRING:
            case Geometry::TYPE_RING:
                f->style()->add( new LineSymbol() );
                has_linesymbol = true;
                break;

            case Geometry::TYPE_POINTSET:
                f->style()->add( new PointSymbol() );
                has_pointsymbol = true;
                break;

            case Geometry::TYPE_POLYGON:
                f->style()->add( new PolygonSymbol() );
                has_polysymbol = true;
                break;
            }
        }

        if ( has_polysymbol )
            polygons.push_back( f );

        if ( has_linesymbol )
            lines.push_back( f );

        if ( has_polylinesymbol )
            polygonizedLines.push_back( f );

        if ( has_pointsymbol )
            points.push_back( f );
    }

    // process them separately.

    if ( polygons.size() > 0 )
    {
        OE_TEST << LC << "Building " << polygons.size() << " polygons." << std::endl;
        osg::ref_ptr<osg::Geode> geode = processPolygons(polygons, context);
        if ( geode->getNumDrawables() > 0 )
        {
            if ( !context.featureIndex() )
            {
                osgUtil::Optimizer o;
                o.optimize( geode.get(), 
                    osgUtil::Optimizer::MERGE_GEOMETRY |
                    osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                    osgUtil::Optimizer::INDEX_MESH |
                    osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
            }
            result->addChild( geode.get() );
        }
    }

    if ( polygonizedLines.size() > 0 )
    {
        OE_TEST << LC << "Building " << polygonizedLines.size() << " polygonized lines." << std::endl;
        bool twosided = polygons.size() > 0 ? false : true;
        osg::ref_ptr<osg::Geode> geode = processPolygonizedLines(polygonizedLines, twosided, context);
        if ( geode->getNumDrawables() > 0 )
        {
            if ( !context.featureIndex() )
            {
                osgUtil::Optimizer o;
                o.optimize( geode.get(), 
                    osgUtil::Optimizer::MERGE_GEOMETRY |
                    osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                    osgUtil::Optimizer::INDEX_MESH |
                    osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
            }
            result->addChild( geode.get() );
        }
    }

    if ( lines.size() > 0 )
    {
        OE_TEST << LC << "Building " << lines.size() << " lines." << std::endl;
        osg::ref_ptr<osg::Geode> geode = processLines(lines, context);
        if ( geode->getNumDrawables() > 0 )
        {
            if ( !context.featureIndex() )
            {
                osgUtil::Optimizer o;
                o.optimize( geode.get(), 
                    osgUtil::Optimizer::MERGE_GEOMETRY );
            }
            applyLineSymbology( geode->getOrCreateStateSet(), line );
            result->addChild( geode.get() );
        }
    }

    if ( points.size() > 0 )
    {
        OE_TEST << LC << "Building " << points.size() << " points." << std::endl;
        osg::ref_ptr<osg::Geode> geode = processPoints(points, context);
        if ( geode->getNumDrawables() > 0 )
        {
            if ( !context.featureIndex() )
            {
                osgUtil::Optimizer o;
                o.optimize( geode.get(), 
                    osgUtil::Optimizer::MERGE_GEOMETRY );
            }
            applyPointSymbology( geode->getOrCreateStateSet(), point );
            result->addChild( geode.get() );
        }
    }

    if ( result->getNumChildren() > 0 )
    {
        // apply the delocalization matrix for no-jitter
        return delocalize( result.release() );
    }
    else
    {
        return 0L;
    }
}
