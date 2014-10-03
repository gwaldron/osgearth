/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/PolygonizeLines>
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarthSymbology/ResourceCache>
#include <osgEarth/Tessellator>
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
#include <osgUtil/Simplifier>
#include <osgUtil/SmoothingVisitor>
#include <osgDB/WriteFile>
#include <osg/Version>
#include <iterator>

#define LC "[BuildGeometryFilter] "

#define OE_TEST OE_NULL

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

namespace
{
    bool isCCW(double x1, double y1, double x2, double y2, double x3, double y3)
    {
        return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1) > 0.0;
    }

    bool segmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
    {
        return isCCW(x1, y1, x3, y3, x4, y4) != isCCW(x2, y2, x3, y3, x4, y4) && isCCW(x1, y1, x2, y2, x3, y3) != isCCW(x1, y1, x2, y2, x4, y4);
    }

    bool holeCompare(osgEarth::Symbology::Ring* i, osgEarth::Symbology::Ring* j)
    {
        return i->getBounds().xMax() > j->getBounds().xMax();
    }

    bool segmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double &xi, double &yi)
    {
        double d = (y4-y3) * (x2-x1) - (x4-x3) * (y2-y1);

        if (d == 0) return false; // parallel

        double ua = ((x4-x3) * (y1-y3) - (y4-y3) * (x1-x3)) / d;
        double ub = ((x2-x1) * (y1-y3) - (y2-y1) * (x1-x3)) / d;

        if (ua >= 0.0 && ua <= 1.0 && ub >= 0.0 && ub <= 1.0)
        {
            xi = x1 + ua * (x2 - x1);
            yi = y1 + ua * (y2 - y1);

            return true;
        }

        return false;
    }
}

BuildGeometryFilter::BuildGeometryFilter( const Style& style ) :
_style        ( style ),
_maxAngle_deg ( 180.0 ),
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

        // access the polygon symbol, and bail out if there isn't one
        const PolygonSymbol* poly =
            input->style().isSet() && input->style()->has<PolygonSymbol>() ? input->style()->get<PolygonSymbol>() :
            _style.get<PolygonSymbol>();

        if ( !poly )
            continue;

        // run a symbol script if present.
        if ( poly->script().isSet() )
        {
            StringExpression temp( poly->script().get() );
            input->eval( temp, &context );
        }

        GeometryIterator parts( input->getGeometry(), false );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip geometry that is invalid for a polygon
            if ( part->size() < 3 )
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


            // compute localizing matrices or use globals
            osg::Matrixd w2l, l2w;
            if (makeECEF)
            {
                osgEarth::GeoExtent featureExtent(featureSRS);
                featureExtent.expandToInclude(part->getBounds());

                computeLocalizers(context, featureExtent, w2l, l2w);
            }
            else
            {
                w2l = _world2local;
                l2w = _local2world;
            }


            // build the geometry:
            tileAndBuildPolygon(part, featureSRS, mapSRS, makeECEF, true, osgGeom, w2l);
            //buildPolygon(part, featureSRS, mapSRS, makeECEF, true, osgGeom, w2l);

            osg::Vec3Array* allPoints = static_cast<osg::Vec3Array*>(osgGeom->getVertexArray());
            if (allPoints && allPoints->size() > 0)
            {
                // subdivide the mesh if necessary to conform to an ECEF globe:
                if ( makeECEF )
                {

                    //convert back to world coords
                    for( osg::Vec3Array::iterator i = allPoints->begin(); i != allPoints->end(); ++i )
                    {
                        osg::Vec3d v(*i);
                        v = v * l2w;
                        v = v * _world2local;

                        (*i)._v[0] = v[0];
                        (*i)._v[1] = v[1];
                        (*i)._v[2] = v[2];
                    }

                    double threshold = osg::DegreesToRadians( *_maxAngle_deg );
                    OE_DEBUG << "Running mesh subdivider with threshold " << *_maxAngle_deg << std::endl;

                    MeshSubdivider ms( _world2local, _local2world );
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

        // run a symbol script if present.
        if ( line->script().isSet() )
        {
            StringExpression temp( line->script().get() );
            input->eval( temp, &context );
        }

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

        polygonizer.installShaders( geode );
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

        // extract the required line symbol; bail out if not found.
        const LineSymbol* line = 
            input->style().isSet() && input->style()->has<LineSymbol>() ? input->style()->get<LineSymbol>() :
            _style.get<LineSymbol>();

        if ( !line )
            continue;

        // run a symbol script if present.
        if ( line->script().isSet() )
        {
            StringExpression temp( line->script().get() );
            input->eval( temp, &context );
        }

        GeometryIterator parts( input->getGeometry(), true );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip invalid geometry for lines.
            if ( part->size() < 2 )
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

            // extract the required point symbol; bail out if not found.
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

#define CROP_POLYS_BEFORE_TESSELLATING 1

void
BuildGeometryFilter::tileAndBuildPolygon(Geometry*               ring,
                                         const SpatialReference* featureSRS,
                                         const SpatialReference* mapSRS,
                                         bool                    makeECEF,
                                         bool                    tessellate,
                                         osg::Geometry*          osgGeom,
                                         const osg::Matrixd      &world2local)
{
#ifdef CROP_POLYS_BEFORE_TESSELLATING

#define MAX_POINTS_PER_CROP_TILE 1024

    bool built = false;
    unsigned count = ring->getTotalPointCount();
    if ( count > MAX_POINTS_PER_CROP_TILE )
    {
        unsigned tiles = (count / MAX_POINTS_PER_CROP_TILE) + 1u;
        double tx = ceil(sqrt((double)tiles));
        double ty = tx;
        Bounds b = ring->getBounds();
        double tw = b.width() / tx;
        double th = b.height() / ty;

        OE_DEBUG << "Found " << count << " points; cropping to " << tx << " x " << ty << std::endl;

        osg::ref_ptr<Polygon> poly = new Polygon;
        poly->resize( 4 );

        built = true;
        for(int x=0; x<(int)tx; ++x)
        {
            for(int y=0; y<(int)ty; ++y)
            {
                (*poly)[0].set( b.xMin() + tw*(double)x,     b.yMin() + th*(double)y,     0.0 );
                (*poly)[1].set( b.xMin() + tw*(double)(x+1), b.yMin() + th*(double)y,     0.0 );
                (*poly)[2].set( b.xMin() + tw*(double)(x+1), b.yMin() + th*(double)(y+1), 0.0 );
                (*poly)[3].set( b.xMin() + tw*(double)x,     b.yMin() + th*(double)(y+1), 0.0 );
                
                osg::ref_ptr<Geometry> ringTile;
                if ( ring->crop(poly.get(), ringTile) )
                {
                    // Use an iterator since crop could return a multi-polygon
                    GeometryIterator gi( ringTile.get(), false );
                    while( gi.hasMore() )
                    {
                        Geometry* geom = gi.next();
                        buildPolygon(geom, featureSRS, mapSRS, makeECEF, tessellate, osgGeom, world2local);
                    }
                }
                else 
                {
                    // If crop resulted in empty geometry (valid case) ringTile will still be valid,
                    // otherwise we need to process the entire polygon without tiling.
                    if (!ringTile.valid())
                    {
                        //clean up geometry
                        osgGeom->setVertexArray(0L);
                        if (osgGeom->getNumPrimitiveSets())
                            osgGeom->removePrimitiveSet(0, osgGeom->getNumPrimitiveSets());

                        OE_NOTICE << LC << "GEOS crop failed, tessellating feature without tiling." << std::endl;

                        built = false;
                        break;
                    }
                }
            }

            // GEOS failed 
            if (!built)
                break;
        }
    }

    if ( !built )
    {
        buildPolygon(ring, featureSRS, mapSRS, makeECEF, tessellate, osgGeom, world2local);
    }
    

    if ( tessellate )
    {
        osgEarth::Tessellator oeTess;
        if (!oeTess.tessellateGeometry(*osgGeom))
        {
            //fallback to osg tessellator
            OE_DEBUG << LC << "Falling back on OSG tessellator (" << osgGeom->getName() << ")" << std::endl;

            osgUtil::Tessellator tess;
            tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
            tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
            tess.retessellatePolygons( *osgGeom );
        }
    }

#else

    // non-cropped way
    buildPolygon(ring, featureSRS, mapSRS, makeECEF, tessellate, osgGeom, world2local);
    if ( tessellate )
    {
        osgEarth::Tessellator oeTess;
        if (!oeTess.tessellateGeometry(*osgGeom))
        {
            //fallback to osg tessellator
            OE_INFO << LC << "OE Tessellation failed! Using OSG tessellator. (" << osgGeom->getName() << ")" << std::endl;

            osgUtil::Tessellator tess;
            tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
            tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
            tess.retessellatePolygons( *osgGeom );
        }
    }
#endif
}

// builds and tessellates a polygon (with or without holes)
void
BuildGeometryFilter::buildPolygon(Geometry*               ring,
                                  const SpatialReference* featureSRS,
                                  const SpatialReference* mapSRS,
                                  bool                    makeECEF,
                                  bool                    tessellate,
                                  osg::Geometry*          osgGeom,
                                  const osg::Matrixd      &world2local)
{
    if ( !ring->isValid() )
        return;

    ring->rewind(osgEarth::Symbology::Geometry::ORIENTATION_CCW);

    osg::ref_ptr<osg::Vec3Array> allPoints = new osg::Vec3Array();
    transformAndLocalize( ring->asVector(), featureSRS, allPoints.get(), mapSRS, world2local, makeECEF );

    Polygon* poly = dynamic_cast<Polygon*>(ring);
    if ( poly )
    {
        RingCollection ordered(poly->getHoles().begin(), poly->getHoles().end());
        std::sort(ordered.begin(), ordered.end(), holeCompare);

        for( RingCollection::const_iterator h = ordered.begin(); h != ordered.end(); ++h )
        {
            Geometry* hole = h->get();
            if ( hole->isValid() )
            {
                hole->rewind(osgEarth::Symbology::Geometry::ORIENTATION_CW);

                osg::ref_ptr<osg::Vec3Array> holePoints = new osg::Vec3Array();
                transformAndLocalize( hole->asVector(), featureSRS, holePoints.get(), mapSRS, world2local, makeECEF );

                // find the point with the highest x value
                unsigned int hCursor = 0;
                for (unsigned int i=1; i < holePoints->size(); i++)
                {
                    if ((*holePoints)[i].x() > (*holePoints)[hCursor].x())
                      hCursor = i;
                }

                double x1 = (*holePoints)[hCursor].x();
                double y1 = (*holePoints)[hCursor].y();
                double y2 = (*holePoints)[hCursor].y();

                unsigned int edgeCursor = UINT_MAX;
                double edgeDistance = DBL_MAX;
                unsigned int foundPointCursor = UINT_MAX;
                for (unsigned int i=0; i < allPoints->size(); i++)
                {
                    unsigned int next = i == allPoints->size() - 1 ? 0 : i + 1;
                    double xMax = osg::maximum((*allPoints)[i].x(), (*allPoints)[next].x());

                    if (xMax > (*holePoints)[hCursor].x())
                    {
                        double x2 = xMax + 1.0;
                        double x3 = (*allPoints)[i].x();
                        double y3 = (*allPoints)[i].y();
                        double x4 = (*allPoints)[next].x();
                        double y4 = (*allPoints)[next].y();

                        double xi=0.0, yi=0.0;
                        bool intersects = false;
                        unsigned int hitPointCursor = UINT_MAX;
                        if (y1 == y3 && x3 > x1)
                        {
                            xi = x3;
                            hitPointCursor = i;
                            intersects = true;
                        }
                        else if (y1 == y4 && x4 > x1)
                        {
                            xi = x4;
                            hitPointCursor = next;
                            intersects = true;
                        }
                        else if (segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4, xi, yi))
                        {
                            intersects = true;
                        }

                        double dist = (osg::Vec2d(xi, yi) - osg::Vec2d(x1, y1)).length();
                        if (intersects && dist < edgeDistance)
                        {
                            foundPointCursor = hitPointCursor;
                            edgeCursor = hitPointCursor != UINT_MAX ? hitPointCursor : (x3 >= x4 ? i : next);
                            edgeDistance = dist;
                        }
                    }
                }

                if (foundPointCursor == UINT_MAX && edgeCursor != UINT_MAX)
                {
                    // test for intersecting edges between x1 and x2
                    // (skipping the two segments for which edgeCursor is a vert)

                    double x2 = (*allPoints)[edgeCursor].x();
                    y2 = (*allPoints)[edgeCursor].y();

                    bool foundIntersection = false;
                    for (unsigned int i=0; i < allPoints->size(); i++)
                    {
                        unsigned int next = i == allPoints->size() - 1 ? 0 : i + 1;

                        if (i == edgeCursor || next == edgeCursor)
                          continue;

                        double x3 = (*allPoints)[i].x();
                        double y3 = (*allPoints)[i].y();
                        double x4 = (*allPoints)[next].x();
                        double y4 = (*allPoints)[next].y();

                        foundIntersection = foundIntersection || segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4);

                        if (foundIntersection)
                        {
                            unsigned int prev = i == 0 ? allPoints->size() - 1 : i - 1;

                            if (!isCCW((*allPoints)[prev].x(), (*allPoints)[prev].y(), x3, y3, x4, y4))
                            {
                                edgeCursor = i;
                                x2 = (*allPoints)[edgeCursor].x();
                                y2 = (*allPoints)[edgeCursor].y();
                                foundIntersection = false;
                            }
                        }

                    }
                }

                if (edgeCursor != UINT_MAX)
                {
                    // build array of correctly ordered new points to add to the outer loop
                    osg::ref_ptr<osg::Vec3Array> insertPoints = new osg::Vec3Array();
                    insertPoints->reserve(holePoints->size() + 2);

                    unsigned int p = hCursor;
                    do
                    {
                        insertPoints->push_back((*holePoints)[p]);
                        p = p == holePoints->size() - 1 ? 0 : p + 1;
                    } while(p != hCursor);

                    insertPoints->push_back((*holePoints)[hCursor]);
                    insertPoints->push_back((*allPoints)[edgeCursor]);
                    
                    // insert new points into outer loop
                    osg::Vec3Array::iterator it = edgeCursor == allPoints->size() - 1 ? allPoints->end() : allPoints->begin() + (edgeCursor + 1);
                    allPoints->insert(it, insertPoints->begin(), insertPoints->end());
                }
            }
        }
    }
    
    GLenum mode = GL_LINE_LOOP;
    if ( osgGeom->getVertexArray() == 0L )
    {
        osgGeom->addPrimitiveSet( new osg::DrawArrays( mode, 0, allPoints->size() ) );
        osgGeom->setVertexArray( allPoints.get() );
    }
    else
    {
        osg::Vec3Array* v = static_cast<osg::Vec3Array*>(osgGeom->getVertexArray());
        osgGeom->addPrimitiveSet( new osg::DrawArrays( mode, v->size(), allPoints->size() ) );
        //v->reserve(v->size() + allPoints->size());
        std::copy(allPoints->begin(), allPoints->end(), std::back_inserter(*v));
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
