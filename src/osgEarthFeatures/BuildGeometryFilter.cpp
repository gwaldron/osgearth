/* -*_maxPolyTilingAngle_deg-c++-*- */
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
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/PolygonizeLines>
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/ResourceCache>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/Tessellator>
#include <osgEarth/Utils>
#include <osgEarth/Clamping>
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
_geoInterp    ( GEOINTERP_RHUMB_LINE ),
_maxPolyTilingAngle_deg( 45.0f )
{
    //nop
}

osg::Geode*
BuildGeometryFilter::processPolygons(FeatureList& features, FilterContext& context)
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

        if ( !poly ) {
            OE_TEST << LC << "Discarding feature with no poly symbol\n";
            continue;
        }

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

            part->removeDuplicates();

            // skip geometry that is invalid for a polygon
            if ( part->size() < 3 ) {
                OE_TEST << LC << "Discarding illegal part (less than 3 verts)\n";
                continue;
            }

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

            // collect all the pre-transformation HAT (Z) values.
            osg::ref_ptr<osg::FloatArray> hats = new osg::FloatArray();
            hats->reserve( part->size() );
            for(Geometry::const_iterator i = part->begin(); i != part->end(); ++i )
                hats->push_back( i->z() );

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
                    OE_TEST << "Running mesh subdivider with threshold " << *_maxAngle_deg << std::endl;

                    MeshSubdivider ms( _world2local, _local2world );
                    if ( input->geoInterp().isSet() )
                        ms.run( *osgGeom, threshold, *input->geoInterp() );
                    else
                        ms.run( *osgGeom, threshold, *_geoInterp );
                }

                // assign the primary color array. PER_VERTEX required in order to support
                // vertex optimization later
                unsigned count = osgGeom->getVertexArray()->getNumElements();
                osg::Vec4Array* colors = new osg::Vec4Array;
                colors->assign( count, primaryColor );
                osgGeom->setColorArray( colors );
                osgGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

                geode->addDrawable( osgGeom );

                // record the geometry's primitive set(s) in the index:
                if ( context.featureIndex() )
                    context.featureIndex()->tagDrawable( osgGeom, input );
        
                // install clamping attributes if necessary
                if (_style.has<AltitudeSymbol>() &&
                    _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU)
                {
                    Clamping::applyDefaultClampingAttrs( osgGeom, input->getDouble("__oe_verticalOffset", 0.0) );
                }
            }
            else
            {
                OE_TEST << LC << "Oh no. buildAndTilePolygon returned nothing.\n";
            }
        }
    }
    
    OE_TEST << LC << "Num drawables = " << geode->getNumDrawables() << "\n";
    return geode;
}


osg::Geode*
BuildGeometryFilter::processPolygonizedLines(FeatureList&   features, 
                                             bool           twosided,
                                             FilterContext& context)
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

            // collect all the pre-transformation HAT (Z) values.
            osg::ref_ptr<osg::FloatArray> hats = new osg::FloatArray();
            hats->reserve( part->size() );
            for(Geometry::const_iterator i = part->begin(); i != part->end(); ++i )
                hats->push_back( i->z() );

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
                context.featureIndex()->tagDrawable( geom, input );
        
            // install clamping attributes if necessary
            if (_style.has<AltitudeSymbol>() &&
                _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU)
            {
                Clamping::applyDefaultClampingAttrs( geom, input->getDouble("__oe_verticalOffset", 0.0) );
                Clamping::setHeights( geom, hats.get() );
            }
        }

        polygonizer.installShaders( geode );
    }
    return geode;
}


osg::Geode*
BuildGeometryFilter::processLines(FeatureList& features, FilterContext& context)
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

            // collect all the pre-transformation HAT (Z) values.
            osg::ref_ptr<osg::FloatArray> hats = new osg::FloatArray();
            hats->reserve( part->size() );
            for(Geometry::const_iterator i = part->begin(); i != part->end(); ++i )
                hats->push_back( i->z() );

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
            
            // subdivide the mesh if necessary to conform to an ECEF globe;
            // but if the tessellation is set to zero, or if the style specifies a
            // tessellation size, skip this step.
            if ( makeECEF && !line->tessellation().isSetTo(0) && !line->tessellationSize().isSet() )
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
                context.featureIndex()->tagDrawable( osgGeom, input );
        
            // install clamping attributes if necessary
            if (_style.has<AltitudeSymbol>() &&
                _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU)
            {
                Clamping::applyDefaultClampingAttrs( osgGeom, input->getDouble("__oe_verticalOffset", 0.0) );
                Clamping::setHeights( osgGeom, hats.get() );
            }
        }
    }
    
    return geode;
}


osg::Geode*
BuildGeometryFilter::processPoints(FeatureList& features, FilterContext& context)
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

            // collect all the pre-transformation HAT (Z) values.
            osg::ref_ptr<osg::FloatArray> hats = new osg::FloatArray();
            hats->reserve( part->size() );
            for(Geometry::const_iterator i = part->begin(); i != part->end(); ++i )
                hats->push_back( i->z() );

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
                context.featureIndex()->tagDrawable( osgGeom, input );
        
            // install clamping attributes if necessary
            if (_style.has<AltitudeSymbol>() &&
                _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU)
            {            
                Clamping::applyDefaultClampingAttrs( osgGeom, input->getDouble("__oe_verticalOffset", 0.0) );
                Clamping::setHeights( osgGeom, hats.get() );
            }
        }
    }
    
    return geode;
}

// Borrowed from MeshConsolidator.cpp
template<typename FROM, typename TO>
osg::PrimitiveSet* copy( FROM* src, unsigned offset )
{
    TO* newDE = new TO( src->getMode() );
    newDE->reserve( src->size() );
    for( typename FROM::const_iterator i = src->begin(); i != src->end(); ++i )
        newDE->push_back( (*i) + offset );
    return newDE;
}


/**
 * Converts an osg::Geometry to use osg::DrawElementsUInt if it doesn't already.
 * This only works on Geometries that are already using DrawElementsUInt, DrawElementsUByte, or DrawElementsUShort
 * We do this to normalize the primitive set types so that we can merge multiple geometries 
 * into one later down the road.
 */
void convertToDrawElementsUInt(osg::Geometry* geometry)
{
    for (unsigned int i = 0; i < geometry->getNumPrimitiveSets(); i++)
    {
        osg::PrimitiveSet* ps = geometry->getPrimitiveSet(i);
        // See if it's already a DrawElementsUInt and do nothing if it is.
        osg::DrawElementsUInt* deUint = dynamic_cast<osg::DrawElementsUInt*>(ps);
        if (!deUint)
        {
            // Copy values from the existing primitive set to a new DrawElementsUInt
            osg::PrimitiveSet* newPS = 0;
            if (dynamic_cast<osg::DrawElementsUByte*>(ps))
            {
                newPS = copy<osg::DrawElementsUByte, osg::DrawElementsUInt>(static_cast<osg::DrawElementsUByte*>(ps), 0);
            }
            else if (dynamic_cast<osg::DrawElementsUShort*>(ps))
            {             
                newPS = copy<osg::DrawElementsUShort, osg::DrawElementsUInt>(static_cast<osg::DrawElementsUShort*>(ps), 0);
            }

            // Set the new primitive set
            if (newPS)
            {
                geometry->setPrimitiveSet(i, newPS);
            }
        }
    }
}

/**
 * Tesselates an osg::Geometry using the osgEarth tesselator.
 * If it fails, fall back to the osgUtil tesselator.
 */
bool tesselateGeometry(osg::Geometry* geometry)
{
    osgEarth::Tessellator oeTess;
    if ( !oeTess.tessellateGeometry(*geometry) )
    {
        osgUtil::Tessellator tess;
        tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
        tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
        tess.retessellatePolygons( *geometry );
    }

    // Make sure all of the primitive sets are osg::DrawElementsUInt
    // The osgEarth tesselator will occassionally fail, and we fall back to the osgUtil::Tesselator which can produce a mix
    // of DrawElementsUInt, DrawElementsUByte and DrawElementsUShort depending on the number of vertices.
    convertToDrawElementsUInt(geometry);
    return true;
}

/**
 * Tiles the Geometry into the given number of columns and rows
 */
void tileGeometry(Geometry* geometry, const SpatialReference* featureSRS, unsigned int numCols, unsigned int numRows, GeometryCollection& out)
{
    // Clear the output list.
    out.clear();

    Bounds b = geometry->getBounds();
    double tw = b.width() / (double)numCols;
    double th = b.height() / (double)numRows;

    // Get the average Z, since GEOS will set teh Z of new verts to that of the cropping polygon,
    // which is stupid but that's how it is.
    double z = 0.0;
    for(unsigned i=0; i<geometry->size(); ++i)
        z += geometry->at(i).z();
    z /= geometry->size();

    osg::ref_ptr<Polygon> poly = new Polygon;
    poly->resize( 4 );        

    for(int x=0; x<(int)numCols; ++x)
    {
        for(int y=0; y<(int)numRows; ++y)
        {
            (*poly)[0].set( b.xMin() + tw*(double)x,     b.yMin() + th*(double)y,     z );
            (*poly)[1].set( b.xMin() + tw*(double)(x+1), b.yMin() + th*(double)y,     z );
            (*poly)[2].set( b.xMin() + tw*(double)(x+1), b.yMin() + th*(double)(y+1), z );
            (*poly)[3].set( b.xMin() + tw*(double)x,     b.yMin() + th*(double)(y+1), z );

            osg::ref_ptr<Geometry> ringTile;
            if ( geometry->crop(poly.get(), ringTile) )
            {
                // Use an iterator since crop could return a multi-polygon
                GeometryIterator gi( ringTile.get(), false );
                while( gi.hasMore() )
                {
                    Geometry* geom = gi.next();
                    out.push_back( geom );                                                
                }
            }
        }
    }
}

/**
 * Tiles the geometry up until all the cells have less than given number of points.
 */
void downsizeGeometry(Geometry* geometry, const SpatialReference* featureSRS, unsigned int maxPoints, GeometryCollection& out)
{
    // If the geometyr is greater than the maximum number of points, we need to tile it up further.
    if (geometry->size() > maxPoints)
    {
        OE_NOTICE << "Downsizing geometry of size " << geometry->size() << std::endl;
        // Tile the geometry.
        GeometryCollection tmp;
        tileGeometry(geometry, featureSRS, 2, 2, tmp );
        
        for (unsigned int i = 0; i < tmp.size(); i++)
        {
            Geometry* g = tmp[i].get();

            // If the generated geometry still has too many points, continue to downsample it recursively.
            if (g->size() > maxPoints)
            {
                // We pass "out" as the destination here since downsizeGeometry will only append tiles that are less than the max size.
                downsizeGeometry( g, featureSRS, maxPoints, out );
            }
            else
            {
                // Append the geometry to the output list.
                out.push_back( g );
            }
        }
    }
    else
    {
        // The geometry is valid, so add it to the output list.
        out.push_back( geometry );
    }
}

/**
 * Prepares a geometry into a grid if it is too big geospatially to have a sensible local tangent plane
 * We will also tile the geometry if it just has too many points to speed up the tesselator.
 */
void prepareForTesselation(Geometry* geometry, const SpatialReference* featureSRS, double targetTileSizeDeg, unsigned int maxPointsPerTile, GeometryCollection& out)
{
    // Clear the output list.
    GeometryCollection tiles;
    
    unsigned int count = geometry->size();

    unsigned int tx = 1;
    unsigned int ty = 1;

    // Tile the geometry if it's geospatial size is too large to have a sensible local tangent plane.
    GeoExtent featureExtentDeg = GeoExtent(featureSRS, geometry->getBounds()).transform(SpatialReference::create("wgs84"));

    // Tile based on the extent
    if ( featureExtentDeg.width() > targetTileSizeDeg  || featureExtentDeg.height() > targetTileSizeDeg)
    {
        // Determine the tile size based on the extent.
        tx = ceil( featureExtentDeg.width() / targetTileSizeDeg );
        ty = ceil (featureExtentDeg.height() / targetTileSizeDeg );        
    }
    else if (count > maxPointsPerTile)
    {
        // Determine the size based on the number of points.
        unsigned numTiles = ((double)count / (double)maxPointsPerTile) + 1u;
        tx = ceil(sqrt((double)numTiles));
        ty = tx;        
    }

    if (tx == 1 && ty == 1)
    {
        // The geometry doesn't need modified so just add it to the list.
        tiles.push_back( geometry );
    }
    else
    {
        tileGeometry( geometry, featureSRS, tx, ty, tiles );
    }

    out.clear();

#if 1
    // Just copy the output tiles to the output.
    std::copy(tiles.begin(), tiles.end(), std::back_inserter(out));
#else
    // Calling this code will recursively subdivide the cells based on the number of points they have.
    // This works but it will produces a non-regular grid which doesn't render well in geocentric
    // due to the curvature of the earth so we disable it for now.
    //
    // Reduce the size of the tiles if needed.
    for (unsigned int i = 0; i < tiles.size(); i++)
    {
        if (tiles[i]->size() > maxPointsPerTile)
        {
            GeometryCollection tmp;
            downsizeGeometry(tiles[i].get(), featureSRS, maxPointsPerTile, tmp);
            std::copy(tmp.begin(), tmp.end(), std::back_inserter(out));
        }
        else
        {
            out.push_back( tiles[i].get() );
        }
    }
#endif
}

void
BuildGeometryFilter::tileAndBuildPolygon(Geometry*               ring,
                                         const SpatialReference* featureSRS,
                                         const SpatialReference* mapSRS,
                                         bool                    makeECEF,
                                         bool                    tessellate,
                                         osg::Geometry*          osgGeom,
                                         const osg::Matrixd      &world2local)
{
#define MAX_POINTS_PER_CROP_TILE 1024
//#define TARGET_TILE_SIZE_EXTENT_DEGREES 5

    if ( ring == 0L )
    {
        OE_WARN << LC << "Ring is NULL.\n";
        return;
    }
    
    // Tile the incoming polygon if necessary
    // NB: this breaks down at higher latitudes; see https://github.com/gwaldron/osgearth/issues/746

    GeometryCollection tiles;
    if (_maxPolyTilingAngle_deg.isSet())
        prepareForTesselation( ring, featureSRS, _maxPolyTilingAngle_deg.get(), MAX_POINTS_PER_CROP_TILE, tiles);    
    else
        tiles.push_back( ring );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    //OE_NOTICE << LC << "TABP: tiles = " << tiles.size() << "\n";

    // Process each ring independently
    for (int ringIndex = 0; ringIndex < tiles.size(); ringIndex++)
    {
        Geometry* geom = tiles[ringIndex].get();
        if (geom)
        {
            // temporary target geometry for this cell:
            osg::ref_ptr<osg::Geometry> temp = new osg::Geometry();
            temp->setVertexArray( new osg::Vec3Array() );

            // establish a local plane for this cell based on its centroid:
            GeoPoint cellCenter(featureSRS, geom->getBounds().center());
            cellCenter.transform(mapSRS, cellCenter);                        
            osg::Matrix world2cell;
            cellCenter.createWorldToLocal( world2cell );

            // build the localized polygon:
            buildPolygon(geom, featureSRS, mapSRS, makeECEF, temp.get(), world2cell);

            // if successful, transform the verts back into our master LTP:
            if ( temp->getNumPrimitiveSets() > 0 )
            {
                // Tesselate the polygon while the coordinates are still in the LTP
                if (tesselateGeometry( temp.get() ))
                {
                    osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(temp->getVertexArray());
                    if ( verts->getNumElements() > 0 )
                    {
                        // Convert the coordinates back to the master LTP.
                        // This is ok, but you will probably run into precision errors if the tile size is very large.
                        osg::Matrix cell2world;
                        cell2world.invert( world2cell );
                        osg::Matrix cell2local = cell2world * world2local; // pre-multiply to avoid precision loss

                        for(int i=0; i<verts->size(); ++i)
                        {
                            (*verts)[i] = (*verts)[i] * cell2local;
                        }
                        geode->addDrawable(temp.get());
                    }
                }
            }
        }
        else
        {
            OE_TEST << LC << "TABP: Uh oh. found a non-Ring geometry: " << geom->toString(geom->getComponentType()) << "\n";
        }
    }

    // The geode is going to contain all of our polygons now, so merge them into one.
    osgUtil::Optimizer optimizer;
    osgUtil::Optimizer::MergeGeometryVisitor mgv;
    // We only want one Geometry, so don't limit the number of vertices.
    mgv.setTargetMaximumNumberOfVertices(UINT_MAX);
    mgv.apply( *geode.get() );

    // and copy them into the output geometry.
    if ( geode->getNumDrawables() > 0 )
    {
        // If we have more than one drawable after the MergeGeometryVisitor ran, we have a problem so
        // dump out some info to help debug.
        if (geode->getNumDrawables() != 1)
        {
            OE_WARN << LC << "MergeGeometryVisitor failed to merge geometries into a single one.  Num drawables " << geode->getNumDrawables() << std::endl;            
            for (unsigned int i = 0; i < geode->getNumDrawables(); i++)
            {
                osg::Geometry* g = geode->getDrawable(i)->asGeometry();
                if (g)
                {
                    osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(g->getVertexArray());
                    if (verts)
                    {
                        OE_WARN << "Geometry " << i << " has " << verts->size() << " verts" << std::endl;
                        OE_WARN << "Geometry " << i << " has " << g->getNumPrimitiveSets() << " primitive sets" << std::endl;
                        for (unsigned int j = 0; j < g->getNumPrimitiveSets(); j++)
                        {
                            osg::PrimitiveSet* ps = g->getPrimitiveSet(j);
                            OE_WARN << "PrimitiveSet " << j << ps->className() << std::endl;
                        }
                    }
                }
            }
        }
        osgGeom->setVertexArray( geode->getDrawable(0)->asGeometry()->getVertexArray() );
        osgGeom->setPrimitiveSetList( geode->getDrawable(0)->asGeometry()->getPrimitiveSetList() );
    }

    osgUtil::SmoothingVisitor::smooth( *osgGeom );
}

// builds and tessellates a polygon (with or without holes)
void
BuildGeometryFilter::buildPolygon(Geometry*               ring,
                                  const SpatialReference* featureSRS,
                                  const SpatialReference* mapSRS,
                                  bool                    makeECEF,
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
            default:
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
            osgUtil::Optimizer::MergeGeometryVisitor mg;
            mg.setTargetMaximumNumberOfVertices(65536);
            geode->accept(mg);

            osgUtil::Optimizer o;
            o.optimize( geode.get(), 
                osgUtil::Optimizer::INDEX_MESH |
                osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                osgUtil::Optimizer::VERTEX_POSTTRANSFORM );

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
            osgUtil::Optimizer::MergeGeometryVisitor mg;
            mg.setTargetMaximumNumberOfVertices(65536);
            geode->accept(mg);

            osgUtil::Optimizer o;
            o.optimize( geode.get(), 
                osgUtil::Optimizer::INDEX_MESH |
                osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                osgUtil::Optimizer::VERTEX_POSTTRANSFORM );

            result->addChild( geode.get() );
        }
    }

    if ( lines.size() > 0 )
    {
        OE_TEST << LC << "Building " << lines.size() << " lines." << std::endl;
        osg::ref_ptr<osg::Geode> geode = processLines(lines, context);
        if ( geode->getNumDrawables() > 0 )
        {
            osgUtil::Optimizer::MergeGeometryVisitor mg;
            mg.setTargetMaximumNumberOfVertices(65536);
            geode->accept(mg);

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
            osgUtil::Optimizer::MergeGeometryVisitor mg;
            mg.setTargetMaximumNumberOfVertices(65536);
            geode->accept(mg);

            applyPointSymbology( geode->getOrCreateStateSet(), point );
            result->addChild( geode.get() );
        }
    }

    // indicate that geometry contains clamping attributes
    if (_style.has<AltitudeSymbol>() &&
        _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU)
    {
        Clamping::installHasAttrsUniform( result->getOrCreateStateSet() );
    }    

    // Prepare buffer objects.
    AllocateAndMergeBufferObjectsVisitor allocAndMerge;
    result->accept( allocAndMerge );


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