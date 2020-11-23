/* -*_maxPolyTilingAngle_deg-c++-*- */
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
#include <osgEarth/BuildGeometryFilter>
#include <osgEarth/Session>
#include <osgEarth/FeatureSourceIndexNode>
#include <osgEarth/PolygonizeLines>
#include <osgEarth/WireLines>
#include <osgEarth/TextSymbol>
#include <osgEarth/PointSymbol>
#include <osgEarth/LineSymbol>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/MeshSubdivider>
#include <osgEarth/ResourceCache>
#include <osgEarth/Tessellator>
#include <osgEarth/Utils>
#include <osgEarth/Clamping>
#include <osgEarth/LineDrawable>
#include <osgEarth/PointDrawable>
#include <osgEarth/StateSetCache>
#include <osgEarth/Registry>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineStipple>
#include <osg/Point>
#include <osg/MatrixTransform>
#include <osg/TriangleIndexFunctor>
#include <osgText/Text>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgUtil/Simplifier>
#include <osgDB/WriteFile>
#include <osg/Version>
#include <iterator>

#define LC "[BuildGeometryFilter] "

#define OE_TEST OE_NULL

#define USE_GNOMONIC_TESSELLATION

using namespace osgEarth;

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

    bool holeCompare(const osg::ref_ptr<Ring>& i, const osg::ref_ptr<Ring>& j)
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

    void ecef_to_gnomonic(osg::Vec3d& p, const osg::Vec3d& c, const osg::EllipsoidModel* e)
    {
        double lat0, lon0, alt0;
        e->convertXYZToLatLongHeight(c.x(), c.y(), c.z(), lat0, lon0, alt0);

        double lat, lon, alt;
        e->convertXYZToLatLongHeight(p.x(), p.y(), p.z(), lat, lon, alt);

        double d = sin(lat0)*sin(lat) + cos(lat0)*cos(lat)*cos(lon - lon0);
        p.x() = (cos(lat)*sin(lon - lon0)) / d;
        p.y() = (cos(lat0)*sin(lat) - sin(lat0)*cos(lat)*cos(lon - lon0)) / d;
    }
}

BuildGeometryFilter::BuildGeometryFilter( const Style& style ) :
_style        ( style ),
_maxAngle_deg ( 180.0 ),
_geoInterp    ( GEOINTERP_RHUMB_LINE ),
_maxPolyTilingAngle_deg( 45.0f ),
_optimizeVertexOrdering( false ),
_maximumCreaseAngle(Angle(0.0, Units::DEGREES))
{
    //nop
}

osg::Geode*
BuildGeometryFilter::processPolygons(FeatureList& features, FilterContext& context)
{
    osg::Geode* geode = new osg::Geode();

    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    //const SpatialReference* mapSRS = 0L;
    const SpatialReference* outputSRS = 0L;

    // set up the reference system info:
    if ( context.isGeoreferenced() )
    {
        featureSRS = context.extent()->getSRS();
        outputSRS  = context.getOutputSRS();
        makeECEF   = context.getOutputSRS()->isGeographic();
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

        if (input->getGeometry() == 0L)
            continue;

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
            osgGeom->setUseVertexBufferObjects(true);

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
                osgEarth::GeoExtent partExtent(featureSRS, part->getBounds());
                computeLocalizers(context, partExtent, w2l, l2w);
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
            tileAndBuildPolygon(part, featureSRS, outputSRS, makeECEF, true, osgGeom.get(), w2l);

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
                    //OE_TEST << "Running mesh subdivider with threshold " << *_maxAngle_deg << std::endl;

                    MeshSubdivider ms( _world2local, _local2world );
                    if ( input->geoInterp().isSet() )
                        ms.run( *osgGeom, threshold, *input->geoInterp() );
                    else
                        ms.run( *osgGeom, threshold, *_geoInterp );
                }

                // assign the primary color array. PER_VERTEX required in order to support
                // vertex optimization later
                unsigned count = osgGeom->getVertexArray()->getNumElements();
                osg::Vec4Array* colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
                colors->assign( count, primaryColor );
                osgGeom->setColorArray( colors );

                geode->addDrawable( osgGeom );

                // record the geometry's primitive set(s) in the index:
                if ( context.featureIndex() )
                    context.featureIndex()->tagDrawable( osgGeom.get(), input );

                // install clamping attributes if necessary
                if (_style.has<AltitudeSymbol>() &&
                    _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU)
                {
                    Clamping::applyDefaultClampingAttrs( osgGeom.get(), input->getDouble("__oe_verticalOffset", 0.0) );
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

namespace
{
    struct CopyHeightsCallback : public PolygonizeLinesOperator::Callback
    {
        osg::FloatArray* _heights;
        osg::ref_ptr<osg::FloatArray> _newHeights;
        CopyHeightsCallback(osg::FloatArray* heights) : _heights(heights) {
            if (_heights) {
                _newHeights = new osg::FloatArray();
                _newHeights->reserve(_heights->size() * 3);
            }
        }
        void operator()(unsigned i) {
            if (_newHeights.valid() && _heights) {
                _newHeights->push_back((*_heights)[i]);
            }
        }
    };
}

osg::Group*
BuildGeometryFilter::processPolygonizedLines(FeatureList&   features,
                                             bool           twosided,
                                             FilterContext& context,
                                             bool wireLines)
{
    osg::Group* group = new osg::Group;

    // establish some referencing
    bool                    makeECEF   = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* outputSRS  = 0L;

    if ( context.isGeoreferenced() )
    {
        featureSRS = context.extent()->getSRS();
        outputSRS = context.getOutputSRS();
        makeECEF = outputSRS->isGeographic();
    }

    // We need to create a different geode for each texture that is used so they can share statesets.
    typedef std::map< std::string, osg::ref_ptr< osg::Geode > > TextureToGeodeMap;
    TextureToGeodeMap geodes;

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

        std::string imageURI;

        // Image URI
        if (line->imageURI().isSet() && context.getSession() && context.getSession()->getResourceCache())
        {
            StringExpression temp( *line->imageURI() );
            imageURI = URI(input->eval(temp, context.getSession()), temp.uriContext()).full();
        }

        // Try to find the existing geode, otherwise create one.
        osg::ref_ptr< osg::Geode > geode;
        TextureToGeodeMap::iterator itr = geodes.find(imageURI);
        if (itr != geodes.end())
        {
            geode = itr->second;
        }
        else
        {
            geode = new osg::Geode;

            // Create the texture for the geode.
            if (imageURI.empty() == false)
            {
                osg::ref_ptr<osg::Texture> tex;
                if (context.getSession()->getResourceCache()->getOrCreateLineTexture(imageURI, tex, context.getDBOptions()))
                {
                    geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), 1);
                }
            }
            geodes[imageURI] = geode;
        }

        // run a symbol script if present.
        if ( line->script().isSet() )
        {
            StringExpression temp( line->script().get() );
            input->eval( temp, &context );
        }

        // The operator we'll use to make lines into polygons.
        osg::ref_ptr<OsgGeometryOperator> polygonizer;
        if (wireLines)
        {
            polygonizer = new WireLinesOperator(*line->stroke());
        }
        else
        {
            polygonizer = new PolygonizeLinesOperator(line);
        }
        //GPULinesOperator gpuLines(*line->stroke() );

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

            // GPU clamping enabled?
            bool gpuClamping =
                _style.has<AltitudeSymbol>() &&
                _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU;

            // collect all the pre-transformation HAT (Z) values.
            osg::ref_ptr<osg::FloatArray> hats = 0L;
            if (gpuClamping)
            {
                hats = new osg::FloatArray();
                hats->reserve( part->size() );
                for(Geometry::const_iterator i = part->begin(); i != part->end(); ++i )
                    hats->push_back( i->z() );
            }

            // transform the geometry into the target SRS and localize it about
            // a local reference point.
            osg::ref_ptr<osg::Vec3Array> verts   = new osg::Vec3Array();
            osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
            transformAndLocalize( part->asVector(), featureSRS, verts.get(), normals.get(), outputSRS, _world2local, makeECEF );

            // turn the lines into polygons.
            CopyHeightsCallback copyHeights(hats.get());
            osg::Geometry* geom = (*polygonizer)( verts.get(), normals.get(), gpuClamping? &copyHeights : 0L, twosided );
            //osg::Geometry* geom = gpuLines(verts.get());
            if ( geom )
            {
                geode->addDrawable( geom );
            }

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
                context.featureIndex()->tagDrawable( geom, input );

            // install clamping attributes if necessary
            if (gpuClamping)
            {
                Clamping::applyDefaultClampingAttrs( geom, input->getDouble("__oe_verticalOffset", 0.0) );
                Clamping::setHeights( geom, copyHeights._newHeights.get() );
                //OE_WARN << "heights = " << hats->size() << ", new hats = " << copyHeights._newHeights->size() << ", verts=" << geom->getVertexArray()->getNumElements() << std::endl;
            }
        }
        polygonizer->installShaders( geode.get() );
    }

    for (TextureToGeodeMap::iterator itr = geodes.begin(); itr != geodes.end(); ++itr)
    {
        // Optimize the Geode
        osg::Geode* geode = itr->second.get();
        osgUtil::Optimizer::MergeGeometryVisitor mg;
        mg.setTargetMaximumNumberOfVertices(Registry::instance()->getMaxNumberOfVertsPerDrawable());
        geode->accept(mg);

        if (_optimizeVertexOrdering == true)
        {
            osgUtil::Optimizer o;
            o.optimize( geode,
                osgUtil::Optimizer::INDEX_MESH
                | osgUtil::Optimizer::VERTEX_PRETRANSFORM
                | osgUtil::Optimizer::VERTEX_POSTTRANSFORM
                );
        }

        // Add it to the group
        group->addChild( geode );
    }

    return group;
}


osg::Group*
BuildGeometryFilter::processLines(FeatureList& features, FilterContext& context)
{
    // Group to contain all the lines we create here
    LineGroup* drawables = new LineGroup();

    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* outputSRS = 0L;

    // set up referencing information:
    if ( context.isGeoreferenced() )
    {
        featureSRS = context.extent()->getSRS();
        outputSRS  = context.getOutputSRS();
        makeECEF = outputSRS->isGeographic();
    }

    // Need to know if we are GPU clamping so we can add more attribs
    bool doGpuClamping =
        _style.has<AltitudeSymbol>() &&
        _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU;

    // For each input feature:
    for (FeatureList::iterator f = features.begin(); f != features.end(); ++f)
    {
        Feature* input = f->get();

        // extract the required line symbol; bail out if not found.
        const LineSymbol* line =
            input->style().isSet() && input->style()->has<LineSymbol>() ? input->style()->get<LineSymbol>() :
            _style.get<LineSymbol>();

        // if there's no line symbol, bail.
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
            bool isRing = (dynamic_cast<Ring*>(part) != 0L);

            // resolve the color:
            osg::Vec4f primaryColor = line->stroke()->color();

            // generate the geometry and localize to the local tangent plane
            osg::ref_ptr< osg::Vec3Array > allPoints = new osg::Vec3Array();
            transformAndLocalize( part->asVector(), featureSRS, allPoints.get(), outputSRS, _world2local, makeECEF );

            // construct a drawable for the lines
            LineDrawable* drawable = new LineDrawable(isRing? GL_LINE_LOOP : GL_LINE_STRIP);

            // if the user requested legacy rendering:
            if (line->useGLLines() == true)
                drawable->setUseGPU(false);

            drawable->importVertexArray(allPoints.get());

            if (line->stroke().isSet())
            {
                if (line->stroke()->width().isSet())
                    drawable->setLineWidth(line->stroke()->width().get());

                if (line->stroke()->stipplePattern().isSet())
                    drawable->setStipplePattern(line->stroke()->stipplePattern().get());

                if (line->stroke()->stippleFactor().isSet())
                    drawable->setStippleFactor(line->stroke()->stippleFactor().get());

                if (line->stroke()->smooth().isSet())
                    drawable->setLineSmooth(line->stroke()->smooth().get());
            }

            // For GPU clamping, we need an attribute array with Heights above Terrain in it.
            if (doGpuClamping)
            {
                osg::FloatArray* hats = new osg::FloatArray();
                hats->setBinding(osg::Array::BIND_PER_VERTEX);
                hats->setNormalize(false);
                drawable->setVertexAttribArray(Clamping::HeightsAttrLocation, hats);
                for (Geometry::const_iterator i = part->begin(); i != part->end(); ++i)
                {
                    drawable->pushVertexAttrib(hats, i->z());
                }
            }

            // assign the color:
            drawable->setColor(primaryColor);

            // embed the feature name if requested. Warning: blocks geometry merge optimization!
            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
                drawable->setName( name );
            }

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
            {
                context.featureIndex()->tagDrawable( drawable, input );
            }

            // install clamping attributes if necessary
            if (doGpuClamping)
            {
                Clamping::applyDefaultClampingAttrs( drawable, input->getDouble("__oe_verticalOffset", 0.0) );
            }

            // finalize the drawable and generate primitive sets
            drawable->dirty();

            drawables->addChild(drawable);
        }
    }

    // Finally, optimize the finished group for rendering.
    if (drawables)
    {
        drawables->optimize();
    }

    return drawables;
}


osg::Geode*
BuildGeometryFilter::processPoints(FeatureList& features, FilterContext& context)
{
    PointGroup* drawables = new PointGroup();

    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* outputSRS = 0L;

    // set up referencing information:
    if ( context.isGeoreferenced() )
    {
        featureSRS = context.extent()->getSRS();
        outputSRS  = context.getOutputSRS();
        makeECEF = outputSRS->isGeographic();
    }

    // Need to know if we are GPU clamping so we can add more attribs
    bool doGpuClamping =
        _style.has<AltitudeSymbol>() &&
        _style.get<AltitudeSymbol>()->technique() == AltitudeSymbol::TECHNIQUE_GPU;

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

            // build the geometry:
            osg::ref_ptr<osg::Vec3Array> allPoints = new osg::Vec3Array();
            transformAndLocalize( part->asVector(), featureSRS, allPoints.get(), outputSRS, _world2local, makeECEF );

            PointDrawable* drawable = new PointDrawable();

            drawable->importVertexArray(allPoints.get());

            if (point->size().isSet())
                drawable->setPointSize(point->size().get());

            if (point->smooth().isSet())
                drawable->setPointSmooth(point->smooth().get());

            // For GPU clamping, we need an attribute array with Heights above Terrain in it.
            if (doGpuClamping)
            {
                osg::FloatArray* hats = new osg::FloatArray();
                hats->setBinding(osg::Array::BIND_PER_VERTEX);
                hats->setNormalize(false);
                drawable->setVertexAttribArray(Clamping::HeightsAttrLocation, hats);
                for (Geometry::const_iterator i = part->begin(); i != part->end(); ++i)
                {
                    drawable->pushVertexAttrib(hats, i->z());
                }
            }

            // assign the color:
            drawable->setColor(primaryColor);

            // embed the feature name if requested. Warning: blocks geometry merge optimization!
            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
                drawable->setName( name );
            }

            // record the geometry's primitive set(s) in the index:
            if ( context.featureIndex() )
            {
                context.featureIndex()->tagDrawable( drawable, input );
            }

            // install clamping attributes if necessary
            if (doGpuClamping)
            {
                Clamping::applyDefaultClampingAttrs( drawable, input->getDouble("__oe_verticalOffset", 0.0) );
            }

            drawable->dirty();

            drawables->addChild(drawable);
        }
    }

    // Finally, optimize the finished group for rendering.
    if (drawables)
    {
        drawables->optimize();
    }

    return drawables;
}

// Borrowed from MeshConsolidator.cpp
namespace
{
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
    bool tesselateGeometry(osg::Geometry* geometry, bool useOSGTessellator)
    {
        if (useOSGTessellator) {
            osgUtil::Tessellator tess;
            tess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
            tess.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
            tess.retessellatePolygons(*geometry);
        }
        else {
            osgEarth::Tessellator oeTess;
            if (!oeTess.tessellateGeometry(*geometry))
            {
                osgUtil::Tessellator tess;
                tess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
                tess.setWindingType(osgUtil::Tessellator::TESS_WINDING_POSITIVE);
                tess.retessellatePolygons(*geometry);
            }
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
}

#ifdef USE_GNOMONIC_TESSELLATION

void
BuildGeometryFilter::tileAndBuildPolygon(
    Geometry*               input,
    const SpatialReference* inputSRS,
    const SpatialReference* outputSRS,
    bool                    makeECEF,
    bool                    tessellate,
    osg::Geometry*          osgGeom,
    const osg::Matrixd&     world2local)
{
    OE_SOFT_ASSERT_AND_RETURN(input != nullptr, __func__, );
    OE_SOFT_ASSERT_AND_RETURN(input->getType() != Geometry::TYPE_MULTI, __func__, );

    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array();
    verts->reserve(input->getTotalPointCount());

    // hard copy so we can project the values
    osg::ref_ptr<Geometry> proj = input->clone();

    Tessellator::Plane plane = Tessellator::PLANE_XY;

    if (outputSRS)
    {
        // for geographic data we need to project into 2D before tessellating:
        if (outputSRS->isGeographic())
        {
            osg::Vec3d temp;
            osg::BoundingBoxd ecef_bb;

            GeometryIterator xform_iter(proj.get(), true);
            while (xform_iter.hasMore())
            {
                Geometry* part = xform_iter.next();
                part->open();
                for (osg::Vec3d& p : *part)
                {
                    inputSRS->transform(p, outputSRS, temp);
                    outputSRS->transformToWorld(temp, p);
                    ecef_bb.expandBy(p);
                }
            }

            const osg::Vec3d& center = ecef_bb.center();

            GeometryIterator proj_iter(proj.get(), true);
            while (proj_iter.hasMore())
            {
                Geometry* part = proj_iter.next();
                for (osg::Vec3d& p : *part)
                {
                    ecef_to_gnomonic(p, center, outputSRS->getEllipsoid());
                }
            }
        }

        else
        {
            GeometryIterator xform_iter(proj.get(), true);
            while (xform_iter.hasMore())
            {
                Geometry* part = xform_iter.next();
                part->open();
                inputSRS->transform(part->asVector(), outputSRS);
            }
        }
    }
    else
    {
        // with no SRS, we need to automatically figure out what 
        // is the closest plane for tessellation
        plane = Tessellator::PLANE_AUTO;
    }

    // tessellate
    Tessellator tess;

    std::vector<uint32_t> indices;
    if (tess.tessellate2D(proj.get(), indices, plane) == false)
        return;

    if (indices.empty())
        return;

    int offset = verts->size();

    osg::Vec3d temp, vert;

    if (outputSRS && outputSRS->isGeographic())
    {
        ConstGeometryIterator verts_iter(input, true);
        while (verts_iter.hasMore())
        {
            const Geometry* part = verts_iter.next();
            for (const auto& p : *part)
            {
                inputSRS->transform(p, outputSRS, temp);
                outputSRS->transformToWorld(temp, vert);
                vert = vert * world2local;
                verts->push_back(vert);
            }
        }
    }
    else
    {
        ConstGeometryIterator verts_iter(proj.get(), true);
        while (verts_iter.hasMore())
        {
            const Geometry* part = verts_iter.next();
            for (const auto& p : *part)
            {
                verts->push_back(p * world2local);
            }
        }
    }

    osg::DrawElements* de = new osg::DrawElementsUInt(
        GL_TRIANGLES,
        indices.size(),
        &indices[0]);

    osgGeom->setVertexArray(verts.get());
    osgGeom->addPrimitiveSet(de);
}

#else

void
BuildGeometryFilter::tileAndBuildPolygon(Geometry*               ring,
                                         const SpatialReference* featureSRS,
                                         const SpatialReference* outputSRS,
                                         bool                    makeECEF,
                                         bool                    tessellate,
                                         osg::Geometry*          osgGeom,
                                         const osg::Matrixd      &world2local)
{
    if (ring==NULL)
        return;

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
            cellCenter.transform(outputSRS, cellCenter);
            osg::Matrix world2cell;
            cellCenter.createWorldToLocal( world2cell );

            // build the localized polygon:
            buildPolygon(geom, featureSRS, outputSRS, makeECEF, temp.get(), world2cell);

            // if successful, transform the verts back into our master LTP:
            if ( temp->getNumPrimitiveSets() > 0 )
            {
                // Tesselate the polygon while the coordinates are still in the LTP
                if (tesselateGeometry( temp.get(), useOSGTessellator().value() ))
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
    mgv.setTargetMaximumNumberOfVertices(Registry::instance()->getMaxNumberOfVertsPerDrawable());
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
}
#endif

// builds and tessellates a polygon (with or without holes)
void
BuildGeometryFilter::buildPolygon(Geometry*               ring,
                                  const SpatialReference* featureSRS,
                                  const SpatialReference* outputSRS,
                                  bool                    makeECEF,
                                  osg::Geometry*          osgGeom,
                                  const osg::Matrixd      &world2local)
{
    if ( !ring->isValid() )
        return;

    ring->rewind(osgEarth::Geometry::ORIENTATION_CCW);

    osg::ref_ptr<osg::Vec3Array> allPoints = new osg::Vec3Array();
    transformAndLocalize( ring->asVector(), featureSRS, allPoints.get(), outputSRS, world2local, makeECEF );

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
                hole->rewind(osgEarth::Geometry::ORIENTATION_CW);

                osg::ref_ptr<osg::Vec3Array> holePoints = new osg::Vec3Array();
                transformAndLocalize( hole->asVector(), featureSRS, holePoints.get(), outputSRS, world2local, makeECEF );

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
}


namespace
{
    struct GenerateNormalFunctor
    {
        osg::Vec3Array* _verts;
        osg::Vec3Array* _normals;

        GenerateNormalFunctor() : _verts(0L), _normals(0L) { }

        void set(osg::Vec3Array *cb, osg::Vec3Array *nb)
        {
            _verts = cb;
            _normals = nb;
        }

        inline void operator()(unsigned i1, unsigned i2, unsigned i3)
        {
            const osg::Vec3& v1 = (*_verts)[i1];
            const osg::Vec3& v2 = (*_verts)[i2];
            const osg::Vec3& v3 = (*_verts)[i3];

            // calc orientation of triangle.
            osg::Vec3 normal = (v2 - v1) ^ (v3 - v1);
            normal.normalize();

            (*_normals)[i1] += normal;
            (*_normals)[i2] += normal;
            (*_normals)[i3] += normal;
        }

        void finish()
        {
            for (unsigned i = 0; i < _normals->size(); ++i)
            {
                (*_normals)[i].normalize();
            }
        }
    };

    struct GenerateNormals : public osg::NodeVisitor
    {
        GenerateNormals() : osg::NodeVisitor()
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }

        inline void apply(osg::Drawable& drawable)
        {
            osg::Geometry* geom = drawable.asGeometry();
            if (geom)
            {
                osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());

                osg::Vec3Array* normals = new osg::Vec3Array(verts->size());
                normals->setBinding(normals->BIND_PER_VERTEX);

                osg::TriangleIndexFunctor<GenerateNormalFunctor> f;
                f.set(verts, normals);
                geom->accept(f);
                f.finish();

                geom->setNormalArray(normals);
            }
            traverse(drawable);
        }
    };
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
    FeatureList wireLines;

    FeatureList splitFeatures;

    // Split features across the dateline if necessary
    if (context.getOutputSRS() && !context.getOutputSRS()->isGeographic())
    {
        for(FeatureList::iterator itr = input.begin(); itr != input.end(); ++itr)
        {
            Feature* f = itr->get();
            FeatureList tmpSplit;
            f->splitAcrossDateLine(tmpSplit);
            splitFeatures.insert(splitFeatures.end(), tmpSplit.begin(), tmpSplit.end());
        }
    }
    else
    {
        // Just copy the input list to the split list
        std::copy(input.begin(), input.end(), std::back_inserter(splitFeatures));
    }

    for(FeatureList::iterator i = splitFeatures.begin(); i != splitFeatures.end(); ++i)
    {
        Feature* f = i->get();

        // first consider the overall style:
        bool has_polysymbol     = poly != 0L;
        bool has_linesymbol     = line != 0L && line->stroke()->widthUnits() == Units::PIXELS;
        bool has_polylinesymbol = line != 0L && line->stroke()->widthUnits() != Units::PIXELS;
        bool has_pointsymbol    = point != 0L;
        bool has_wirelinessymbol = line && line->useWireLines().value();

        // if the featue has a style set, that overrides:
        if ( f->style().isSet() )
        {
            has_polysymbol     = has_polysymbol     || (f->style()->has<PolygonSymbol>());
            has_linesymbol     = has_linesymbol     || (f->style()->has<LineSymbol>() && f->style()->get<LineSymbol>()->stroke()->widthUnits() == Units::PIXELS);
            has_polylinesymbol = has_polylinesymbol || (f->style()->has<LineSymbol>() && f->style()->get<LineSymbol>()->stroke()->widthUnits() != Units::PIXELS);
            has_pointsymbol    = has_pointsymbol    || (f->style()->has<PointSymbol>());
            has_wirelinessymbol = has_wirelinessymbol
                || (f->style()->has<LineSymbol>() && f->style()->get<LineSymbol>()->useWireLines().value());
        }

        // if there's a polygon with outlining disabled, nix the line symbol.
        if (has_polysymbol)
        {
            if (poly && poly->outline() == false)
                has_linesymbol = false;
            else if (f->style().isSet() && f->style()->has<PolygonSymbol>() && f->style()->get<PolygonSymbol>()->outline() == false)
                has_linesymbol = false;
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

            case Geometry::TYPE_POINT:
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
        {
#if 0 // Placeholder. We probably need this, but let's wait and see
            // split polygons that cross the andimeridian:
            if (f->getSRS()->isGeographic() &&
                f->calculateExtent().crossesAntimeridian())
            {
                FeatureList temp;
                f->splitAcrossDateLine(temp);
                std::copy(temp.begin(), temp.end(), std::back_inserter(polygons));
            }
            else
#endif
            {
                polygons.push_back( f );
            }
        }

        if ( has_linesymbol )
            lines.push_back( f );

        if ( has_wirelinessymbol)
        {
            wireLines.push_back( f);
        }
        else if ( has_polylinesymbol )
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
            mg.setTargetMaximumNumberOfVertices(Registry::instance()->getMaxNumberOfVertsPerDrawable());
            geode->accept(mg);

            if (_optimizeVertexOrdering == true)
            {
                osg::Timer_t t = osg::Timer::instance()->tick();
                osgUtil::Optimizer o;
                o.optimize( geode.get(),
                    osgUtil::Optimizer::INDEX_MESH |
                    osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                    osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
                OE_INFO << "Vertex ordering optimization took " << osg::Timer::instance()->delta_s(t, osg::Timer::instance()->tick()) << std::endl;
            }

            // Generate normals. CANNOT use OSG's SmoothingVisitor because it adds verts
            // but ignores other vertex attribute arrays.
            GenerateNormals gen;
            geode->accept(gen);

            result->addChild( geode.get() );
        }
    }

    if ( polygonizedLines.size() > 0 )
    {
        OE_TEST << LC << "Building " << polygonizedLines.size() << " polygonized lines." << std::endl;
        bool twosided = polygons.size() > 0 ? false : true;
        osg::ref_ptr< osg::Group > lines = processPolygonizedLines(polygonizedLines, twosided, context,
                                                                   false);

        if (lines->getNumChildren() > 0)
        {
            result->addChild( lines.get() );
        }

    }

    if ( wireLines.size() > 0 )
    {
        OE_TEST << LC << "Building " << wireLines.size() << " wire lines." << std::endl;
        osg::ref_ptr< osg::Group > lines = processPolygonizedLines(wireLines, true, context, true);

        if (lines->getNumChildren() > 0)
        {
            result->addChild( lines.get() );
        }

    }

    if ( lines.size() > 0 )
    {
        OE_TEST << LC << "Building " << lines.size() << " lines." << std::endl;

        osg::ref_ptr<osg::Group> group = processLines(lines, context);

        if ( group->getNumChildren() > 0 )
        {
            result->addChild(group.get());
        }
    }

    if ( points.size() > 0 )
    {
        OE_TEST << LC << "Building " << points.size() << " points." << std::endl;

        osg::ref_ptr<osg::Group> group = processPoints(points, context);

        if ( group->getNumChildren() > 0 )
        {
            result->addChild(group.get());
        }
    }

    //// indicate that geometry contains clamping attributes
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
