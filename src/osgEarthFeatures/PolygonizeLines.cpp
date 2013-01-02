/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthFeatures/PolygonizeLines>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgUtil/Optimizer>

#define LC "[PolygonizeLines] "

using namespace osgEarth::Features;

#define OV(p) "("<<p.x()<<","<<p.y()<<")"

namespace
{
    typedef std::pair<osg::Vec3,osg::Vec3> Segment;

    // Given two rays (point + direction vector), find the intersection
    // of those rays in 2D space and put the result in [out]. Return true
    // if they intersect, false if they do not.
    bool interesctRays(const osg::Vec3& p0, const osg::Vec3& pd, // point, dir
                       const osg::Vec3& q0, const osg::Vec3& qd, // point, dir
                       osg::Vec3& out)
    {
        const float epsilon = 0.001f;

        float det = pd.y()*qd.x()-pd.x()*qd.y();
        if ( osg::equivalent(det, 0.0f, epsilon) )
            return false;

        float u = (qd.x()*(q0.y()-p0.y())+qd.y()*(p0.x()-q0.x()))/det;
        if ( u < epsilon ) //0.0f )
            return false;

        float v = (pd.x()*(q0.y()-p0.y())+pd.y()*(p0.x()-q0.x()))/det;
        if ( v < epsilon ) //0.0f )
            return false;

        out = p0 + pd*u;
        return true;
    }

    // Rotate the directional vector [in] counter-clockwise by [angle] radians
    // and return the result in [out].
    inline void rotate(const osg::Vec3& in, float angle, osg::Vec3& out)
    {
        float ca = cosf(angle), sa = sinf(angle);
        out.x() = in.x()*ca - in.y()*sa;
        out.y() = in.x()*sa + in.y()*ca;
        out.z() = in.z();
    }

    // Add two triangles to an EBO vector; [side] controls the winding
    // direction.
    inline void addTris(std::vector<unsigned>& ebo, unsigned i, unsigned prev_i, unsigned current, int side)
    {
        if ( side == 0 )
        {
            ebo.push_back( i-1 );
            ebo.push_back( i );
            ebo.push_back( prev_i );
            ebo.push_back( prev_i );
            ebo.push_back( i );
            ebo.push_back( current );
        }
        else
        {
            ebo.push_back( i-1 );
            ebo.push_back( prev_i );
            ebo.push_back( i );
            ebo.push_back( prev_i );
            ebo.push_back( current );
            ebo.push_back( i );
        }
    }

    // Add a triangle to an EBO vector; [side] control the winding
    // direction.
    inline void addTri(std::vector<unsigned>& ebo, unsigned i0, unsigned i1, unsigned i2, int side)
    {
        ebo.push_back( i0 );
        ebo.push_back( side == 0 ? i1 : i2 );
        ebo.push_back( side == 0 ? i2 : i1 );
    }
}


PolygonizeLinesOperator::PolygonizeLinesOperator(const Stroke& stroke) :
_stroke( stroke )
{
    //nop
}


osg::Geometry*
PolygonizeLinesOperator::operator()(osg::Vec3Array* verts) const
{
    // number of verts on the original line.
    unsigned lineSize = verts->size();

    // cannot generate a line with less than 2 verts.
    if ( lineSize < 2 )
        return 0L;

    float width            = *_stroke.width();
    float halfWidth        = 0.5f * width;
    float maxRoundingAngle = asin( _stroke.roundingRatio().get() );

    osg::Geometry*  geom  = new osg::Geometry();

    // Add the input verts to the geometry. This forms the "spine" of the
    // polygonized line. We need the spine so we can affect proper clamping,
    // texturing and vertex attribution.
    geom->setVertexArray( verts );

    // triangulate the points into a mesh.
    std::vector<unsigned> ebo;

    // buffer the left side:
    unsigned  i;
    osg::Vec3 prevBufVert;
    osg::Vec3 prevBufVec;
    unsigned  prevBufVertPtr;
    unsigned  eboPtr = 0;
    osg::Vec3 prevDir;

    // iterate over both "sides" of the center line:
    for( int s=0; s<=1; ++s )
    {
        // s==0 is the left side, s==1 is the right side.
        float side = s == 0 ? -1.0f : 1.0f;

        // iterate over each line segment.
        for( i=0; i<lineSize-1; ++i )
        {
            Segment   seg   ( (*verts)[i], (*verts)[i+1] );             // current segment.
            osg::Vec3 dir = seg.second - seg.first;
            dir.normalize();                                            // directional vector of segment
            osg::Vec3 bufVec( (side)*dir.y(), (-side)*dir.x(), 0.0f );  // buffering vector (orthogonal to dir)
            bufVec *= halfWidth;                                        // buffering size.

            osg::Vec3 bufVert = (*verts)[i] + bufVec;                   // starting buffered vert.

            if ( i == 0 )
            {
                // first vert-- no corner to check, just make the buffered vert.
                verts->push_back( bufVert );
                prevBufVert = bufVert;
                prevBufVertPtr = verts->size() - 1;

                // render the front end-cap.
                if ( _stroke.lineCap() == Stroke::LINECAP_ROUND )
                {
                    float angle = osg::PI_2;
                    int steps = (int)ceil(angle/maxRoundingAngle);
                    float step = angle/(float)steps;
                    osg::Vec3 circlevec = verts->back() - (*verts)[i];

                    for( int j=1; j<=steps; ++j )
                    {
                        osg::Vec3 v;
                        float a = step * (float)j;
                        rotate( circlevec, -(side)*a, v );
                        verts->push_back( (*verts)[i] + v );
                        addTri( ebo, i, verts->size()-2, verts->size()-1, s );
                    }
                }
                else if ( _stroke.lineCap() == Stroke::LINECAP_SQUARE )
                {
                    float cornerWidth = sqrt(2.0*halfWidth*halfWidth);
                    verts->push_back( verts->back() - dir*halfWidth );
                    addTri( ebo, i, verts->size()-2, verts->size()-1, s );
                    verts->push_back( (*verts)[i] - dir*halfWidth );
                    addTri( ebo, i, verts->size()-2, verts->size()-1, s );
                }
            }
            else
            {
                // does the new segment turn create a reflex angle (>180deg)?
                float z = (prevDir ^ dir).z();
                bool isOutside = s == 0 ? z <= 0.0 : z >= 0.0;
                bool isInside = !isOutside;

                // if this is an inside angle (or we're using mitered corners)
                // calculate the corner point by finding the convergance of the two
                // vectors enimating from the previous and next buffered points.
                if ( isInside || _stroke.lineJoin() == Stroke::LINEJOIN_MITRE )
                {
                    // unit vector from the previous buffered vert to the current one
                    osg::Vec3 vec1 = (*verts)[i] - (*verts)[i-1];
                    vec1.normalize();

                    // unit vector from the current buffered vert to the next one.
                    osg::Vec3 nextBufVert = seg.second + bufVec;
                    osg::Vec3 vec2        = (*verts)[i] - (*verts)[i+1];
                    vec2.normalize();

                    // find the intersection of these two vectors. Check for the 
                    // special case of colinearity.
                    osg::Vec3 isect;
                    if ( interesctRays(prevBufVert, vec1, nextBufVert, vec2, isect) )
                        verts->push_back(isect);
                    else
                        verts->push_back(bufVert);

                    // now that we have the current buffered point, build triangles
                    // for *previous* segment.
                    addTris( ebo, i, prevBufVertPtr, verts->size()-1, s );
                }

                else if ( _stroke.lineJoin() == Stroke::LINEJOIN_ROUND )
                {
                    // for a rounded corner, first create the first rim point:
                    osg::Vec3 start = (*verts)[i] + prevBufVec;
                    verts->push_back( start );
                    addTris( ebo, i, prevBufVertPtr, verts->size()-1, s );

                    // insert the edge-rounding points:
                    float angle = acosf( (prevBufVec * bufVec)/(halfWidth*halfWidth) );
                    int steps = (int)ceil(angle/maxRoundingAngle);
                    float step = angle/(float)steps;
                    osg::Vec3 circlevec = start - (*verts)[i];
                    for( int j=1; j<=steps; ++j )
                    {
                        osg::Vec3 v;
                        float a = step * (float)j;
                        rotate( circlevec, side*a, v );
                        verts->push_back( (*verts)[i] + v );
                        addTri( ebo, i, verts->size()-1, verts->size()-2, s );
                    }
                }

                // record these for the next segment.
                prevBufVert    = verts->back();
                prevBufVertPtr = verts->size() - 1;
            }

            // record these for the next segment.
            prevDir    = dir;
            prevBufVec = bufVec;
        }

        // record the final point.
        verts->push_back( (*verts)[i] + prevBufVec );

        // build triangles for the final segment.
        addTris( ebo, i, prevBufVertPtr, verts->size()-1, s );

        // build the final end cap.
        if ( _stroke.lineCap() == Stroke::LINECAP_ROUND )
        {
            float angle = osg::PI_2;
            int steps = (int)ceil(angle/maxRoundingAngle);
            float step = angle/(float)steps;
            osg::Vec3 circlevec = verts->back() - (*verts)[i];

            for( int j=1; j<=steps; ++j )
            {
                osg::Vec3 v;
                float a = step * (float)j;
                rotate( circlevec, (side)*a, v );
                verts->push_back( (*verts)[i] + v );
                addTri( ebo, i, verts->size()-1, verts->size()-2, s );
            }
        }
        else if ( _stroke.lineCap() == Stroke::LINECAP_SQUARE )
        {
            float cornerWidth = sqrt(2.0*halfWidth*halfWidth);
            verts->push_back( verts->back() + prevDir*halfWidth );
            addTri( ebo, i, verts->size()-1, verts->size()-2, s );
            verts->push_back( (*verts)[i] + prevDir*halfWidth );
            addTri( ebo, i, verts->size()-1, verts->size()-2, s );
        }
    }

    // copy the ebo into a primitive set of appropriate size:
    osg::DrawElements* primset =
        verts->size() > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt  ( GL_TRIANGLES ) :
        verts->size() > 0xFF   ? (osg::DrawElements*)new osg::DrawElementsUShort( GL_TRIANGLES ) :
                                 (osg::DrawElements*)new osg::DrawElementsUByte ( GL_TRIANGLES );

    // todo: copy from ebo
    primset->reserveElements( ebo.size() );
    for(i=0; i<ebo.size(); ++i )
        primset->addElement( ebo[i] );

    geom->addPrimitiveSet( primset );

    return geom;
}


//------------------------------------------------------------------------


PolygonizeLinesFilter::PolygonizeLinesFilter(const Style& style) :
_style( style )
{
    //nop
}


osg::Node*
PolygonizeLinesFilter::push(FeatureList& input, FilterContext& cx)
{
    // compute the coordinate localization matrices.
    computeLocalizers( cx );

    // establish some things
    bool                    makeECEF   = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS     = 0L;

    if ( cx.isGeoreferenced() )
    {
        makeECEF   = cx.getSession()->getMapInfo().isGeocentric();
        featureSRS = cx.extent()->getSRS();
        mapSRS     = cx.getSession()->getMapInfo().getProfile()->getSRS();
    }

    // The operator we'll use to make lines into polygons.
    const LineSymbol* line = _style.get<LineSymbol>();
    PolygonizeLinesOperator polygonize( line ? (*line->stroke()) : Stroke() );

    // Geode to hold all the geometries.
    osg::Geode* geode = new osg::Geode();

    // iterate over all features.
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
    {
        Feature* f = i->get();

        // iterate over all the feature's geometry parts. We will treat
        // them as lines strings.
        GeometryIterator parts( f->getGeometry(), false );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip empty geometry
            if ( part->size() == 0 )
                continue;

            // transform the geometry into the target SRS and localize it about 
            // a local reference point.
            osg::Vec3Array* verts = new osg::Vec3Array();
            transformAndLocalize( part->asVector(), featureSRS, verts, mapSRS, _world2local, makeECEF );

            // turn the lines into polygons.
            osg::Geometry* geom = polygonize( verts );
            geode->addDrawable( geom );
        }
    }

    // attempt to combine geometries for better performance
    MeshConsolidator::run( *geode );

    // GPU performance optimization:
#if 0 // issue: ignores vertex attributes
    osgUtil::Optimizer optimizer;
    optimizer.optimize(
        result,
        osgUtil::Optimizer::VERTEX_PRETRANSFORM |
        osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
#endif

    return delocalize( geode );
}
