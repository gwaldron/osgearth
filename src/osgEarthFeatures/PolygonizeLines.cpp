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


using namespace osgEarth::Features;

#define OV(p) "("<<p.x()<<","<<p.y()<<")"

namespace
{
    typedef std::pair<osg::Vec3,osg::Vec3> Segment;

    bool interesctRays(const osg::Vec3& p0, const osg::Vec3& pd,
                       const osg::Vec3& q0, const osg::Vec3& qd,
                       osg::Vec3& out)
    {
        //OE_NOTICE << "isect: p0=" << OV(p0) << ", pd=" << OV(pd)
        //    << ", q0=" << OV(q0) << ", qd=" << OV(qd) << std::endl;

        float det = pd.y()*qd.x()-pd.x()*qd.y();
        if ( osg::equivalent(det, 0.0f) )
            return false;

        float u = (qd.x()*(q0.y()-p0.y())+qd.y()*(p0.x()-q0.x()))/det;
        if ( u < 0.0f )
            return false;

        float v = (pd.x()*(q0.y()-p0.y())+pd.y()*(p0.x()-q0.x()))/det;
        if ( v < 0.0f )
            return false;

        out = p0 + pd*u;
        return true;
    }

    inline void rotate(const osg::Vec3& in, float angle, osg::Vec3& out)
    {
        float ca = cosf(angle), sa = sinf(angle);
        out.x() = in.x()*ca - in.y()*sa;
        out.y() = in.x()*sa + in.y()*ca;
        out.z() = 0.0f;
    }

    inline void addTris(std::vector<unsigned>& ebo, unsigned i, unsigned prev_i, unsigned current, int side)
    {
        // build triangles for *previous* segment
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
PolygonizeLinesOperator::operator()(const std::vector<osg::Vec3d>& input,
                                    const osg::Matrix&             world2local) const
{
    // number of verts on the original line.
    unsigned lineSize = input.size();

    // cannot generate a line with less than 2 verts.
    if ( lineSize < 2 )
        return 0L;

    float width            = *_stroke.width();
    float halfWidth        = 0.5f * width;
    float maxRoundingAngle = asin( _stroke.roundingRatio().get() );

    osg::Geometry*  geom  = new osg::Geometry();
    osg::Vec3Array* verts = new osg::Vec3Array();
    geom->setVertexArray( verts );

    // first, convert the input to local coords and store them.
    for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        verts->push_back( (*i) * world2local );
    }

    // triangulate the points into a mesh.
    std::vector<unsigned> ebo;

    // buffer the left side:
    unsigned  i;
    osg::Vec3 prevBufVert;
    osg::Vec3 prevBufVec;
    unsigned  prevBufVertPtr;
    unsigned  eboPtr = 0;
    osg::Vec3 prevDir;

    for( int s=0; s<=1; ++s )
    {
        float side = s == 0 ? -1.0f : 1.0f;

        for( i=0; i<lineSize-1; ++i )
        {
            Segment   seg   ( (*verts)[i], (*verts)[i+1] );
            osg::Vec3 dir = seg.second - seg.first;
            dir.normalize();
            osg::Vec3 bufVec( (side)*dir.y(), (-side)*dir.x(), 0.0f );
            bufVec *= halfWidth;

            osg::Vec3 bufVert = (*verts)[i] + bufVec;

            if ( i == 0 )
            {
                // first vert? no corner to check, just make the buffered vert.
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
                bool isOutside = s == 0 ? (prevDir ^ dir).z() <= 0.0 : (prevDir ^ dir).z() >= 0.0;
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

