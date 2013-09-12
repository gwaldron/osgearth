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
#include <osgEarth/DPLineSegmentIntersector>
#include <osg/KdTree>
#include <osg/TriangleFunctor>

using namespace osgEarth;

namespace
{
    struct TriangleIntersection
    {
        TriangleIntersection(unsigned int index, const osg::Vec3d& normal, double r1, const osg::Vec3* v1, double r2, const osg::Vec3* v2, double r3, const osg::Vec3* v3):
            _index(index),
            _normal(normal),
            _r1(r1),
            _v1(v1),
            _r2(r2),
            _v2(v2),
            _r3(r3),
            _v3(v3) {}

        unsigned int         _index;
        const osg::Vec3      _normal;
        double               _r1;
        const osg::Vec3*     _v1;
        double               _r2;
        const osg::Vec3*     _v2;
        double               _r3;
        const osg::Vec3*     _v3;

    protected:

        TriangleIntersection& operator = (const TriangleIntersection&) { return *this; }
    };


    typedef std::multimap<double,TriangleIntersection> TriangleIntersections;


    struct DoublePrecisionTriangleIntersector
    {
        osg::Vec3d   _s;
        osg::Vec3d   _d;
        double       _length;

        int         _index;
        double       _ratio;
        bool        _hit;
        bool        _limitOneIntersection;

        TriangleIntersections _intersections;

        DoublePrecisionTriangleIntersector()
        {
            _length = 0.0;
            _index = 0;
            _ratio = 0.0;
            _hit = false;
            _limitOneIntersection = false;
        }

        void set(const osg::Vec3d& start, osg::Vec3d& end, double ratio=FLT_MAX)
        {
            _hit=false;
            _index = 0;
            _ratio = ratio;

            _s = start;
            _d = end - start;
            _length = _d.length();
            _d /= _length;
        }

        inline void operator () (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool treatVertexDataAsTemporary)
        {
            ++_index;

            if (_limitOneIntersection && _hit) return;

            if (v1==v2 || v2==v3 || v1==v3) return;

            osg::Vec3d v1d(v1), v2d(v2), v3d(v3);

            osg::Vec3d v12 = v2d-v1d;
            osg::Vec3d n12 = v12^_d;
            double ds12 = (_s-v1d)*n12;
            double d312 = (v3d-v1d)*n12;
            if (d312>=0.0)
            {
                if (ds12<0.0) return;
                if (ds12>d312) return;
            }
            else                     // d312 < 0
            {
                if (ds12>0.0) return;
                if (ds12<d312) return;
            }

            osg::Vec3d v23 = v3d-v2d;
            osg::Vec3d n23 = v23^_d;
            double ds23 = (_s-v2d)*n23;
            double d123 = (v1d-v2d)*n23;
            if (d123>=0.0)
            {
                if (ds23<0.0) return;
                if (ds23>d123) return;
            }
            else                     // d123 < 0
            {
                if (ds23>0.0) return;
                if (ds23<d123) return;
            }

            osg::Vec3d v31 = v1d-v3d;
            osg::Vec3d n31 = v31^_d;
            double ds31 = (_s-v3d)*n31;
            double d231 = (v2d-v3d)*n31;
            if (d231>=0.0)
            {
                if (ds31<0.0) return;
                if (ds31>d231) return;
            }
            else                     // d231 < 0
            {
                if (ds31>0.0) return;
                if (ds31<d231) return;
            }


            double r3;
            if (osg::equivalent(ds12,0.0)) r3=0.0;
            else if (!osg::equivalent(d312,0.0)) r3 = ds12/d312;
            else return; // the triangle and the line must be parallel intersection.

            double r1;
            if (osg::equivalent(ds23,0.0)) r1=0.0;
            else if (!osg::equivalent(d123,0.0)) r1 = ds23/d123;
            else return; // the triangle and the line must be parallel intersection.

            double r2;
            if (osg::equivalent(ds31,0.0)) r2=0.0;
            else if (!osg::equivalent(d231,0.0)) r2 = ds31/d231;
            else return; // the triangle and the line must be parallel intersection.

            double total_r = (r1+r2+r3);
            if (!osg::equivalent(total_r,1.0))
            {
                if (osg::equivalent(total_r,0.0)) return; // the triangle and the line must be parallel intersection.
                double inv_total_r = 1.0/total_r;
                r1 *= inv_total_r;
                r2 *= inv_total_r;
                r3 *= inv_total_r;
            }

            osg::Vec3d in = v1d*r1+v2d*r2+v3d*r3;
            if (!in.valid())
            {
                //OSG_WARN<<"Warning:: Picked up error in TriangleIntersect"<<std::endl;
                //OSG_WARN<<"   ("<<v1<<",\t"<<v2<<",\t"<<v3<<")"<<std::endl;
                //OSG_WARN<<"   ("<<r1<<",\t"<<r2<<",\t"<<r3<<")"<<std::endl;
                return;
            }

            float d = (in-_s)*_d;

            if (d<0.0) return;
            if (d>_length) return;

            osg::Vec3d normal = v12^v23;
            normal.normalize();

            float r = d/_length;


            if (treatVertexDataAsTemporary)
            {
                _intersections.insert(std::pair<const double,TriangleIntersection>(r,TriangleIntersection(_index-1,normal,r1,0,r2,0,r3,0)));
            }
            else
            {
                _intersections.insert(std::pair<const double,TriangleIntersection>(r,TriangleIntersection(_index-1,normal,r1,&v1,r2,&v2,r3,&v3)));
            }
            _hit = true;

        }

    };
}

//----------------------------------------------------------------------------    

osgUtil::Intersector* 
DPLineSegmentIntersector::clone(osgUtil::IntersectionVisitor& iv)
{
    if (_coordinateFrame==MODEL && iv.getModelMatrix()==0)
    {
        //GW: changed the next line
        osg::ref_ptr<DPLineSegmentIntersector> lsi = new DPLineSegmentIntersector(_start, _end);
        lsi->_parent = this;
        lsi->_intersectionLimit = this->_intersectionLimit;
        return lsi.release();
    }

    // compute the matrix that takes this Intersector from its CoordinateFrame into the local MODEL coordinate frame
    // that geometry in the scene graph will always be in.
    osg::Matrix matrix;
    switch (_coordinateFrame)
    {
    case(WINDOW):
        if (iv.getWindowMatrix()) matrix.preMult( *iv.getWindowMatrix() );
        if (iv.getProjectionMatrix()) matrix.preMult( *iv.getProjectionMatrix() );
        if (iv.getViewMatrix()) matrix.preMult( *iv.getViewMatrix() );
        if (iv.getModelMatrix()) matrix.preMult( *iv.getModelMatrix() );
        break;
    case(PROJECTION):
        if (iv.getProjectionMatrix()) matrix.preMult( *iv.getProjectionMatrix() );
        if (iv.getViewMatrix()) matrix.preMult( *iv.getViewMatrix() );
        if (iv.getModelMatrix()) matrix.preMult( *iv.getModelMatrix() );
        break;
    case(VIEW):
        if (iv.getViewMatrix()) matrix.preMult( *iv.getViewMatrix() );
        if (iv.getModelMatrix()) matrix.preMult( *iv.getModelMatrix() );
        break;
    case(MODEL):
        if (iv.getModelMatrix()) matrix = *iv.getModelMatrix();
        break;
    }

    osg::Matrix inverse;
    inverse.invert(matrix);

    //GW: changed the next line
    osg::ref_ptr<DPLineSegmentIntersector> lsi = new DPLineSegmentIntersector(_start * inverse, _end * inverse);
    lsi->_parent = this;
    lsi->_intersectionLimit = this->_intersectionLimit;
    return lsi.release();
}
        
void
DPLineSegmentIntersector::intersect(osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable)
{
    if (reachedLimit()) return;

    osg::Vec3d s(_start), e(_end);
    if ( !intersectAndClip( s, e, drawable->getBound() ) ) return;

    if (iv.getDoDummyTraversal()) return;

    osg::KdTree* kdTree = iv.getUseKdTreeWhenAvailable() ? dynamic_cast<osg::KdTree*>(drawable->getShape()) : 0;
    if (kdTree)
    {
        osg::KdTree::LineSegmentIntersections intersections;
        intersections.reserve(4);
        if (kdTree->intersect(s,e,intersections))
        {
            // OSG_NOTICE<<"Got KdTree intersections"<<std::endl;
            for(osg::KdTree::LineSegmentIntersections::iterator itr = intersections.begin();
                itr != intersections.end();
                ++itr)
            {
                osg::KdTree::LineSegmentIntersection& lsi = *(itr);

                // get ratio in s,e range
                double ratio = lsi.ratio;

                // remap ratio into _start, _end range
                double remap_ratio = ((s-_start).length() + ratio * (e-s).length() )/(_end-_start).length();


                Intersection hit;
                hit.ratio = remap_ratio;
                hit.matrix = iv.getModelMatrix();
                hit.nodePath = iv.getNodePath();
                hit.drawable = drawable;
                hit.primitiveIndex = lsi.primitiveIndex;

                hit.localIntersectionPoint = _start*(1.0-remap_ratio) + _end*remap_ratio;

                // OSG_NOTICE<<"KdTree: ratio="<<hit.ratio<<" ("<<hit.localIntersectionPoint<<")"<<std::endl;

                hit.localIntersectionNormal = lsi.intersectionNormal;

                hit.indexList.reserve(3);
                hit.ratioList.reserve(3);
                if (lsi.r0!=0.0f)
                {
                    hit.indexList.push_back(lsi.p0);
                    hit.ratioList.push_back(lsi.r0);
                }

                if (lsi.r1!=0.0f)
                {
                    hit.indexList.push_back(lsi.p1);
                    hit.ratioList.push_back(lsi.r1);
                }

                if (lsi.r2!=0.0f)
                {
                    hit.indexList.push_back(lsi.p2);
                    hit.ratioList.push_back(lsi.r2);
                }

                insertIntersection(hit);
            }
        }

        return;
    }

    // GW: changed this ONE line
    osg::TriangleFunctor<DoublePrecisionTriangleIntersector> ti;

    ti.set(s,e);
    ti._limitOneIntersection = (_intersectionLimit == LIMIT_ONE_PER_DRAWABLE || _intersectionLimit == LIMIT_ONE);
    drawable->accept(ti);

    if (ti._hit)
    {
        osg::Geometry* geometry = drawable->asGeometry();

        for(TriangleIntersections::iterator thitr = ti._intersections.begin();
            thitr != ti._intersections.end();
            ++thitr)
        {

            // get ratio in s,e range
            double ratio = thitr->first;

            // remap ratio into _start, _end range
            double remap_ratio = ((s-_start).length() + ratio * (e-s).length() )/(_end-_start).length();

            if ( _intersectionLimit == LIMIT_NEAREST && !getIntersections().empty() )
            {
                if (remap_ratio >= getIntersections().begin()->ratio )
                    break;
                else
                    getIntersections().clear();
            }

            TriangleIntersection& triHit = thitr->second;

            Intersection hit;
            hit.ratio = remap_ratio;
            hit.matrix = iv.getModelMatrix();
            hit.nodePath = iv.getNodePath();
            hit.drawable = drawable;
            hit.primitiveIndex = triHit._index;

            hit.localIntersectionPoint = _start*(1.0-remap_ratio) + _end*remap_ratio;

            // OSG_NOTICE<<"Conventional: ratio="<<hit.ratio<<" ("<<hit.localIntersectionPoint<<")"<<std::endl;

            hit.localIntersectionNormal = triHit._normal;

            if (geometry)
            {
                osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
                if (vertices)
                {
                    osg::Vec3* first = &(vertices->front());

                    if (triHit._v1)
                    {
                        hit.indexList.push_back(triHit._v1-first);
                        hit.ratioList.push_back(triHit._r1);
                    }
                    if (triHit._v2)
                    {
                        hit.indexList.push_back(triHit._v2-first);
                        hit.ratioList.push_back(triHit._r2);
                    }
                    if (triHit._v3)
                    {
                        hit.indexList.push_back(triHit._v3-first);
                        hit.ratioList.push_back(triHit._r3);
                    }
                }
            }

            insertIntersection(hit);

        }
    }
}
