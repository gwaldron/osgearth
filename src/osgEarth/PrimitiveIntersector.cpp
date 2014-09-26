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

#include <osgEarth/PrimitiveIntersector>
#include <osgEarth/StringUtils>
#include <osgEarth/Utils>
#include <osg/Geode>
#include <osg/KdTree>
#include <osg/Notify>
#include <osg/TemplatePrimitiveFunctor>

#define LC "[PrmitiveIntersector] "

using namespace osgEarth;

namespace
{
struct PrimitiveIntersection
{
    PrimitiveIntersection(unsigned int index, const osg::Vec3d& normal, float r1, const osg::Vec3d* v1, float r2, const osg::Vec3d* v2, float r3, const osg::Vec3d* v3):
        _index(index),
        _normal(normal),
        _r1(r1),
        _v1(v1),
        _r2(r2),
        _v2(v2),
        _r3(r3),
        _v3(v3) {}

    unsigned int        _index;
    const osg::Vec3d     _normal;
    float               _r1;
    const osg::Vec3d*    _v1;
    float               _r2;
    const osg::Vec3d*    _v2;
    float               _r3;
    const osg::Vec3d*    _v3;

protected:

    PrimitiveIntersection& operator = (const PrimitiveIntersection&) { return *this; }
};

typedef std::multimap<float,PrimitiveIntersection> PrimitiveIntersections;

struct PrimitiveIntersectorFunctor
{
    osg::Vec3d   _s;
    osg::Vec3d   _d;
    osg::Vec3d    _thickness;

    float       _length;

    int         _index;
    float       _ratio;
    bool        _hit;
    bool        _limitOneIntersection;

    PrimitiveIntersections _intersections;

    PrimitiveIntersectorFunctor()
    {
        _length = 0.0f;
        _index = 0;
        _ratio = 0.0f;
        _hit = false;
        _limitOneIntersection = false;
    }

    void set(const osg::Vec3d& start, osg::Vec3d& end, const osg::Vec3d& thickness, float ratio=FLT_MAX)
    {
        _hit=false;
        _thickness = thickness;
        _index = 0;
        _ratio = ratio;

        _s = start;
        _d = end - start;
        _length = _d.length();
        _d /= _length;
    }

    //POINT
    inline void operator () (const osg::Vec3d& p, bool treatVertexDataAsTemporary)
    {
        if (_limitOneIntersection && _hit) return;

        osg::Vec3d n = _d ^ _thickness;

        osg::Vec3d v1 = p + _thickness;
        osg::Vec3d v2 = p - n;
        osg::Vec3d v3 = p - _thickness;
        osg::Vec3d v4 = p + n;

        //this->operator()(v1, v2, v3, v4, treatVertexDataAsTemporary);
        this->triNoBuffer(v1, v2, v3, treatVertexDataAsTemporary);
        --_index;
        this->triNoBuffer(v1, v3, v4, treatVertexDataAsTemporary);
    }

    //LINE
    inline void operator () (const osg::Vec3d& v1, const osg::Vec3d& v2, bool treatVertexDataAsTemporary)
     {
        if (_limitOneIntersection && _hit) return;

        float thickness =  _thickness.length();
        osg::Vec3d l12 = v2 - v1;
        osg::Vec3d ln = _d ^ l12;
        ln.normalize();

        osg::Vec3d vq1 = v1 + ln*thickness;
        osg::Vec3d vq2 = v2 + ln*thickness;
        osg::Vec3d vq3 = v2 - ln*thickness;
        osg::Vec3d vq4 = v1 - ln*thickness;

        //this->operator()(vq1, vq2, vq3, vq4, treatVertexDataAsTemporary);
        this->triNoBuffer(vq1, vq2, vq3, treatVertexDataAsTemporary);
        if (_limitOneIntersection && _hit) return;

        --_index;
        this->triNoBuffer(vq1, vq3, vq4, treatVertexDataAsTemporary);
    }

    //QUAD
    inline void operator () (
        const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3, const osg::Vec3d& v4, bool treatVertexDataAsTemporary
        )
    {
        if (_limitOneIntersection && _hit) return;

        this->operator()(v1, v2, v3, treatVertexDataAsTemporary);
        if (_limitOneIntersection && _hit) return;

        --_index;
        this->operator()(v1, v3, v4, treatVertexDataAsTemporary);
    }

    //TRIANGLE (buffered)
    inline void operator () (
        const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3, bool treatVertexDataAsTemporary
        )
    {
        if (_limitOneIntersection && _hit) return;

        // first do a simple test against the unbuffered triangle:
        this->triNoBuffer(v1, v2, v3, treatVertexDataAsTemporary);
        if (_limitOneIntersection && _hit) return;

        // now buffer each edge and test against that.
        float thickness = _thickness.length();
        osg::Vec3d ln, buf;

        osg::Vec3d v12 = v2-v1; ln = _d ^ v12; ln.normalize(); buf = ln*thickness;
        --_index;
        this->triNoBuffer(v1+buf, v2+buf, v2-buf, treatVertexDataAsTemporary);
        if (_limitOneIntersection && _hit) return;

        --_index;
        this->triNoBuffer(v1+buf, v3-buf, v1-buf, treatVertexDataAsTemporary);
        if (_limitOneIntersection && _hit) return;

        osg::Vec3d v23 = v3-v1; ln = _d ^ v23; ln.normalize(); buf = ln*thickness;
        --_index;
        this->triNoBuffer(v2+buf, v3+buf, v3-buf, treatVertexDataAsTemporary );
        if (_limitOneIntersection && _hit) return;

        --_index;
        this->triNoBuffer(v2+buf, v3-buf, v2-buf, treatVertexDataAsTemporary );
        if (_limitOneIntersection && _hit) return;

        osg::Vec3d v31 = v1-v3; ln = _d ^ v31; ln.normalize(); buf = ln*thickness;
        --_index;
        this->triNoBuffer(v3+buf, v1+buf, v1-buf, treatVertexDataAsTemporary);
        if (_limitOneIntersection && _hit) return;

        --_index;
        this->triNoBuffer(v3+buf, v1-buf, v3-buf, treatVertexDataAsTemporary);
    }

    //TRIANGLE (no buffer applied)
    inline void triNoBuffer(const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3, bool treatVertexDataAsTemporary)
    {
        ++_index;

        if (_limitOneIntersection && _hit) return;

        if (v1==v2 || v2==v3 || v1==v3) return;

        osg::Vec3d v12 = v2-v1;
        osg::Vec3d n12 = v12^_d;
        float ds12 = (_s-v1)*n12;
        float d312 = (v3-v1)*n12;
        if (d312>=0.0f)
        {
            if (ds12<0.0f) return;
            if (ds12>d312) return;
        }
        else                     // d312 < 0
        {
            if (ds12>0.0f) return;
            if (ds12<d312) return;
        }

        osg::Vec3d v23 = v3-v2;
        osg::Vec3d n23 = v23^_d;
        float ds23 = (_s-v2)*n23;
        float d123 = (v1-v2)*n23;
        if (d123>=0.0f)
        {
            if (ds23<0.0f) return;
            if (ds23>d123) return;
        }
        else                     // d123 < 0
        {
            if (ds23>0.0f) return;
            if (ds23<d123) return;
        }

        osg::Vec3d v31 = v1-v3;
        osg::Vec3d n31 = v31^_d;
        float ds31 = (_s-v3)*n31;
        float d231 = (v2-v3)*n31;
        if (d231>=0.0f)
        {
            if (ds31<0.0f) return;
            if (ds31>d231) return;
        }
        else                     // d231 < 0
        {
            if (ds31>0.0f) return;
            if (ds31<d231) return;
        }


        float r3;
        if (ds12==0.0f) r3=0.0f;
        else if (d312!=0.0f) r3 = ds12/d312;
        else return; // the triangle and the line must be parallel intersection.

        float r1;
        if (ds23==0.0f) r1=0.0f;
        else if (d123!=0.0f) r1 = ds23/d123;
        else return; // the triangle and the line must be parallel intersection.

        float r2;
        if (ds31==0.0f) r2=0.0f;
        else if (d231!=0.0f) r2 = ds31/d231;
        else return; // the triangle and the line must be parallel intersection.

        float total_r = (r1+r2+r3);
        if (total_r!=1.0f)
        {
            if (total_r==0.0f) return; // the triangle and the line must be parallel intersection.
            float inv_total_r = 1.0f/total_r;
            r1 *= inv_total_r;
            r2 *= inv_total_r;
            r3 *= inv_total_r;
        }

        osg::Vec3d in = v1*r1+v2*r2+v3*r3;
        if (!in.valid())
        {
            //OE_WARN << LC << "Picked up error in TriangleIntersect" << std::endl;;
            return;
        }

        float d = (in-_s)*_d;

        if (d<0.0f) return;
        if (d>_length) return;

        osg::Vec3d normal = v12^v23;
        normal.normalize();

        float r = d/_length;


        if (treatVertexDataAsTemporary)
        {
            _intersections.insert(std::pair<const float,PrimitiveIntersection>(r,PrimitiveIntersection(_index-1,normal,r1,0,r2,0,r3,0)));
        }
        else
        {
            _intersections.insert(std::pair<const float,PrimitiveIntersection>(r,PrimitiveIntersection(_index-1,normal,r1,&v1,r2,&v2,r3,&v3)));
        }
        _hit = true;
    }


};

} //namespace

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  PrimitiveIntersector
//
PrimitiveIntersector::PrimitiveIntersector()
{
}

PrimitiveIntersector::PrimitiveIntersector(CoordinateFrame cf, double x, double y, double thickness):
    Intersector(cf),
    _parent(0),
    _overlayIgnore(false)
{
    switch(cf)
    {
        case WINDOW: _start.set(x,y,0.0); _end.set(x,y,1.0); break;
        case PROJECTION : _start.set(x,y,-1.0); _end.set(x,y,1.0); break;
        case VIEW : _start.set(x,y,0.0); _end.set(x,y,1.0); break;
        case MODEL : _start.set(x,y,0.0); _end.set(x,y,1.0); break;
    }

    setThickness(thickness);
}

PrimitiveIntersector::PrimitiveIntersector(CoordinateFrame cf, const osg::Vec3d& start, const osg::Vec3d& end, double thickness, bool overlayIgnore):
    Intersector(cf),
    _parent(0),
    _overlayIgnore(overlayIgnore)
{
  _start.set(start);
  _end.set(end);

  setThickness(thickness);
}

PrimitiveIntersector::Intersection::Intersection(const PrimitiveIntersector::Intersection &rhs)
{
  ratio = rhs.ratio;
  nodePath = rhs.nodePath;
  drawable = rhs.drawable;
  matrix = rhs.matrix;
  localIntersectionPoint = rhs.localIntersectionPoint;
  localIntersectionNormal = rhs.localIntersectionNormal;
  indexList = rhs.indexList;
  ratioList = rhs.ratioList;
  primitiveIndex = rhs.primitiveIndex;
}

void PrimitiveIntersector::setThickness(double thickness)
{
  _thicknessVal = thickness;
  double halfThickness = 0.5 * thickness;
  _thickness.set(_start.x()+halfThickness, _start.y()+halfThickness, _start.z()+halfThickness);
}

osgUtil::Intersector* PrimitiveIntersector::clone(osgUtil::IntersectionVisitor& iv)
{
    if (_coordinateFrame==MODEL && iv.getModelMatrix()==0)
    {
        osg::ref_ptr<PrimitiveIntersector> lsi = new PrimitiveIntersector;
        
        lsi->_start = _start;
        lsi->_end = _end;
        lsi->_thickness = _thickness;
        lsi->_thicknessVal = _thicknessVal;
        lsi->_parent = this;
        lsi->_intersectionLimit = _intersectionLimit;

        return lsi.release();
    }

    // compute the matrix that takes this Intersector from its CoordinateFrame into the local MODEL coordinate frame
    // that geometry in the scene graph will always be in.
    osg::Matrix matrix;
    osg::Matrix inverse;

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

    inverse.invert(matrix);

    osg::ref_ptr<PrimitiveIntersector> lsi = new PrimitiveIntersector;
    lsi->_start = _start*inverse;
    lsi->_end = _end*inverse;
    lsi->_thickness = _thickness*inverse;
    lsi->_thicknessVal = _thicknessVal;
    lsi->_parent = this;
    lsi->_intersectionLimit = _intersectionLimit;
    
    return lsi.release();
}

bool PrimitiveIntersector::enter(const osg::Node& node)
{
    if (reachedLimit()) return false;

    osg::BoundingSphere bs = node.getBound();
    if (bs.valid())
    {
        bs.radius() += (_thickness - _start).length();
    }

    return !node.isCullingActive() || intersects(bs);
}

void PrimitiveIntersector::leave()
{
    // do nothing
}

void PrimitiveIntersector::intersect(osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable)
{
    if (reachedLimit()) return;

    osg::BoundingBox bb = Utils::getBoundingBox(drawable);

    if (bb.valid())
        bb.expandBy(osg::BoundingSphere(bb.center(), (_thickness - _start).length()));

    osg::Vec3d s(_start), e(_end);
    if ( !intersectAndClip( s, e, bb ) ) return;

    if (iv.getDoDummyTraversal()) return;


    osg::TemplatePrimitiveFunctor<PrimitiveIntersectorFunctor> ti;

    ti.set(s,e,_thickness-_start);
    ti._limitOneIntersection = (_intersectionLimit == LIMIT_ONE_PER_DRAWABLE || _intersectionLimit == LIMIT_ONE);
    drawable->accept(ti);

    if (ti._hit)
    {
        osg::Geometry* geometry = drawable->asGeometry();

        for(PrimitiveIntersections::iterator thitr = ti._intersections.begin(); thitr != ti._intersections.end(); ++thitr)
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

            PrimitiveIntersection& triHit = thitr->second;

            Intersection hit;
            hit.ratio = remap_ratio;
            hit.matrix = iv.getModelMatrix();
            hit.nodePath = iv.getNodePath();
            hit.drawable = drawable;
            hit.primitiveIndex = findPrimitiveIndex(drawable, triHit._index);

            hit.localIntersectionPoint = _start*(1.0-remap_ratio) + _end*remap_ratio;

            hit.localIntersectionNormal = triHit._normal;

            if (geometry)
            {
                osg::Vec3dArray* vertices = dynamic_cast<osg::Vec3dArray*>(geometry->getVertexArray());
                if (vertices)
                {
                    osg::Vec3d* first = &(vertices->front());
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

void PrimitiveIntersector::reset()
{
    Intersector::reset();

    _intersections.clear();
}

bool PrimitiveIntersector::intersects(const osg::BoundingSphere& bs)
{
    // if bs not valid then return true based on the assumption that an invalid sphere is yet to be defined.
    if (!bs.valid()) return true;

    osg::Vec3d sm = _start - bs._center;
    double c = sm.length2()-bs._radius*bs._radius;
    if (c<0.0) return true;

    osg::Vec3d se = _end-_start;
    double a = se.length2();
    double b = (sm*se)*2.0;
    double d = b*b-4.0*a*c;

    if (d<0.0) return false;

    d = sqrt(d);

    double div = 1.0/(2.0*a);

    double r1 = (-b-d)*div;
    double r2 = (-b+d)*div;

    if (r1<=0.0 && r2<=0.0) return false;

    if (r1>=1.0 && r2>=1.0) return false;

    if (_intersectionLimit == LIMIT_NEAREST && !getIntersections().empty())
    {
        double ratio = (sm.length() - bs._radius) / sqrt(a);
        if (ratio >= getIntersections().begin()->ratio) return false;
    }

    // passed all the rejection tests so line must intersect bounding sphere, return true.
    return true;
}

bool PrimitiveIntersector::intersectAndClip(osg::Vec3d& s, osg::Vec3d& e,const osg::BoundingBox& bbInput)
{
    osg::Vec3d bb_min(bbInput._min);
    osg::Vec3d bb_max(bbInput._max);

    double epsilon = 1e-4;
    bb_min.x() -= epsilon;
    bb_min.y() -= epsilon;
    bb_min.z() -= epsilon;
    bb_max.x() += epsilon;
    bb_max.y() += epsilon;
    bb_max.z() += epsilon;

    // compate s and e against the xMin to xMax range of bb.
    if (s.x()<=e.x())
    {

        // trivial reject of segment wholely outside.
        if (e.x()<bb_min.x()) return false;
        if (s.x()>bb_max.x()) return false;

        if (s.x()<bb_min.x())
        {
            // clip s to xMin.
            s = s+(e-s)*(bb_min.x()-s.x())/(e.x()-s.x());
        }

        if (e.x()>bb_max.x())
        {
            // clip e to xMax.
            e = s+(e-s)*(bb_max.x()-s.x())/(e.x()-s.x());
        }
    }
    else
    {
        if (s.x()<bb_min.x()) return false;
        if (e.x()>bb_max.x()) return false;

        if (e.x()<bb_min.x())
        {
            // clip s to xMin.
            e = s+(e-s)*(bb_min.x()-s.x())/(e.x()-s.x());
        }

        if (s.x()>bb_max.x())
        {
            // clip e to xMax.
            s = s+(e-s)*(bb_max.x()-s.x())/(e.x()-s.x());
        }
    }

    // compate s and e against the yMin to yMax range of bb.
    if (s.y()<=e.y())
    {

        // trivial reject of segment wholely outside.
        if (e.y()<bb_min.y()) return false;
        if (s.y()>bb_max.y()) return false;

        if (s.y()<bb_min.y())
        {
            // clip s to yMin.
            s = s+(e-s)*(bb_min.y()-s.y())/(e.y()-s.y());
        }

        if (e.y()>bb_max.y())
        {
            // clip e to yMax.
            e = s+(e-s)*(bb_max.y()-s.y())/(e.y()-s.y());
        }
    }
    else
    {
        if (s.y()<bb_min.y()) return false;
        if (e.y()>bb_max.y()) return false;

        if (e.y()<bb_min.y())
        {
            // clip s to yMin.
            e = s+(e-s)*(bb_min.y()-s.y())/(e.y()-s.y());
        }

        if (s.y()>bb_max.y())
        {
            // clip e to yMax.
            s = s+(e-s)*(bb_max.y()-s.y())/(e.y()-s.y());
        }
    }

    // compate s and e against the zMin to zMax range of bb.
    if (s.z()<=e.z())
    {

        // trivial reject of segment wholely outside.
        if (e.z()<bb_min.z()) return false;
        if (s.z()>bb_max.z()) return false;

        if (s.z()<bb_min.z())
        {
            // clip s to zMin.
            s = s+(e-s)*(bb_min.z()-s.z())/(e.z()-s.z());
        }

        if (e.z()>bb_max.z())
        {
            // clip e to zMax.
            e = s+(e-s)*(bb_max.z()-s.z())/(e.z()-s.z());
        }
    }
    else
    {
        if (s.z()<bb_min.z()) return false;
        if (e.z()>bb_max.z()) return false;

        if (e.z()<bb_min.z())
        {
            // clip s to zMin.
            e = s+(e-s)*(bb_min.z()-s.z())/(e.z()-s.z());
        }

        if (s.z()>bb_max.z())
        {
            // clip e to zMax.
            s = s+(e-s)*(bb_max.z()-s.z())/(e.z()-s.z());
        }
    }

    return true;
}

unsigned int PrimitiveIntersector::findPrimitiveIndex(osg::Drawable* drawable, unsigned int index)
{
    if (!drawable)
      return index;

    const osg::Geometry* geom = drawable->asGeometry();
    if ( geom )
    {
        unsigned int primIndex = 0;
        unsigned int encounteredPrims = 0;

        const osg::Geometry::PrimitiveSetList& primSets = geom->getPrimitiveSetList();
        for( osg::Geometry::PrimitiveSetList::const_iterator i = primSets.begin(); i != primSets.end(); ++i )
        {
            bool simple = false;
            unsigned int numPrims = 0;

            const osg::PrimitiveSet* pset = i->get();
            switch( pset->getMode() )
            {
            case osg::PrimitiveSet::TRIANGLE_STRIP:
            case osg::PrimitiveSet::TRIANGLE_FAN:
                numPrims = osg::maximum(pset->getNumIndices() - 2, 0U);
                encounteredPrims += numPrims;
                break;
            case osg::PrimitiveSet::QUAD_STRIP:
                numPrims = osg::maximum((pset->getNumIndices() - 2) / 2, 0U);
                encounteredPrims += numPrims;
                break;
            case osg::PrimitiveSet::LINE_STRIP:
                numPrims = osg::maximum(pset->getNumIndices() - 1, 0U);
                encounteredPrims += numPrims;
                break;
            case osg::PrimitiveSet::LINE_LOOP:
                numPrims = pset->getNumIndices();
                encounteredPrims += numPrims;
                break;
            default:
                numPrims = pset->getNumPrimitives();
                primIndex += osg::minimum(numPrims, index - encounteredPrims);
                encounteredPrims += numPrims;
                simple = true;
            }

            if (encounteredPrims > index)
                return primIndex;

            // primIndex already incremented above for simple primitives
            if (!simple)
                primIndex++;
        }
    }

    //Should never reach here
    OE_DEBUG << LC << "Could not find primitive index!" << std::endl;

    return index;
}
