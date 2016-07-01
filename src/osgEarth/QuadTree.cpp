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
#undef NDEBUG
#include <cassert>
#include "QuadTree"
#include <osg/Geode>
#include <osg/TriangleIndexFunctor>
#include <osg/Timer>

#include <osg/io_utils>

using namespace osgEarth;

//#define VERBOSE_OUTPUT


////////////////////////////////////////////////////////////////////////////////
//
// BuildQuadTree Declarartion - class used for building an single QuadTree

struct BuildQuadTree
{
    BuildQuadTree(QuadTree& quadTree):
        _quadTree(quadTree) {}

    typedef std::vector< osg::Vec3 >            CenterList;
    typedef std::vector< unsigned int >         Indices;
    typedef std::vector< unsigned int >         AxisStack;

    bool build(QuadTree::BuildOptions& options, osg::Geometry* geometry);

    void computeDivisions(QuadTree::BuildOptions& options);

    int divide(QuadTree::BuildOptions& options, osg::BoundingBox& bb, int nodeIndex, unsigned int level);

    QuadTree&           _quadTree;

    osg::BoundingBox    _bb;
    AxisStack           _axisStack;
    Indices             _primitiveIndices;
    CenterList          _centers;

protected:

    BuildQuadTree& operator = (const BuildQuadTree&) { return *this; }
};

////////////////////////////////////////////////////////////////////////////////
//
// Functor for collecting triangle indices from Geometry
struct TriangleIndicesCollector
{
    TriangleIndicesCollector():
        _buildQuadTree(0)
    {
    }

    inline void operator () (unsigned int p0, unsigned int p1, unsigned int p2)
    {
        const osg::Vec3& v0 = (*(_buildQuadTree->_quadTree.getVertices()))[p0];
        const osg::Vec3& v1 = (*(_buildQuadTree->_quadTree.getVertices()))[p1];
        const osg::Vec3& v2 = (*(_buildQuadTree->_quadTree.getVertices()))[p2];

        unsigned int i = _buildQuadTree->_quadTree.addTriangle(QuadTree::Triangle(p0,p1,p2));

        osg::BoundingBox bb(v0,v0);
        bb.expandBy(v1);
        bb.expandBy(v2);

        _buildQuadTree->_centers.push_back(bb.center());
        _buildQuadTree->_primitiveIndices.push_back(i);

    }

    BuildQuadTree* _buildQuadTree;
};


////////////////////////////////////////////////////////////////////////////////
//
// BuildQuadTree Implementation

bool BuildQuadTree::build(QuadTree::BuildOptions& options, osg::Geometry* geometry)
{

#ifdef VERBOSE_OUTPUT
    OSG_NOTICE<<"QuadTreeBuilder::createQuadTree()"<<std::endl;
#endif

    osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
    if (!vertices) return false;

    if (vertices->size() <= options._targetNumTrianglesPerLeaf) return false;
    
#if OSG_VERSION_GREATER_OR_EQUAL(3,3,2)
    _bb = geometry->getBoundingBox();
#else
    _bb = geometry->getBound();
#endif

    _quadTree.setVertices(vertices);

    unsigned int estimatedSize = (unsigned int)(2.0*float(vertices->size())/float(options._targetNumTrianglesPerLeaf));

#ifdef VERBOSE_OUTPUT
    OSG_NOTICE<<"quadTree->_quadNodes.reserve()="<<estimatedSize<<std::endl<<std::endl;
#endif

    _quadTree.getNodes().reserve(estimatedSize*5);

    computeDivisions(options);

    options._numVerticesProcessed += vertices->size();

    _primitiveIndices.reserve(options._numTriangles);
    _centers.reserve(options._numTriangles);

    _quadTree.getTriangles().reserve(options._numTriangles);


    osg::TriangleIndexFunctor<TriangleIndicesCollector> collectTriangleIndices;
    collectTriangleIndices._buildQuadTree = this;
    geometry->accept(collectTriangleIndices);

    _primitiveIndices.reserve(vertices->size());

    QuadTree::QuadNode node(-1, _primitiveIndices.size());
    node.bb = _bb;

    int nodeNum = _quadTree.addNode(node);

    osg::BoundingBox bb = _bb;
    nodeNum = divide(options, bb, nodeNum, 0);

    // now reorder the triangle list so that it's in order as per the primitiveIndex list.
    QuadTree::TriangleList triangleList(_quadTree.getTriangles().size());
    for(unsigned int i=0; i<_primitiveIndices.size(); ++i)
    {
        triangleList[i] = _quadTree.getTriangle(_primitiveIndices[i]);
    }

    _quadTree.getTriangles().swap(triangleList);


#ifdef VERBOSE_OUTPUT
    OSG_NOTICE<<"Root nodeNum="<<nodeNum<<std::endl;
#endif


//    OSG_NOTICE<<"_quadNodes.size()="<<_quadNodes.size()<<"  estimated size = "<<estimatedSize<<std::endl;
//    OSG_NOTICE<<"_quadLeaves.size()="<<_quadLeaves.size()<<"  estimated size = "<<estimatedSize<<std::endl<<std::endl;


    return !_quadTree.getNodes().empty();
}

void BuildQuadTree::computeDivisions(QuadTree::BuildOptions& options)
{
    osg::Vec3 dimensions(_bb.xMax()-_bb.xMin(),
                         _bb.yMax()-_bb.yMin(),
                         _bb.zMax()-_bb.zMin());

#ifdef VERBOSE_OUTPUT
    OSG_NOTICE<<"computeDivisions("<<options._maxNumLevels<<") "<<dimensions<< " { "<<std::endl;
#endif

    _axisStack.reserve(options._maxNumLevels);

    for(unsigned int level=0; level<options._maxNumLevels; ++level)
    {
        int axis = (dimensions[0]>=dimensions[1])? 0 : 1;

        _axisStack.push_back(axis);
        dimensions[axis] *= 0.5f;

#ifdef VERBOSE_OUTPUT
        OSG_NOTICE<<"  "<<level<<", "<<dimensions<<", "<<axis<<std::endl;
#endif
    }

#ifdef VERBOSE_OUTPUT
    OSG_NOTICE<<"}"<<std::endl;
#endif
}

int BuildQuadTree::divide(QuadTree::BuildOptions& options, osg::BoundingBox& bb, int nodeIndex, unsigned int level)
{
    QuadTree::QuadNode& node = _quadTree.getNode(nodeIndex);

    bool needToDivide = level < _axisStack.size() &&
                        (node.first<0 && static_cast<unsigned int>(node.second)>options._targetNumTrianglesPerLeaf);

    if (!needToDivide)
    {
        if (node.first<0)
        {
            int istart = -node.first-1;
            int iend = istart+node.second-1;

            // leaf is done, now compute bound on it.
            node.bb.init();
            for(int i=istart; i<=iend; ++i)
            {
                const QuadTree::Triangle& tri = _quadTree.getTriangle(_primitiveIndices[i]);
                const osg::Vec3& v0 = (*_quadTree.getVertices())[tri.p0];
                const osg::Vec3& v1 = (*_quadTree.getVertices())[tri.p1];
                const osg::Vec3& v2 = (*_quadTree.getVertices())[tri.p2];
                node.bb.expandBy(v0);
                node.bb.expandBy(v1);
                node.bb.expandBy(v2);

            }

            if (node.bb.valid())
            {
                float epsilon = 1e-6f;
                node.bb._min.x() -= epsilon;
                node.bb._min.y() -= epsilon;
                node.bb._min.z() -= epsilon;
                node.bb._max.x() += epsilon;
                node.bb._max.y() += epsilon;
                node.bb._max.z() += epsilon;
            }

#ifdef VERBOSE_OUTPUT
            if (!node.bb.valid())
            {
                OSG_NOTICE<<"After reset "<<node.first<<","<<node.second<<std::endl;
                OSG_NOTICE<<"  bb._min ("<<node.bb._min<<")"<<std::endl;
                OSG_NOTICE<<"  bb._max ("<<node.bb._max<<")"<<std::endl;
            }
            else
            {
                OSG_NOTICE<<"Set bb for nodeIndex = "<<nodeIndex<<std::endl;
            }
#endif
        }

        return nodeIndex;

    }

    int axis = _axisStack[level];

#ifdef VERBOSE_OUTPUT
    OSG_NOTICE<<"divide("<<nodeIndex<<", "<<level<< "), axis="<<axis<<std::endl;
#endif

    if (node.first<0)
    {
        // leaf node as first <= 0, so look at dividing it.

        int istart = -node.first-1;
        int iend = istart+node.second-1;

        //OSG_NOTICE<<"  divide leaf"<<std::endl;

        float original_min = bb._min[axis];
        float original_max = bb._max[axis];

        float mid = (original_min+original_max)*0.5f;

        int originalLeftChildIndex = 0;
        int originalRightChildIndex = 0;
        bool insitueDivision = false;

        {
            //osg::Vec3Array* vertices = quadTree._vertices.get();
            int left = istart;
            int right = iend;

            while(left<right)
            {
                while(left<right && (_centers[_primitiveIndices[left]][axis]<=mid)) { ++left; }

                while(left<right && (_centers[_primitiveIndices[right]][axis]>mid)) { --right; }

                while(left<right && (_centers[_primitiveIndices[right]][axis]>mid)) { --right; }

                if (left<right)
                {
                    std::swap(_primitiveIndices[left], _primitiveIndices[right]);
                    ++left;
                    --right;
                }
            }

            if (left==right)
            {
                if (_centers[_primitiveIndices[left]][axis]<=mid) ++left;
                else --right;
            }

            QuadTree::QuadNode leftLeaf(-istart-1, (right-istart)+1);
            QuadTree::QuadNode rightLeaf(-left-1, (iend-left)+1);

#if 0
            OSG_NOTICE<<"In  node.first     ="<<node.first     <<" node.second     ="<<node.second<<std::endl;
            OSG_NOTICE<<"    leftLeaf.first ="<<leftLeaf.first <<" leftLeaf.second ="<<leftLeaf.second<<std::endl;
            OSG_NOTICE<<"    rightLeaf.first="<<rightLeaf.first<<" rightLeaf.second="<<rightLeaf.second<<std::endl;
            OSG_NOTICE<<"    left="<<left<<" right="<<right<<std::endl;

            if (node.second != (leftLeaf.second +rightLeaf.second))
            {
                OSG_NOTICE<<"*** Error in size, leaf.second="<<node.second
                                        <<", leftLeaf.second="<<leftLeaf.second
                                        <<", rightLeaf.second="<<rightLeaf.second<<std::endl;
            }
            else
            {
                OSG_NOTICE<<"Size OK, leaf.second="<<node.second
                                        <<", leftLeaf.second="<<leftLeaf.second
                                        <<", rightLeaf.second="<<rightLeaf.second<<std::endl;
            }
#endif

            if (leftLeaf.second<=0)
            {
                //OSG_NOTICE<<"LeftLeaf empty"<<std::endl;
                originalLeftChildIndex = 0;
                //originalRightChildIndex = addNode(rightLeaf);
                originalRightChildIndex = nodeIndex;
                insitueDivision = true;
            }
            else if (rightLeaf.second<=0)
            {
                //OSG_NOTICE<<"RightLeaf empty"<<std::endl;
                // originalLeftChildIndex = addNode(leftLeaf);
                originalLeftChildIndex = nodeIndex;
                originalRightChildIndex = 0;
                insitueDivision = true;
            }
            else
            {
                originalLeftChildIndex = _quadTree.addNode(leftLeaf);
                originalRightChildIndex = _quadTree.addNode(rightLeaf);
            }
        }


        float restore = bb._max[axis];
        bb._max[axis] = mid;

        //OSG_NOTICE<<"  divide leftLeaf "<<quadTree.getNode(nodeNum).first<<std::endl;
        int leftChildIndex = originalLeftChildIndex!=0 ? divide(options, bb, originalLeftChildIndex, level+1) : 0;

        bb._max[axis] = restore;

        restore = bb._min[axis];
        bb._min[axis] = mid;

        //OSG_NOTICE<<"  divide rightLeaf "<<quadTree.getNode(nodeNum).second<<std::endl;
        int rightChildIndex = originalRightChildIndex!=0 ? divide(options, bb, originalRightChildIndex, level+1) : 0;

        bb._min[axis] = restore;


        if (!insitueDivision)
        {
            // take a second reference to node we are working on as the std::vector<> resize could
            // have invalidate the previous node ref.
            QuadTree::QuadNode& newNodeRef = _quadTree.getNode(nodeIndex);

            newNodeRef.first = leftChildIndex;
            newNodeRef.second = rightChildIndex;

            insitueDivision = true;

            newNodeRef.bb.init();
            if (leftChildIndex!=0) newNodeRef.bb.expandBy(_quadTree.getNode(leftChildIndex).bb);
            if (rightChildIndex!=0) newNodeRef.bb.expandBy(_quadTree.getNode(rightChildIndex).bb);

            if (!newNodeRef.bb.valid())
            {
                OSG_NOTICE<<"leftChildIndex="<<leftChildIndex<<" && originalLeftChildIndex="<<originalLeftChildIndex<<std::endl;
                OSG_NOTICE<<"rightChildIndex="<<rightChildIndex<<" && originalRightChildIndex="<<originalRightChildIndex<<std::endl;

                OSG_NOTICE<<"Invalid BB leftChildIndex="<<leftChildIndex<<", "<<rightChildIndex<<std::endl;
                OSG_NOTICE<<"  bb._min ("<<newNodeRef.bb._min<<")"<<std::endl;
                OSG_NOTICE<<"  bb._max ("<<newNodeRef.bb._max<<")"<<std::endl;

                if (leftChildIndex!=0)
                {
                    OSG_NOTICE<<"  getNode(leftChildIndex).bb min = "<<_quadTree.getNode(leftChildIndex).bb._min<<std::endl;
                    OSG_NOTICE<<"                                 max = "<<_quadTree.getNode(leftChildIndex).bb._max<<std::endl;
                }
                if (rightChildIndex!=0)
                {
                    OSG_NOTICE<<"  getNode(rightChildIndex).bb min = "<<_quadTree.getNode(rightChildIndex).bb._min<<std::endl;
                    OSG_NOTICE<<"                              max = "<<_quadTree.getNode(rightChildIndex).bb._max<<std::endl;
                }
            }
        }
    }
    else
    {
        OSG_NOTICE<<"NOT expecting to get here"<<std::endl;
    }

    return nodeIndex;

}

////////////////////////////////////////////////////////////////////////////////
//
// IntersectQuadTree
//
struct IntersectQuadTree
{
    IntersectQuadTree(const osg::Vec3Array& vertices,
                    const QuadTree::QuadNodeList& nodes,
                    const QuadTree::TriangleList& triangles,
                    QuadTree::LineSegmentIntersections& intersections,
                    const osg::Vec3d& s, const osg::Vec3d& e):
                        _vertices(vertices),
                        _quadNodes(nodes),
                        _triangles(triangles),
                        _intersections(intersections),
                        _s(s),
                        _e(e)
    {
        _d = e - s;
        _length = _d.length();
        _inverse_length = _length!=0.0f ? 1.0f/_length : 0.0;
        _d *= _inverse_length;

        _d_invX = _d.x()!=0.0f ? _d/_d.x() : osg::Vec3(0.0f,0.0f,0.0f);
        _d_invY = _d.y()!=0.0f ? _d/_d.y() : osg::Vec3(0.0f,0.0f,0.0f);
        _d_invZ = _d.z()!=0.0f ? _d/_d.z() : osg::Vec3(0.0f,0.0f,0.0f);
    }

    void intersect(const QuadTree::QuadNode& node, const osg::Vec3& s, const osg::Vec3& e) const;
    bool intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const;

    const osg::Vec3Array&               _vertices;
    const QuadTree::QuadNodeList&           _quadNodes;
    const QuadTree::TriangleList&         _triangles;
    QuadTree::LineSegmentIntersections&   _intersections;

    osg::Vec3 _s;
    osg::Vec3 _e;

    osg::Vec3 _d;
    float     _length;
    float     _inverse_length;

    osg::Vec3 _d_invX;
    osg::Vec3 _d_invY;
    osg::Vec3 _d_invZ;


protected:

    IntersectQuadTree& operator = (const IntersectQuadTree&) { return *this; }
};


void IntersectQuadTree::intersect(const QuadTree::QuadNode& node, const osg::Vec3& ls, const osg::Vec3& le) const
{
    static const float esplison = 1e-10f;
    if (node.first<0)
    {
        // treat as a leaf

        //OSG_NOTICE<<"QuadTree::intersect("<<&leaf<<")"<<std::endl;
        int istart = -node.first-1;
        int iend = istart + node.second;

        for(int i=istart; i<iend; ++i)
        {
            //const Triangle& tri = _triangles[_primitiveIndices[i]];
            const QuadTree::Triangle& tri = _triangles[i];
            // OSG_NOTICE<<"   tri("<<tri.p1<<","<<tri.p2<<","<<tri.p3<<")"<<std::endl;

            const osg::Vec3& v0 = _vertices[tri.p0];
            const osg::Vec3& v1 = _vertices[tri.p1];
            const osg::Vec3& v2 = _vertices[tri.p2];

            osg::Vec3 T = _s - v0;
            osg::Vec3 E2 = v2 - v0;
            osg::Vec3 E1 = v1 - v0;

            osg::Vec3 P =  _d ^ E2;

            float det = P * E1;

            float r,r0,r1,r2;

            if (det>esplison)
            {
                float u = (P*T);
                if (u<0.0 || u>det) continue;

                osg::Vec3 Q = T ^ E1;
                float v = (Q*_d);
                if (v<0.0 || v>det) continue;

                if ((u+v)> det) continue;

                float inv_det = 1.0f/det;
                float t = (Q*E2)*inv_det;
                if (t<0.0 || t>_length) continue;

                u *= inv_det;
                v *= inv_det;

                r0 = 1.0f-u-v;
                r1 = u;
                r2 = v;
                r = t * _inverse_length;
            }
            else if (det<-esplison)
            {

                float u = (P*T);
                if (u>0.0 || u<det) continue;

                osg::Vec3 Q = T ^ E1;
                float v = (Q*_d);
                if (v>0.0 || v<det) continue;

                if ((u+v) < det) continue;

                float inv_det = 1.0f/det;
                float t = (Q*E2)*inv_det;
                if (t<0.0 || t>_length) continue;

                u *= inv_det;
                v *= inv_det;

                r0 = 1.0f-u-v;
                r1 = u;
                r2 = v;
                r = t * _inverse_length;
            }
            else
            {
                continue;
            }

            osg::Vec3 in = v0*r0 + v1*r1 + v2*r2;
            osg::Vec3 normal = E1^E2;
            normal.normalize();

#if 1
            _intersections.push_back(QuadTree::LineSegmentIntersection());
            QuadTree::LineSegmentIntersection& intersection = _intersections.back();

            intersection.ratio = r;
            intersection.primitiveIndex = i;
            intersection.intersectionPoint = in;
            intersection.intersectionNormal = normal;

            intersection.p0 = tri.p0;
            intersection.p1 = tri.p1;
            intersection.p2 = tri.p2;
            intersection.r0 = r0;
            intersection.r1 = r1;
            intersection.r2 = r2;

#endif
            // OSG_NOTICE<<"  got intersection ("<<in<<") ratio="<<r<<std::endl;
        }
    }
    else
    {
        if (node.first>0)
        {
            osg::Vec3 l(ls), e(le);
            if (intersectAndClip(l,e, _quadNodes[node.first].bb))
            {
                intersect(_quadNodes[node.first], l, e);
            }
        }
        if (node.second>0)
        {
            osg::Vec3 l(ls), e(le);
            if (intersectAndClip(l,e, _quadNodes[node.second].bb))
            {
                intersect(_quadNodes[node.second], l, e);
            }
        }
    }
}

bool IntersectQuadTree::intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const
{
    //return true;

    //if (!bb.valid()) return true;

    // compate s and e against the xMin to xMax range of bb.
    if (s.x()<=e.x())
    {

        // trivial reject of segment wholely outside.
        if (e.x()<bb.xMin()) return false;
        if (s.x()>bb.xMax()) return false;

        if (s.x()<bb.xMin())
        {
            // clip s to xMin.
            s = s+_d_invX*(bb.xMin()-s.x());
        }

        if (e.x()>bb.xMax())
        {
            // clip e to xMax.
            e = s+_d_invX*(bb.xMax()-s.x());
        }
    }
    else
    {
        if (s.x()<bb.xMin()) return false;
        if (e.x()>bb.xMax()) return false;

        if (e.x()<bb.xMin())
        {
            // clip s to xMin.
            e = s+_d_invX*(bb.xMin()-s.x());
        }

        if (s.x()>bb.xMax())
        {
            // clip e to xMax.
            s = s+_d_invX*(bb.xMax()-s.x());
        }
    }

    // compate s and e against the yMin to yMax range of bb.
    if (s.y()<=e.y())
    {

        // trivial reject of segment wholely outside.
        if (e.y()<bb.yMin()) return false;
        if (s.y()>bb.yMax()) return false;

        if (s.y()<bb.yMin())
        {
            // clip s to yMin.
            s = s+_d_invY*(bb.yMin()-s.y());
        }

        if (e.y()>bb.yMax())
        {
            // clip e to yMax.
            e = s+_d_invY*(bb.yMax()-s.y());
        }
    }
    else
    {
        if (s.y()<bb.yMin()) return false;
        if (e.y()>bb.yMax()) return false;

        if (e.y()<bb.yMin())
        {
            // clip s to yMin.
            e = s+_d_invY*(bb.yMin()-s.y());
        }

        if (s.y()>bb.yMax())
        {
            // clip e to yMax.
            s = s+_d_invY*(bb.yMax()-s.y());
        }
    }

    // compate s and e against the zMin to zMax range of bb.
    if (s.z()<=e.z())
    {

        // trivial reject of segment wholely outside.
        if (e.z()<bb.zMin()) return false;
        if (s.z()>bb.zMax()) return false;

        if (s.z()<bb.zMin())
        {
            // clip s to zMin.
            s = s+_d_invZ*(bb.zMin()-s.z());
        }

        if (e.z()>bb.zMax())
        {
            // clip e to zMax.
            e = s+_d_invZ*(bb.zMax()-s.z());
        }
    }
    else
    {
        if (s.z()<bb.zMin()) return false;
        if (e.z()>bb.zMax()) return false;

        if (e.z()<bb.zMin())
        {
            // clip s to zMin.
            e = s+_d_invZ*(bb.zMin()-s.z());
        }

        if (s.z()>bb.zMax())
        {
            // clip e to zMax.
            s = s+_d_invZ*(bb.zMax()-s.z());
        }
    }

    // OSG_NOTICE<<"clampped segment "<<s<<" "<<e<<std::endl;

    // if (s==e) return false;

    return true;
}


////////////////////////////////////////////////////////////////////////////////
//
// QuadTree::BuildOptions

QuadTree::BuildOptions::BuildOptions():
_numVerticesProcessed(0),
_targetNumTrianglesPerLeaf(4),
_maxNumLevels(32),
_numTriangles(0)
{
    //nop
}

////////////////////////////////////////////////////////////////////////////////
//
// QuadTree

QuadTree::QuadTree()
{
}

QuadTree::QuadTree(const QuadTree& rhs, const osg::CopyOp& copyop):
    Shape(rhs, copyop),
    _vertices(rhs._vertices),
    _quadNodes(rhs._quadNodes),
    _triangles(rhs._triangles)
{
}

bool QuadTree::build(BuildOptions& options, osg::Geometry* geometry)
{
    BuildQuadTree build(*this);
    return build.build(options, geometry);
}

bool QuadTree::intersect(const osg::Vec3d& start, const osg::Vec3d& end, LineSegmentIntersections& intersections) const
{
    if (_quadNodes.empty())
    {
        OSG_NOTICE<<"Warning: _quadTree is empty"<<std::endl;
        return false;
    }

    unsigned int numIntersectionsBefore = intersections.size();

    IntersectQuadTree intersector(*_vertices,
                                _quadNodes,
                                _triangles,
                                intersections,
                                start, end);

    intersector.intersect(getNode(0), start, end);

    return numIntersectionsBefore != intersections.size();
}

////////////////////////////////////////////////////////////////////////////////
//
// QuadTreeBuilder
QuadTreeBuilder::QuadTreeBuilder():
    osg::NodeVisitor()
{
    setTraversalMode( TRAVERSE_ALL_CHILDREN );
    this->setNodeMaskOverride( ~0 );

    _quadTreePrototype = new QuadTree;
}

QuadTreeBuilder::QuadTreeBuilder(const QuadTreeBuilder& rhs) :
    osg::NodeVisitor( rhs ),
    _buildOptions(rhs._buildOptions),
    _quadTreePrototype(rhs._quadTreePrototype)
{
}

void QuadTreeBuilder::apply(osg::Geode& geode)
{
    for(unsigned int i=0; i<geode.getNumDrawables(); ++i)
    {

        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if (geom)
        {
            QuadTree* previous = dynamic_cast<QuadTree*>(geom->getShape());
            if (previous) continue;

            osg::ref_ptr<osg::Object> obj = _quadTreePrototype->cloneType();
            osg::ref_ptr<QuadTree> quadTree = dynamic_cast<QuadTree*>(obj.get());

            if (quadTree.valid() && quadTree->build(_buildOptions, geom))
            {
                geom->setShape(quadTree.get());
            }
        }
    }
}
