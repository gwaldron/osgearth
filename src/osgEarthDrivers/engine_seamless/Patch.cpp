/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2010 Pelican Ventures, Inc.
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

#include "Patch"
#include "PatchSet"

#include <algorithm>
#include <iterator>

#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include <osg/Vec3d>

namespace seamless
{
using namespace osg;

class MyNodeAcceptOp : public NodeAcceptOp
{
public:
    MyNodeAcceptOp(NodeVisitor& nv) : NodeAcceptOp(nv) {}
    MyNodeAcceptOp(const NodeAcceptOp& naop) : NodeAcceptOp(naop) {}
    template<typename T>
    void operator()(T node) { return node->accept(_nv); }
};

Patch::Patch()
    : _errorThreshold(.5f)
{
}

Patch::Patch(const Patch& rhs, const CopyOp& copyop)
    : Node(rhs, copyop), _errorThreshold(rhs._errorThreshold)
{
    for (int res = 0; res < 2; ++res)
        for (int i = 0; i < 4; ++i)
            _trile[res][i]
                = static_cast<Geode*>(copyop(rhs._trile[res][i].get()));
    for (int strip = 0; strip < 4; ++strip)
        for (int i = 0; i < 4; ++i)
            _strip[strip][i]
                = static_cast<Geode*>(copyop(rhs._strip[strip][i].get()));
    _data =static_cast<Data*>(copyop(rhs._data.get()));
    _patchSet = static_cast<PatchSet*>(copyop(rhs._patchSet.get()));
}

Patch::~Patch()
{
}

Patch::Data::Data()
{
}

Patch::Data::Data(const Patch::Data& rhs, const osg::CopyOp& copyop)
    : vertexData(rhs.vertexData, copyop), normalData(rhs.normalData, copyop),
      colorData(rhs.colorData, copyop),
      secondaryColorData(rhs.secondaryColorData, copyop),
      fogCoordData(rhs.fogCoordData, copyop)
{
    for (Geometry::ArrayDataList::const_iterator
             itr = rhs.texCoordList.begin(), end = rhs.texCoordList.end();
         itr !=end;
         ++itr)
        texCoordList.push_back(Geometry::ArrayData(*itr, copyop));
    for (Geometry::ArrayDataList::const_iterator
             itr = rhs.vertexAttribList.begin(),
             end = rhs.vertexAttribList.end();
         itr !=end;
         ++itr)
        vertexAttribList.push_back(Geometry::ArrayData(*itr, copyop));
}

void Patch::dirtyVertexData()
{
    Geometry::ArrayData& vdata = _data->vertexData;
    if (vdata.array.valid())
        vdata.array->dirty();
}

void Patch::Data::getGeometryAttributes(const Geometry* geom)
{
    vertexData = geom->getVertexData();
    normalData = geom->getNormalData();
    colorData = geom->getColorData();
    secondaryColorData = geom->getSecondaryColorData();
    fogCoordData = geom->getFogCoordData();
    texCoordList.clear();
    const Geometry::ArrayDataList& texList = geom->getTexCoordArrayList();
    std::copy(texList.begin(), texList.end(), std::back_inserter(texCoordList));
    vertexAttribList.clear();
    const Geometry::ArrayDataList& vaList = geom->getVertexAttribArrayList();
    std::copy(vaList.begin(), vaList.end(),
              std::back_inserter(vertexAttribList));
}

void Patch::Data::setGeometryAttributes(Geometry* geom)
{
    geom->setVertexData(vertexData);
    geom->setNormalData(normalData);
    geom->setColorData(colorData);
    geom->setSecondaryColorData(secondaryColorData);
    geom->setFogCoordData(fogCoordData);
    const Geometry::ArrayData emptyData;
    unsigned numTexCoords = geom->getNumTexCoordArrays();
    for (unsigned i = 0; i < texCoordList.size(); ++i)
        geom->setTexCoordData(i, texCoordList[i]);
    for (unsigned i = texCoordList.size(); i < numTexCoords; ++i)
        geom->setTexCoordData(i, emptyData);
    unsigned numVertAttribs = geom->getNumVertexAttribArrays();
    for (unsigned i = vertexAttribList.size(); i < vertexAttribList.size(); ++i)
        geom->setVertexAttribData(i, vertexAttribList[i]);
    for (unsigned i = vertexAttribList.size(); i < numVertAttribs; ++i)
        geom->setVertexAttribData(i, emptyData);
}

void Patch::init()
{
    for (int res = 0; res < 2; ++res)
    {
        for (int trile = 0; trile < 4; ++trile)
        {
            Geometry* geom = new Geometry;
            geom->setUseVertexBufferObjects(true);
            _data->setGeometryAttributes(geom);
            geom->addPrimitiveSet(_patchSet->trilePset[res][trile]);
            _trile[res][trile] = new Geode;
            _trile[res][trile]->addDrawable(geom);
        }
    }
    for (int j = 0; j < 4; ++j)
    {
        for (int i = 0; i < 4; ++i)
        {
            Geometry* geom = new Geometry;
            geom->setUseVertexBufferObjects(true);
            _data->setGeometryAttributes(geom);
            geom->addPrimitiveSet(_patchSet->stripPset[j][i]);
            _strip[j][i] = new Geode;
            _strip[j][i]->addDrawable(geom);
        }
    }
}

// Find the point closest to P3 on the line segment from P1 to P2
Vec3 closestPointOnSegment(const Vec3& p1, const Vec3& p2, const Vec3& p3)
{
    Vec3 vec = p2 - p1;
    float len2 = vec.length2();
    if (equivalent(len2, 0))
        return p1;
    float u = ((p3 - p1) * vec) / len2;
    if (u <= 0.0)
        return p1;
    else if (u >= 1.0)
        return p2;
    else
        return p1 + vec * u;
}

float distanceToSegment(const Vec3& p1, const Vec3& p2, const Vec3& p3)
{
    Vec3 pt = closestPointOnSegment(p1, p2, p3);
    return (p3 - pt).length();
}

namespace
{
// Map from edge number to grid coordinates at the end points. The
// values are multiplied by the patch resolution.
int edgeCoords[4][2][2] = {{{0, 0}, {1, 0}},
                           {{1, 0}, {1, 1}},
                           {{1, 1}, {0, 1}},
                           {{0, 1}, {0,0}}};

// Get the geometric coordinates of edge end points.
inline void getEdge(const Patch* patch, Vec3& p1, Vec3& p2, int edgeNo)
{
    PatchSet* ps = patch->getPatchSet();
    int psRes = ps->getResolution(); // patch dimension - 1
    int coords[2][2];
    for (int j = 0; j < 2; ++j)
        for (int i = 0; i < 2; ++i)
            coords[j][i] = edgeCoords[edgeNo][j][i] * psRes;
    Vec3Array* verts
        = static_cast<Vec3Array*>(patch->getData()->vertexData.array.get());
    p1 = (*verts)[ps->makeIndex(coords[0][0], coords[0][1])];
    p2 = (*verts)[ps->makeIndex(coords[1][0], coords[1][1])];
}
}

float Patch::getEdgeError(const Vec3& eye, int edge)
{
    Vec3 p1, p2;
    getEdge(this, p1, p2, edge);
    float len = (p2 - p1).length();
    Vec3 closestPt =  closestPointOnSegment(p1, p2, eye);
    float d = (closestPt - eye).length();
    return _patchSet->getPrecisionFactor() * len / d;
}

float Patch::getPatchError(const Vec3& eye)
{
    float epsilon = getEdgeError(eye, 0);
    for (int i = 1; i < 4; ++i)
        epsilon = maximum(epsilon, getEdgeError(eye, i));
    return epsilon;
}
void Patch::traverse(NodeVisitor& nv)
{
    if (!_trile[0][0].valid())
        return;
    if (nv.getTraversalMode() == NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
        std::for_each(&_trile[0][0], &_trile[1][3] + 1, MyNodeAcceptOp(nv));
        std::for_each(&_strip[0][0], &_strip[3][3] + 1, MyNodeAcceptOp(nv));
        return;
    }
    if (nv.getTraversalMode() != NodeVisitor::TRAVERSE_ACTIVE_CHILDREN)
        return;
    float epsilon[4];
    int res[4]; // Resolution of each edge / trile.
    // Get error value for edges
    Vec3 eye = nv.getViewPoint();
    for (int i = 0; i < 4; ++i)
    {
        epsilon[i] = getEdgeError(eye, i);
        if (epsilon[i] > _errorThreshold)
            res[i] = 1;
        else
            res[i] = 0;
    }

    for (int i = 0; i < 4; ++i)
        _trile[res[i]][i]->accept(nv);
    // Now choose a strip
    for (int i = 0; i < 4; ++i)
    {
        // One neighbor is trile i; the other is clockwise.
        int neighbor = (i - 1 + 4) % 4;
        int strip = res[neighbor] * 2 + res[i];
        _strip[strip][i]->accept(nv);
    }
}

BoundingSphere Patch::computeBound() const
{
    BoundingSphere bsphere;
    if (!_trile[0][0].valid())
        return bsphere;
    BoundingBox bb;
    bb.init();
    for (int res = 0; res < 2; ++res)
        for (int i = 0; i < 4; ++i)
            bb.expandBy(_trile[res][i]->getBoundingBox());
    for (int strip = 0; strip < 4; ++strip)
        for (int i = 0; i < 4; ++i)
            bb.expandBy(_strip[strip][i]->getBoundingBox());
    if (!bb.valid())
        return bsphere;
    bsphere.expandBy(bb);
    return bsphere;
}

void Patch::resizeGLObjectBuffers(unsigned int maxSize)
{
    Node::resizeGLObjectBuffers(maxSize);

    if (!_trile[0][0].valid())
        return;
    for (int res = 0; res < 2; ++res)
        for (int i = 0; i < 4; ++i)
            _trile[res][i]->resizeGLObjectBuffers(maxSize);
}

void Patch::releaseGLObjects(State* state) const
{
    Node::releaseGLObjects(state);
    if (!_trile[0][0].valid())
        return;
    for (int res = 0; res < 2; ++res)
        for (int i = 0; i < 4; ++i)
            _trile[res][i]->releaseGLObjects(state);
}

void Patch::setPatchSet(PatchSet* patchSet)
{
    _patchSet = patchSet;
}

PatchSet* Patch::getPatchSet() const
{
    return _patchSet.get();
}

}
