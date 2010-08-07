#include "Patch"

#include <algorithm>
#include <iterator>

#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include <osg/Vec3d>

namespace teng
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
{
    // Silly default value assuming a 60 degree field of view, laptop
    // screen (.36 meter near plane), 90 dpi, and max screen size of
    // our highest resolution polygon  of 4 pixels.
    _precisionFactor = .36 * (90.0 / .0254) / (4 * resolution);
}

Patch::Patch(const Patch& rhs, const CopyOp& copyop)
    : Node(rhs, copyop), _precisionFactor(rhs._precisionFactor)
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

namespace
{
inline unsigned short makeIndexCoord(int x, int y)
{
    return static_cast<unsigned short>(y * (Patch::resolution + 1) + x);
}

inline void getGridCoords(unsigned short index, int& x, int& y)
{
    y = index / (Patch::resolution + 1);
    x = index % (Patch::resolution + 1);
}

ref_ptr<DrawElementsUShort> makeBasicTrile(int delta)
{
    ref_ptr<DrawElementsUShort> pset = new DrawElementsUShort(GL_TRIANGLES);
    // y axis
    int xBegin = delta;
    int xEnd = Patch::resolution - delta;
    for (int j = 0; j < Patch::resolution / 2 - delta; j += delta)
    {
        for (int i = xBegin; i < xEnd; i += 2 * delta)
        {
            pset->push_back(makeIndexCoord(i, j));
            pset->push_back(makeIndexCoord(i + delta, j));
            pset->push_back(makeIndexCoord(i + delta, j + delta));
            pset->push_back(makeIndexCoord(i + delta, j + delta));
            pset->push_back(makeIndexCoord(i + delta, j));
            pset->push_back(makeIndexCoord(i + 2 * delta, j));
            if (i + 2 * delta == xEnd)
                break;
            pset->push_back(makeIndexCoord(i + delta, j + delta));
            pset->push_back(makeIndexCoord(i + 2 * delta, j));
            pset->push_back(makeIndexCoord(i + 2 * delta, j + delta));
            pset->push_back(makeIndexCoord(i + 2 * delta, j + delta));
            pset->push_back(makeIndexCoord(i + 2 * delta, j));
            pset->push_back(makeIndexCoord(i + 3 * delta, j + delta));
        }
        xBegin += delta;
        xEnd -= delta;
    }
    return pset;
}

// Shared primitive sets for all the geometry.
OpenThreads::Mutex psetMutex;
ref_ptr<DrawElementsUShort> trilePset[2][4];
ref_ptr<DrawElementsUShort> stripPset[4][4];

// Rotate grid indices 90 deg counter-clockwise.
unsigned short rotateIndex(unsigned short index)
{
    int x, y;
    getGridCoords(index, x, y);
    x -= Patch::resolution / 2;
    y -= Patch::resolution / 2;
    int newx = -y;
    int newy = x;
    newx += Patch::resolution / 2;
    newy += Patch::resolution / 2;
    return makeIndexCoord(newx, newy);
}

// Stitching  strip between two triles of the same resolution.

ref_ptr<DrawElementsUShort> makeSingleStrip(int delta)
{
    ref_ptr<DrawElementsUShort> pset = new DrawElementsUShort(GL_TRIANGLES);
    for (int i = 0; i < Patch::resolution / 2; i += delta)
    {
        if (i > 0)
        {
            pset->push_back(makeIndexCoord(i - delta, i));
            pset->push_back(makeIndexCoord(i, i));
            pset->push_back(makeIndexCoord(i, i + delta));
        }
        pset->push_back(makeIndexCoord(i, i));
        pset->push_back(makeIndexCoord(i + delta, i + delta));
        pset->push_back(makeIndexCoord(i, i + delta));
        pset->push_back(makeIndexCoord(i, i));
        pset->push_back(makeIndexCoord(i + delta, i));
        pset->push_back(makeIndexCoord(i + delta, i + delta));
        if (i < Patch::resolution / 2 - delta)
        {
            pset->push_back(makeIndexCoord(i + delta, i + delta));
            pset->push_back(makeIndexCoord(i + delta, i));
            pset->push_back(makeIndexCoord(i + 2 * delta, i + delta));
        }
    }
    return pset;
}

// Stitching strip between triles of different resolutions.

ref_ptr<DrawElementsUShort> makeDualStrip()
{
    ref_ptr<DrawElementsUShort> pset = new DrawElementsUShort(GL_TRIANGLES);
    for (int i = 0, j = 2; j <= Patch::resolution / 2; i += 2, j += 2)
    {
        pset->push_back(makeIndexCoord(i, j));
        if (i == 0)
            pset->push_back(makeIndexCoord(0, 0));
        else
            pset->push_back(makeIndexCoord(i - 2, j -2));
        pset->push_back(makeIndexCoord(i + 1, j - 2));
        pset->push_back(makeIndexCoord(i, j));
        pset->push_back(makeIndexCoord(i + 1, j - 2));
        pset->push_back(makeIndexCoord(i + 2, j - 1));

        pset->push_back(makeIndexCoord(i, j));
        pset->push_back(makeIndexCoord(i + 2, j - 1));
        pset->push_back(makeIndexCoord(i + 3, j));
    }
    return pset;
}

void initPrimitiveSets()
{
    for (int res = 0; res < 2; res++)
    {
        // Bottom trile
        trilePset[res][0] = makeBasicTrile(2 - res);
        // The other triles are rotations of the first on the grid of
        // coordinate indices.
        for (int i = 1; i < 4; ++i)
        {
            trilePset[res][i] = new DrawElementsUShort(GL_TRIANGLES);
            for (DrawElementsUShort::iterator itr = trilePset[res][i - 1]->begin(),
                     end = trilePset[res][i - 1]->end();
                 itr != end;
                ++itr)
                trilePset[res][i]->push_back(rotateIndex(*itr));
        }
    }
    // First, the strips for the edge from lower left to middle
    stripPset[0][0] = makeSingleStrip(2); // low res
    stripPset[1][0] = makeDualStrip();
    // The other dual strip is the reflection across y = x
    stripPset[2][0] = new DrawElementsUShort(GL_TRIANGLES);
    for (DrawElementsUShort::iterator itr = stripPset[1][0]->begin(),
             end = stripPset[1][0]->end();
         itr != end;
        ++itr)
    {
        int x, y;
        getGridCoords(*itr, x, y);
        stripPset[2][0]->push_back(makeIndexCoord(y, x));
    }
    stripPset[3][0] = makeSingleStrip(1); // hi res
    // Now rotate the strips for the other diagonals.
    for (int j = 1; j < 4; ++j)
    {
        for (int i = 0; i < 4; ++i)
        {
            stripPset[i][j] = new DrawElementsUShort(GL_TRIANGLES);
            for (DrawElementsUShort::iterator itr = stripPset[i][j - 1]->begin(),
                     end = stripPset[i][j - 1]->end();
                 itr != end;
                ++itr)
                stripPset[i][j]->push_back(rotateIndex(*itr));
        }
    }
}
}

void Patch::init()
{
    Geometry* trileGeom[2][4];
    Geometry* stripGeom[4][4];
    for (Geometry** pgeom = &trileGeom[0][0]; pgeom <= &trileGeom[1][3]; ++pgeom)
    {
        *pgeom = new Geometry;
        _data->setGeometryAttributes(*pgeom);
    }
    for (Geometry** pgeom = &stripGeom[0][0]; pgeom <= &stripGeom[3][3]; ++pgeom)
    {
        *pgeom = new Geometry;
        _data->setGeometryAttributes(*pgeom);
    }
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(psetMutex);
        if (!trilePset[0][0].valid())
            initPrimitiveSets();
        for (int res = 0; res < 2; ++res)
        {
            for (int trile = 0; trile < 4; ++trile)
                trileGeom[res][trile]->addPrimitiveSet(trilePset[res][trile]);
        }
        for (int j = 0; j < 4; ++j)
        {
            for (int i = 0; i < 4; ++i)
                stripGeom[j][i]->addPrimitiveSet(stripPset[j][i]);
        }
    }
    for (int res = 0; res < 2; ++res)
    {
        for (int trile = 0; trile < 4; ++trile)
        {
            _trile[res][trile] = new Geode;
            _trile[res][trile]->addDrawable(trileGeom[res][trile]);
        }
    }
    for (int j = 0; j < 4; ++j)
    {
        for (int i = 0; i < 4; ++i)
        {
            _strip[j][i] = new Geode;
            _strip[j][i]->addDrawable(stripGeom[j][i]);
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

int edgeCoords[4][2][2] = {{{0, 0}, {Patch::resolution, 0}},
                           {{Patch::resolution, 0},
                            {Patch::resolution, Patch::resolution}},
                           {{Patch::resolution, Patch::resolution},
                            {0, Patch::resolution}},
                           {{0, Patch::resolution}, {0,0}}};

float Patch::getEdgeError(const Vec3& eye, int edge)
{
    Vec3Array* verts = static_cast<Vec3Array*>(_data->vertexData.array.get());
    const Vec3& p1 = (*verts)[makeIndexCoord(edgeCoords[edge][0][0],
                                             edgeCoords[edge][0][1])];
    const Vec3& p2 = (*verts)[makeIndexCoord(edgeCoords[edge][1][0],
                                             edgeCoords[edge][1][1])];
    float len = (p2 - p1).length();
    Vec3 closestPt =  closestPointOnSegment(p1, p2, eye);
    float d = (closestPt - eye).length();
    return _precisionFactor * len / d;
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
    Vec3Array* verts = static_cast<Vec3Array*>(_data->vertexData.array.get());
    float epsilon[4];
    int res[4]; // Resolution of each edge / trile.
    // Get error value for edges
    Vec3 eye = nv.getEyePoint();
    for (int i = 0; i < 4; ++i)
    {
        epsilon[i] = getEdgeError(eye, i);
        if (epsilon[i] > .5f)
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
            bb.expandBy(_trile[res][i]->getBound());
    if (!bb.valid())
        return bsphere;
    bsphere._center = bb.center();
    bsphere._radius = 0.0f;
    for (int res = 0; res < 2; ++res)
        for (int i = 0; i < 4; ++i)
            bsphere.expandRadiusBy(_trile[res][i]->getBound());
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
}
