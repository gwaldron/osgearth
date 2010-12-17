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

#include <algorithm>

#include "PatchSet"

#include <osg/Math>
#include <osg/MatrixTransform>

#include "Patch"
#include "PatchGroup"

namespace seamless
{
using namespace osg;
using namespace osgEarth;

PatchSet::PatchSet()
    : _maxLevel(16), _patchOptionsPrototype(new PatchOptions), _mapf(0),
      _resolution(128), _verticalScale(1.0f)
{
    setPrecisionFactor(4);
    initPrimitiveSets();

}

PatchSet::PatchSet(const Drivers::SeamlessOptions& options,
                   PatchOptions* poptionsPrototype)
    :  _maxLevel(16),
       _patchOptionsPrototype(poptionsPrototype ? poptionsPrototype
                              : new PatchOptions),
       _mapf(0), _options(options)
{
    _resolution = options.resolution().value();
    _verticalScale = options.verticalScale().value();
    setPrecisionFactor(4);
    initPrimitiveSets();
}

PatchSet::PatchSet(const PatchSet& rhs, const CopyOp& copyop)
    : _precisionFactor(rhs._precisionFactor), _resolution(rhs._resolution),
      _maxLevel(rhs._maxLevel), _verticalScale(rhs._verticalScale),
      _patchOptionsPrototype(static_cast<PatchOptions*>(copyop(rhs._patchOptionsPrototype.get()))),
      _map(static_cast<Map*>(copyop(rhs._map.get())))
{
    _patchOptionsPrototype
        = static_cast<PatchOptions*>(copyop(_patchOptionsPrototype.get()));
    for (int j = 0; j < 2; ++j)
        for (int i = 0; j < 4; ++i)
            trilePset[j][i]
                = static_cast<DrawElementsUShort*>(copyop(rhs.trilePset[j][i].get()));
    for (int j = 0; j < 4; ++j)
        for (int i = 0; j < 4; ++i)
            stripPset[j][i]
                = static_cast<DrawElementsUShort*>(copyop(rhs.stripPset[j][i].get()));
    if (rhs._mapf)
        _mapf = new MapFrame(*_mapf);
}

PatchSet::~PatchSet()
{
    delete _mapf;
}

void PatchSet::setMap(const Map* map)
{
    _map = map;
    if (map)
    {
        delete _mapf;
        _mapf = new MapFrame(map, Map::TERRAIN_LAYERS, "seamless");
    }
}

Node* PatchSet::createPatchGroup(const std::string& filename,
                                 PatchOptions* poptions)
{
    PatchGroup* pgroup = new PatchGroup;
    pgroup->setOptions(poptions);
    Transform* patch = createPatch(filename, poptions);
    BoundingSphere bsphere = patch->getBound();
    pgroup->setCenter(bsphere.center());
    if (poptions->getPatchLevel() >= _maxLevel)
    {
        pgroup->addChild(patch, 0.0, 1e10);
    }
    else
    {
        pgroup->addChild(patch, 0.0, 1.0);
        pgroup->setRange(1, 1.0, 1e10);
        pgroup->setFileName(1, "foo.osgearth_engine_seamless_patch");
    }
    return pgroup;
}

// Default implementation that creates a flat 81920m x 81920m plane.

Transform* PatchSet::createPatch(const std::string& filename, PatchOptions* poptions)
{
    Patch* patch = new Patch;
    patch->setPatchSet(this);
    Vec2d ll, ur;
    poptions->getPatchExtents(ll, ur);
    Vec2d range = (ur - ll);
    ref_ptr<Patch::Data> data = new Patch::Data;
    int patchDim = _resolution + 1;
    Vec3Array* verts = new Vec3Array(patchDim * patchDim);
    for (int j = 0; j < patchDim; ++j)
        for (int i = 0; i < patchDim; ++i)
            (*verts)[patchDim * j + i]
                = Vec3((ll.x() + i * range.x()
                        / static_cast<float>(_resolution)) * 81920.0,
                       (ll.y() + j * range.y()
                        / static_cast<float>(_resolution)) * 81920.0,
                       0.0);
    data->vertexData.array = verts;
    data->vertexData.binding = Geometry::BIND_PER_VERTEX;
    Vec3Array* norms = new Vec3Array(1);
    (*norms)[0] = Vec3d(0.0, 0.0, 1.0);
    data->normalData.array = norms;
    data->normalData.binding = Geometry::BIND_OVERALL;
    Vec4Array* colors = new Vec4Array(1);
    (*colors)[0] = Vec4(1.0, 1.0, 1.0, 1.0);
    data->colorData.array = colors;
    data->colorData.binding = Geometry::BIND_OVERALL;
    patch->setData(data);
    MatrixTransform* transform = new MatrixTransform;
    transform->addChild(patch);
    return transform;
}

Node* PatchSet::createPatchSetGraph(const std::string& filename)
{
    PatchOptions* poptions = osg::clone(_patchOptionsPrototype.get());
    poptions->setPatchSet(this);
    return createPatchGroup(filename, poptions);
}

double PatchSet::calcPrecisionFactor(int pixelError, double horiz_fov_deg,
                               int screenRes, int dpi)
{
    // Find near plane distance in meters
    const double pixelsPerMeter = dpi / .0254;
    const double halfScreen = .5 * screenRes / pixelsPerMeter;
    const double dNear = halfScreen / tan(DegreesToRadians(horiz_fov_deg / 2));
    return dNear * pixelsPerMeter / (pixelError * _resolution);
}

namespace
{
inline void getGridCoords(int psRes, unsigned short index, int& x, int& y)
{
    y = index / (psRes + 1);
    x = index % (psRes + 1);
}
}

ref_ptr<DrawElementsUShort> PatchSet::makeBasicTrile(int delta)
{
    ref_ptr<DrawElementsUShort> pset = new DrawElementsUShort(GL_TRIANGLES);
    // y axis
    int xBegin = delta;
    int xEnd = _resolution - delta;
    for (int j = 0; j < _resolution / 2 - delta; j += delta)
    {
        for (int i = xBegin; i < xEnd; i += 2 * delta)
        {
            pset->push_back(makeIndex(i, j));
            pset->push_back(makeIndex(i + delta, j));
            pset->push_back(makeIndex(i + delta, j + delta));
            pset->push_back(makeIndex(i + delta, j + delta));
            pset->push_back(makeIndex(i + delta, j));
            pset->push_back(makeIndex(i + 2 * delta, j));
            if (i + 2 * delta == xEnd)
                break;
            pset->push_back(makeIndex(i + delta, j + delta));
            pset->push_back(makeIndex(i + 2 * delta, j));
            pset->push_back(makeIndex(i + 2 * delta, j + delta));
            pset->push_back(makeIndex(i + 2 * delta, j + delta));
            pset->push_back(makeIndex(i + 2 * delta, j));
            pset->push_back(makeIndex(i + 3 * delta, j + delta));
        }
        xBegin += delta;
        xEnd -= delta;
    }
    return pset;
}

// Rotate grid indices 90 deg counter-clockwise.
unsigned short PatchSet::rotateIndex(unsigned short index)
{
    int x, y;
    int psRes = _resolution;
    getGridCoords(psRes, index, x, y);
    x -= psRes / 2;
    y -= psRes / 2;
    int newx = -y;
    int newy = x;
    newx += psRes / 2;
    newy += psRes / 2;
    return makeIndex(newx, newy);
}

// Stitching  strip between two triles of the same resolution.

ref_ptr<DrawElementsUShort> PatchSet::makeSingleStrip(int delta)
{
    ref_ptr<DrawElementsUShort> pset = new DrawElementsUShort(GL_TRIANGLES);
    for (int i = 0; i < _resolution / 2; i += delta)
    {
        if (i > 0)
        {
            pset->push_back(makeIndex(i - delta, i));
            pset->push_back(makeIndex(i, i));
            pset->push_back(makeIndex(i, i + delta));
        }
        pset->push_back(makeIndex(i, i));
        pset->push_back(makeIndex(i + delta, i + delta));
        pset->push_back(makeIndex(i, i + delta));
        pset->push_back(makeIndex(i, i));
        pset->push_back(makeIndex(i + delta, i));
        pset->push_back(makeIndex(i + delta, i + delta));
        if (i < _resolution / 2 - delta)
        {
            pset->push_back(makeIndex(i + delta, i + delta));
            pset->push_back(makeIndex(i + delta, i));
            pset->push_back(makeIndex(i + 2 * delta, i + delta));
        }
    }
    return pset;
}

// Stitching strip between triles of different resolutions.

ref_ptr<DrawElementsUShort> PatchSet::makeDualStrip()
{
    ref_ptr<DrawElementsUShort> pset = new DrawElementsUShort(GL_TRIANGLES);
    for (int i = 0, j = 2; j <= _resolution / 2; i += 2, j += 2)
    {
        pset->push_back(makeIndex(i, j));
        if (i == 0)
            pset->push_back(makeIndex(0, 0));
        else
            pset->push_back(makeIndex(i - 2, j -2));
        pset->push_back(makeIndex(i + 1, j - 2));
        pset->push_back(makeIndex(i, j));
        pset->push_back(makeIndex(i + 1, j - 2));
        pset->push_back(makeIndex(i + 2, j - 1));

        pset->push_back(makeIndex(i, j));
        pset->push_back(makeIndex(i + 2, j - 1));
        pset->push_back(makeIndex(i + 3, j));
    }
    return pset;
}

void PatchSet::initPrimitiveSets()
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
        getGridCoords(_resolution, *itr, x, y);
        stripPset[2][0]->push_back(makeIndex(y, x));
    }
    // Now switch the order on the triangles on the reflected strip
    for (size_t i = 1; i < stripPset[2][0]->size(); i += 3)
    {
        std::swap((*stripPset[2][0].get())[i],
                  (*stripPset[2][0].get())[i + 1]);
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

osg::Node* PatchSet::createChild(const PatchOptions* parentOptions, int childNum)
{
    Vec2d lowerLeft(0.0, 1.0);
    Vec2d upperRight(1.0, 1.0);

    parentOptions->getPatchExtents(lowerLeft, upperRight);
    Vec2d range = upperRight - lowerLeft;
    Vec2d newRange = range * .5;
    double x = (childNum % 2) * .5;
    double y = (childNum / 2) * .5;
    PatchOptions* pgroupOptions = osg::clone(parentOptions);
    Vec2d ll = lowerLeft + componentMultiply(Vec2d(x, y), range);
    pgroupOptions->setPatchExtents(ll, ll + newRange);
    pgroupOptions->setPatchLevel(parentOptions->getPatchLevel() + 1);
    Node* pgroup = createPatchGroup("foobies.osgearth_engine_seamless_patch",
                                    pgroupOptions);
    return pgroup;
}
}
