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

#include "GeoPatch"

#include <algorithm>

#include "Geographic"
#include "Euler"
#include "PatchSet"

namespace seamless
{
using namespace osg;
using namespace osgEarth;

GeoPatch::GeoPatch()
{
    _face = -1;
    std::fill(&_edgeLengths[0], &_edgeLengths[4], 0.0);

}

GeoPatch::GeoPatch(const TileKey& key)
{
    // When an arc on the cube grid is subdivided, this is the largest
    // ratio of the lengths of the parent arc and its longest
    // child. The ratio goes to .5 as the arcs are further subdivided
    // at higher LODs. The error threshold is set to this value to
    // insure that the triles that share an edge will display the same
    // LOD when the enclosing patches are from different LODs.
    setErrorThreshold(.5371);
    const GeoExtent& extent = key.getExtent();
    double xMin = extent.xMin(), yMin = extent.yMin(),
        xMax = extent.xMax(), yMax = extent.yMax();
    euler::cubeToFace(xMin, yMin, xMax, yMax, _face);
    Vec2d faceCoords[4];
    _faceCoords[0][0] = xMin; _faceCoords[0][1] = yMin;
    _faceCoords[1][0] = xMax; _faceCoords[1][1] = yMin;
    _faceCoords[2][0] = xMax; _faceCoords[2][1] = yMax;
    _faceCoords[3][0] = xMin; _faceCoords[3][1] = yMax;
    for (int i = 0; i < 4; ++i)
        _edgeLengths[i] = euler::arcLength(_faceCoords[i],
                                           _faceCoords[(i + 1) % 4], _face);
}

GeoPatch::GeoPatch(const GeoPatch& rhs, const CopyOp& copyop)
    : Patch(rhs, copyop), _face(rhs._face)
{
    std::copy(&rhs._edgeLengths[0], &rhs._edgeLengths[4], &_edgeLengths[0]);
    std::copy(&rhs._faceCoords[0], &rhs._faceCoords[4], &_faceCoords[0]);
}

float GeoPatch::getEdgeError(const osg::Vec3& eye, int edge)
{
    // Hack to get back to face parameters and world coordinates.
    Transform* parent = static_cast<Transform*>(getParent(0));
    PatchGroup* pgroup = static_cast<PatchGroup*>(parent->getParent(0));
    Matrix worldMat;
    parent->computeLocalToWorldMatrix(worldMat, 0);
    Vec3d worldEye = Vec3d(eye) * worldMat;
    double d = euler::distanceToSegment(worldEye, _faceCoords[edge],
                                        _faceCoords[(edge + 1) % 4], _face);
    return _patchSet->getPrecisionFactor() * _edgeLengths[edge] / d;
}

void GeoPatch::setGeographic(Geographic* geo)
{
    _patchSet = geo;
}

Geographic* GeoPatch::getGeographic() const
{
    return static_cast<Geographic*>(_patchSet.get());
}
}
