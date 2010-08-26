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

#include <seamless/GeoPatch>

#include <seamless/Geographic>
#include <seamless/Euler>
#include <seamless/PatchSet>

namespace seamless
{
using namespace osg;
using namespace osgEarth;

GeoPatch::GeoPatch()
    : _edgeLength(0.0)
{

}

GeoPatch::GeoPatch(const GeoPatch& rhs, const CopyOp& copyop)
    : Patch(rhs, copyop), _edgeLength(rhs._edgeLength)
{
}

namespace
{
// Map from vertex number to grid coordinates. An edge is then defined
// as being from a vertex to vertex + 1 mod 4.
int vertCoords[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
}

float GeoPatch::getEdgeError(const osg::Vec3& eye, int edge)
{
    // Massive hack to get back to face parameters.
    MatrixList worldmats = getWorldMatrices();
    Vec3d worldEye = Vec3d(eye) * worldmats[0];
    Group* parent = getParent(0);
    PatchGroup* pgroup = dynamic_cast<PatchGroup*>(parent->getParent(0));
    if (!pgroup)
    {
        OE_FATAL << "can't find parent!\n";
        return 0;
    }
    GeographicOptions* goptions
        = dynamic_cast<GeographicOptions*>(pgroup->getDatabaseOptions());
    TileKey* tk = goptions->getTileKey();
    int face;
    const GeoExtent& extent = tk->getGeoExtent();
    double xMin = extent.xMin(), yMin = extent.yMin(),
        xMax = extent.xMax(), yMax = extent.yMax();
    euler::cubeToFace(xMin, yMin, xMax, yMax, face);
    Vec2d faceCoords[4];
    faceCoords[0][0] = xMin; faceCoords[0][1] = yMin;
    faceCoords[1][0] = xMax; faceCoords[1][1] = yMin;
    faceCoords[2][0] = xMax; faceCoords[2][1] = yMax;
    faceCoords[3][0] = xMin; faceCoords[3][1] = yMax;
    Vec2d edgeCoords[2];
    for (unsigned j = 0; j < 2; ++j)
    {
        unsigned vert = (edge + j) % 4;
        edgeCoords[j] = faceCoords[vert];
    }
    double len = euler::arcLength(edgeCoords[0], edgeCoords[1], face);
    double d = euler::distanceToSegment(worldEye, edgeCoords[0], edgeCoords[1],
                                         face);
    return _patchSet->getPrecisionFactor() * len / d;
}

}
