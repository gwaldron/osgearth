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

#include <seamless/PatchSet>

namespace seamless
{
using namespace osg;
//using namespace osgEarth;

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
    PatchSet* ps = getPatchSet();
    int psRes = ps->getResolution(); // patch dimension - 1
    int coords[2][2];
    for (unsigned j = 0; j < 2; ++j)
    {
        unsigned vert = (edge + j) % 4;
        for (unsigned i = 0; i < 2; ++i)
            coords[j][i] = vertCoords[j][i] * psRes;
    }
    Vec3Array* verts
        = static_cast<Vec3Array*>(getData()->vertexData.array.get());
    Vec3 p1 = (*verts)[ps->makeIndex(coords[0][0], coords[0][1])];
    Vec3 p2 = (*verts)[ps->makeIndex(coords[1][0], coords[1][1])];
#if 0
    // Massive hack.
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
    int face = QscProfile::getFace(tk);
    const GeoExtent& extent = tk->getGeoExtent();
    Vec2d faceCoords[4];
    faceCoords[0][0] = extent.xMin();  faceCoords[0][1] = extent.yMin();
    faceCoords[1][0] = extent.xMax(); faceCoords[1][1] = extent.yMin();
    faceCoords[2][0] = extent.xMax(); faceCoords[2][1] = extent.yMax();
    faceCoords[3][0] = extent.xMin(); faceCoords[3][1] = extent.yMin();
    Vec2d edgeCoords[2];
    for (unsigned j = 0; j < 2; ++j)
    {
        unsigned vert = (edge + j) % 4;
        p[j] = faceCoords[vert];
    }
    // Finally, faceCoords to the cube
#endif    
    float len = (p2 - p1).length();
    Vec3 closestPt =  closestPointOnSegment(p1, p2, eye);
    float d = (closestPt - eye).length();
    return _patchSet->getPrecisionFactor() * _edgeLength / d;
}
      
}
