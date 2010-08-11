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

#include <seamless/Projected>

#include <string>

#include <osg/Math>

#include <osgEarth/GeoData>

namespace seamless
{
using namespace std;
using namespace osg;
using namespace osgEarth;

class ProjectedOptions : public PatchOptions
{
public:
    ProjectedOptions()
    {
    }
    ProjectedOptions(string& str)
    {
    }
    ProjectedOptions(const ProjectedOptions& rhs,
                     const CopyOp& copyop = CopyOp::SHALLOW_COPY)
    {
    }
    void setGeoExtent(const GeoExtent& geoExtent) { _geoExtent = geoExtent; }
    GeoExtent getGeoExtent() const { return _geoExtent; }
    void setTileKey(TileKey* key) { _tileKey = key; }
    TileKey* getTileKey() const { return _tileKey.get(); }
protected:
    GeoExtent _geoExtent;
    ref_ptr<TileKey> _tileKey;
};

namespace
{
TileKey* makeTileKey(const Projected* ps, const ProjectedOptions* pjoptions)
{
    Vec2d ll, ur;
    pjoptions->getPatchExtents(ll, ur);
    double levelFactor = pow(2.0, pjoptions->getPatchLevel());
    int x = static_cast<int>(ll.x() * levelFactor);
    int y = static_cast<int>(ll.y() * levelFactor);
    return new TileKey(pjoptions->getPatchLevel(), x, y,
                       ps->getMap()->getProfile());
}
}

Projected::Projected(Map* map)
    : PatchSet(64), _map(map)
{
    setPrecisionFactor(8);
}

void Projected::fillPatch(const std::string& filename, Patch* patch,
                          PatchOptions* poptions)
{
    ProjectedOptions* pjoptions = static_cast<ProjectedOptions*>(poptions);
    TileKey* key
        = makeTileKey(static_cast<Projected*>(patch->getPatchSet()), pjoptions);
    ref_ptr<HeightField> hf = _map->createHeightField(key, true,
                                                      INTERP_BILINEAR);
    ref_ptr<Patch::Data> data = new Patch::Data;
    int patchDim = _resolution + 1;
    if (patchDim != hf->getNumColumns() || patchDim != hf->getNumColumns())
    {
        OSG_FATAL << "patch size " << patchDim << "is not equal to height field dimensions " << hf->getNumColumns() << "x" << hf->getNumRows() << "\n";
        return;
    }
    Vec3Array* verts = new Vec3Array(patchDim * patchDim);
    Vec3Array* normals = new Vec3Array(patchDim * patchDim);
    for (int j = 0; j < patchDim; ++j)
        for (int i = 0; i < patchDim; ++i)
        {
            (*verts)[patchDim * j + i] = hf->getVertex(i, j);
            (*normals)[patchDim * j + i] = hf->getNormal(i, j);
        }
    data->vertexData.array = verts;
    data->vertexData.binding = Geometry::BIND_PER_VERTEX;
    data->normalData.array = normals;
    data->normalData.binding = Geometry::BIND_PER_VERTEX;
        Vec4Array* colors = new Vec4Array(1);
    (*colors)[0] = Vec4(1.0, 1.0, 1.0, 1.0);
    data->colorData.array = colors;
    data->colorData.binding = Geometry::BIND_OVERALL;
    patch->setData(data);
}
}
