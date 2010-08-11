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
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/ThreadingUtils>

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
    void setTileKey(TileKey* key) { _tileKey = key; }
    TileKey* getTileKey() const { return _tileKey.get(); }
protected:
    ref_ptr<TileKey> _tileKey;
};

namespace
{
// Create a tile key for the map using the normalized patch
// extents. This might look a little dicey, but there is enough
// precision in the extents to recover the integer tile key
// coordinates up to a LOD of 56. That should be sufficient :)

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

HeightField* resampleHeightField(HeightField* hf, int newDim)
{
    const unsigned nCols = hf->getNumColumns();
    const unsigned nRows = hf->getNumRows();
    if (nCols == newDim && nRows == newDim)
        return hf;
    HeightField* result = new HeightField;
    result->allocate(newDim, newDim);
    result->setOrigin(hf->getOrigin());
    result->setXInterval(hf->getXInterval() * static_cast<float>(nCols)
                         / static_cast<float>(newDim));
    result->setYInterval(hf->getYInterval() * static_cast<float>(nRows)
                         / static_cast<float>(newDim));
    for (unsigned r = 0; r < newDim; ++r)
    {
        for (unsigned c = 0; c < newDim; ++c)
        {
            float height
                = HeightFieldUtils::getHeightAtNormalizedLocation(
                    hf, static_cast<double>(c) / (newDim - 1),
                    static_cast<double>(r) / (newDim - 1), INTERP_BILINEAR);
            result->setHeight(c, r, height);
        }
    }
    return result;
}
}

// Hard-wire the patch resolution and screen-space polygon size.
Projected::Projected(Map* map)
    : PatchSet(64), _map(map)
{
    setPrecisionFactor(8);
    const MapLayerList& heightList = _map->getHeightFieldMapLayers();
    {
        int maxLevel = 0;
        Threading::ScopedReadLock lock(_map->getMapDataMutex());
        for (MapLayerList::const_iterator itr = heightList.begin(),
                 end = heightList.end();
             itr != end;
             ++itr)
            if ((*itr)->maxLevel().isSet()
                && (*itr)->maxLevel().get() > maxLevel)
                maxLevel = (*itr)->maxLevel().get();
        if (maxLevel > 0)
            setMaxLevel(maxLevel);
    }
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
    hf = resampleHeightField(hf, patchDim);
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
