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

#include "Projected"

#include <string>

#include <osg/Math>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Texture2D>

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
        : PatchOptions(str)
    {
    }
    ProjectedOptions(const ProjectedOptions& rhs,
                     const CopyOp& copyop = CopyOp::SHALLOW_COPY)
        : PatchOptions(rhs, copyop)
    {

    }
};

namespace
{
// Create a tile key for the map using the normalized patch
// extents. This might look a little dicey, but there is enough
// precision in the extents to recover the integer tile key
// coordinates up to a LOD of 56. That should be sufficient :)

TileKey makeTileKey(const Projected* ps, const ProjectedOptions* pjoptions)
{
    Vec2d ll, ur;
    pjoptions->getPatchExtents(ll, ur);
    double levelFactor = pow(2.0, pjoptions->getPatchLevel());
    int x = static_cast<int>(ll.x() * levelFactor);
    int y = static_cast<int>(ll.y() * levelFactor);
    return TileKey(pjoptions->getPatchLevel(), x, y,
                   ps->getMap()->getProfile());
}

HeightField* resampleHeightField(HeightField* hf, unsigned newDim)
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

// Hard-wire the screen-space polygon size.
Projected::Projected(const Map* map,
                     const osgEarth::Drivers::SeamlessOptions& options)
    : PatchSet(options)
{
    setPrecisionFactor(8);
    setMap(map);
    {
        int maxLevel = 0;
        const ElevationLayerVector& elevations = _mapf->elevationLayers();
        for (ElevationLayerVector::const_iterator itr = elevations.begin(),
                 end = elevations.end();
             itr != end;
             ++itr)
        {
            const TerrainLayerOptions& options
                = (*itr)->getTerrainLayerOptions();
            if (options.maxLevel().isSet()
                && options.maxLevel().get() > maxLevel)
                maxLevel = options.maxLevel().get();
        }
        if (maxLevel > 0)
            setMaxLevel(maxLevel);
    }
}

Transform* Projected::createPatch(const std::string& filename,
                                  PatchOptions* poptions)
{
    Patch* patch = new Patch;
    patch->setPatchSet(this);
    ProjectedOptions* pjoptions = static_cast<ProjectedOptions*>(poptions);
    TileKey key
        = makeTileKey(static_cast<Projected*>(patch->getPatchSet()), pjoptions);
    const GeoExtent& extent = key.getExtent();
    double xMin = extent.xMin(), yMin = extent.yMin();
    double centerX, centerY;
    extent.getCentroid(centerX, centerY);
    MatrixTransform* transform = new MatrixTransform;
    Matrixd mat = Matrixd::translate(centerX, centerY, 0.0);
    transform->setMatrix(mat);
    transform->addChild(patch);
    ref_ptr<HeightField> hf;
    GeoImage gimage;
    {
        _mapf->getHeightField(key, true, hf, 0L, INTERP_BILINEAR);
        const ImageLayerVector& layers = _mapf->imageLayers();
        if (!layers.empty())
            gimage = layers[0]->createImage(key);
    }
    ref_ptr<Patch::Data> data = new Patch::Data;
    int patchDim = _resolution + 1;
    hf = resampleHeightField(hf, patchDim);
    Vec3Array* verts = new Vec3Array(patchDim * patchDim);
    Vec3Array* normals = new Vec3Array(patchDim * patchDim);
    Vec2f minCoord(xMin - centerX, yMin - centerY);
    float xInt = hf->getXInterval(), yInt = hf->getYInterval();
    for (int j = 0; j < patchDim; ++j)
        for (int i = 0; i < patchDim; ++i)
        {
            (*verts)[patchDim * j + i] = Vec3(
                minCoord.x() + xInt * i, minCoord.y() + yInt * j,
                hf->getHeight(i, j) * getVerticalScale());
            // XXX normals change if verticalScale != 1.0
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
    if (gimage.valid())
    {
        Texture2D* tex = new Texture2D();
        tex->setImage(gimage.getImage());
        tex->setWrap(Texture::WRAP_S, Texture::CLAMP_TO_EDGE);
        tex->setWrap(Texture::WRAP_T, Texture::CLAMP_TO_EDGE);
        tex->setFilter(Texture::MIN_FILTER, Texture::LINEAR_MIPMAP_LINEAR);
        tex->setFilter(Texture::MAG_FILTER, Texture::LINEAR);
        StateSet* ss = patch->getOrCreateStateSet();
        ss->setTextureAttributeAndModes(0, tex, StateAttribute::ON);
    }
    Vec2Array* texCoords = new Vec2Array(patchDim * patchDim);
    for (int j = 0; j < patchDim; ++j)
        for (int i = 0; i < patchDim; ++i)
            (*texCoords)[patchDim * j + i]
                = Vec2(static_cast<float>(i) / (patchDim - 1),
                       static_cast<float>(j) / (patchDim - 1));
    data->texCoordList
        .push_back(Geometry::ArrayData(texCoords, Geometry::BIND_PER_VERTEX));
    patch->setData(data);
    return transform;
}
}
