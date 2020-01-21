/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "CreateTileImplementation"
#include "EngineContext"
#include <osgEarth/TerrainTileModel>
#include <osgEarth/MaskLayer>
#include <osgEarth/TileKey>
#include <osgEarth/Locators>
#include <osg/Node>
#include <osg/ValueObject>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::REX;

#undef LC
#define LC "[REX::CreateTileImpl] "

namespace
{
    struct MinMax {
        osg::Vec3d min, max;
    };
}


osg::Node*
CreateTileImplementation::createTile(
    EngineContext* context,
    const TerrainTileModel* model,
    int flags,
    unsigned referenceLOD,
    const TileKey& area)
{
    if (model == 0L)
    {
        OE_WARN << LC << "Illegal: createTile(NULL)" << std::endl;
        return 0L;
    }

    // Verify that we have a map:
    osg::ref_ptr<const Map> map = context->getMap();
    if(!map.valid())
    {
        return 0L;
    }

    // Dimension of each tile in vertices
    unsigned tileSize = context->options().tileSize().get();

    bool includeTilesWithMasks = (flags & TerrainEngineNode::CREATE_TILE_INCLUDE_TILES_WITH_MASKS) != 0;
    bool includeTilesWithoutMasks = (flags & TerrainEngineNode::CREATE_TILE_INCLUDE_TILES_WITHOUT_MASKS) != 0;

    TileKey rootkey = area.valid() ? area : model->getKey();
    const SpatialReference* srs = rootkey.getExtent().getSRS();

    // Find the axis aligned bounding box of the mask boundary for each layer
    MaskLayerVector maskLayers;
    std::vector<MinMax> boundaryMinMaxes;

    context->getMap()->getLayers(maskLayers);

    for (MaskLayerVector::iterator iLayer = maskLayers.begin(); iLayer != maskLayers.end(); ++iLayer)
    {
        MaskLayer* layer = iLayer->get();
        osg::Vec3dArray* boundary = layer->getOrCreateMaskBoundary(1.0, srs, (ProgressCallback*)0L);

        if (!boundary)
            continue;

        // Calculate the axis-aligned bounding box of the boundary polygon:
        MinMax minmax;
        minmax.min = minmax.max = boundary->front();

        for (osg::Vec3dArray::iterator it = boundary->begin(); it != boundary->end(); ++it)
        {
            if (it->x() < minmax.min.x())
                minmax.min.x() = it->x();

            if (it->y() < minmax.min.y())
                minmax.min.y() = it->y();

            if (it->x() > minmax.max.x())
                minmax.max.x() = it->x();

            if (it->y() > minmax.max.y())
                minmax.max.y() = it->y();
        }

        boundaryMinMaxes.push_back(minmax);
    }

    // Will hold keys at reference lod to check
    std::vector<TileKey> keys;

    // Recurse down through tile hierarchy checking for masks at each level.
    // If a given tilekey doesn't have any masks then we don't have to check children.
    std::stack<TileKey> keyStack;
    keyStack.push(rootkey);
    while (!keyStack.empty())
    {
        TileKey key = keyStack.top();
        keyStack.pop();

        if (key.getLOD() < referenceLOD)
        {
            // Make a locator for coordinate conversion:
            GeoLocator geoLocator(key.getExtent());

            bool hasMasks = false;

            for (std::vector<MinMax>::iterator it = boundaryMinMaxes.begin(); it != boundaryMinMaxes.end(); ++it)
            {
                // convert that bounding box to "unit" space (0..1 across the tile)
                osg::Vec3d min_ndc, max_ndc;
                geoLocator.mapToUnit(it->min, min_ndc);
                geoLocator.mapToUnit(it->max, max_ndc);

                // true if boundary overlaps tile in X dimension:
                bool x_match = ((min_ndc.x() >= 0.0 && max_ndc.x() <= 1.0) ||
                    (min_ndc.x() <= 0.0 && max_ndc.x() > 0.0) ||
                    (min_ndc.x() < 1.0 && max_ndc.x() >= 1.0));

                if (!x_match)
                    continue;

                // true if boundary overlaps tile in Y dimension:
                bool y_match = ((min_ndc.y() >= 0.0 && max_ndc.y() <= 1.0) ||
                    (min_ndc.y() <= 0.0 && max_ndc.y() > 0.0) ||
                    (min_ndc.y() < 1.0 && max_ndc.y() >= 1.0));

                if (y_match)
                {
                    // only care if this tile has any masks so we can stop as soon as we find one
                    hasMasks = true;
                    break;
                }
            }

            if (hasMasks == true && includeTilesWithMasks == false)
                continue;

            if (hasMasks == false && includeTilesWithoutMasks == false)
                continue;

            // In order to make this much faster what we need is a way to tell if a key is
            // completely inside the masked region and has no skirt geometry.
            // If there is a fast way to check this, then we can just add the (empty) output geometry
            // with the current tilekey encoded into the user data.
            // This will be a lower lod than the reference lod, but since there is no skirt geometry
            // and the region is totally masked out, the user can easily compute the set of reference lod
            // keys if they need to, and if they don't then this will save having to generate the
            // couple thousand iterations throught the loop below.
            // This will take care of the case when the mask covers many reference lod tiles,
            // and the recursive nature of this loop will make using this function on lower lod tiles much faster.

            keyStack.push(key.createChildKey(0));
            keyStack.push(key.createChildKey(1));
            keyStack.push(key.createChildKey(2));
            keyStack.push(key.createChildKey(3));
        }
        else
        {
            keys.push_back(key);
        }
    }

    if (keys.empty())
        return 0L;

    // group to hold all the tiles
    osg::Group* group = new osg::Group();

    for (std::vector<TileKey>::const_iterator subkey = keys.begin(); subkey != keys.end(); ++subkey)
    {
        // Mask generator creates geometry from masking boundaries when they exist.
        MaskGenerator maskGen(*subkey, tileSize, map.get());

        if (maskGen.hasMasks() == true && includeTilesWithMasks == false)
            continue;

        if (maskGen.hasMasks() == false && includeTilesWithoutMasks == false)
            continue;

        osg::ref_ptr<SharedGeometry> sharedGeom;

        context->getGeometryPool()->getPooledGeometry(
            *subkey,
            tileSize,
            &maskGen,
            sharedGeom);

        osg::ref_ptr<osg::Drawable> drawable = sharedGeom.get();

        osg::UserDataContainer* udc = drawable->getOrCreateUserDataContainer();
        udc->setUserValue("tile_key", subkey->str());

        if (sharedGeom.valid())
        {
            osg::ref_ptr<osg::Geometry> geom = sharedGeom->makeOsgGeometry();
            drawable = geom.get();
            drawable->setUserDataContainer(udc);

            if (!sharedGeom->empty())
            {
                // Burn elevation data into the vertex list
                if (model->elevationModel().valid())
                {
                    // Clone the vertex array since it's shared and we're going to alter it
                    geom->setVertexArray(osg::clone(geom->getVertexArray(), osg::CopyOp::DEEP_COPY_ALL));

                    // Apply the elevation model to the verts, noting that the texture coordinate
                    // runs [0..1] across the tile and the normal is the up vector at each vertex.
                    osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
                    osg::Vec3Array* ups = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
                    osg::Vec3Array* tileCoords = dynamic_cast<osg::Vec3Array*>(geom->getTexCoordArray(0));

                    const osg::HeightField* hf = model->elevationModel()->getHeightField();
                    const osg::RefMatrixf* hfmatrix = model->elevationModel()->getMatrix();

                    // Tile coords must be transformed into the local tile's space
                    // for elevation grid lookup:
                    osg::Matrix scaleBias;
                    subkey->getExtent().createScaleBias(model->getKey().getExtent(), scaleBias);

                    // Apply elevation to each vertex.
                    for (unsigned i = 0; i < verts->size(); ++i)
                    {
                        osg::Vec3& vert = (*verts)[i];
                        osg::Vec3& up = (*ups)[i];
                        osg::Vec3& tileCoord = (*tileCoords)[i];

                        // Skip verts on a masking boundary since their elevations are hard-wired.
                        if ((VERTEX_MARKER_BOUNDARY & (int)tileCoord.z()) == 0) // if BOUNARY bit not set
                        {
                            osg::Vec3d n = osg::Vec3d(tileCoord.x(), tileCoord.y(), 0);
                            n = n * scaleBias;
                            if (hfmatrix) n = n * (*hfmatrix);

                            float z = HeightFieldUtils::getHeightAtNormalizedLocation(hf, n.x(), n.y());
                            if (z != NO_DATA_VALUE)
                            {
                                vert += up*z;
                            }
                        }
                    }
                }

                // Encode the masking extents into a user data object
                if (maskGen.hasMasks())
                {
                    // Find the NDC coords of the masking patch geometry:
                    osg::Vec3d maskMin, maskMax;
                    maskGen.getMinMax(maskMin, maskMax);

                    // Clamp to the tile's extent
                    maskMin.x() = osg::clampBetween(maskMin.x(), 0.0, 1.0);
                    maskMin.y() = osg::clampBetween(maskMin.y(), 0.0, 1.0);
                    maskMax.x() = osg::clampBetween(maskMax.x(), 0.0, 1.0);
                    maskMax.y() = osg::clampBetween(maskMax.y(), 0.0, 1.0);

                    const GeoExtent& e = subkey->getExtent();
                    osg::Vec2d tkMin(e.xMin() + maskMin.x()*e.width(), e.yMin() + maskMin.y()*e.height());
                    osg::Vec2d tkMax(e.xMin() + maskMax.x()*e.width(), e.yMin() + maskMax.y()*e.height());

                    udc->setUserValue("mask_patch_min", tkMin);
                    udc->setUserValue("mask_patch_max", tkMax);
                }
            }

            // Establish a local reference frame for the tile:
            GeoPoint centroid;
            subkey->getExtent().getCentroid(centroid);

            osg::Matrix local2world;
            centroid.createLocalToWorld(local2world);

            osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
            xform->addChild(drawable.get());

            group->addChild(xform);
        }
    }

    return group;
}
