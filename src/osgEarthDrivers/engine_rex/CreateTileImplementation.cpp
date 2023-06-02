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
#include <osgEarth/TileKey>
#include <osgEarth/Locators>
#include <osg/MatrixTransform>
#include <osg/ValueObject>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::REX;

#undef LC
#define LC "[REX::CreateTileImpl] "

osg::Node*
CreateTileImplementation::createTile(
    EngineContext* context,
    const TerrainTileModel* model,
    int flags,
    unsigned referenceLOD,
    const TileKey& area,
    Cancelable* progress)
{
    if (model == nullptr)
    {
        OE_WARN << LC << "Illegal: createTile(NULL)" << std::endl;
        return 0L;
    }

    // Verify that we have a map:
    osg::ref_ptr<const Map> map = context->getMap();
    if (!map.valid())
    {
        return nullptr;
    }

    // Dimension of each tile in vertices
    unsigned tileSize = context->options().getTileSize();
    TileKey rootkey = area.valid() ? area : model->key();
    const SpatialReference* srs = rootkey.getExtent().getSRS();

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

    bool include_constrained = (flags & TerrainEngineNode::CREATE_TILE_INCLUDE_TILES_WITH_MASKS) != 0;
    bool include_unconstrained = (flags & TerrainEngineNode::CREATE_TILE_INCLUDE_TILES_WITHOUT_MASKS) != 0;

    // group to hold all the tiles
    osg::ref_ptr<osg::Group> group;

    for (std::vector<TileKey>::const_iterator subkey = keys.begin(); subkey != keys.end(); ++subkey)
    {
        osg::ref_ptr<SharedGeometry> sharedGeom;

        context->getGeometryPool()->getPooledGeometry(
            *subkey,
            tileSize,
            map.get(),
            context->options(),
            sharedGeom,
            progress);

        if (progress && progress->isCanceled())
        {
            return nullptr;
        }

        if (sharedGeom.valid() == false &&
            include_constrained == true &&
            include_unconstrained == false)
        {
            // This means that we found a constrained tile that was completely 
            // masked out - all triangles were removed. If we are ONLY returning
            // constrained tiles, make an empty group for it to mark its
            // existance.
            if (!group.valid())
                group = new osg::Group();

            osg::Group* empty_tile_group = new osg::Group();
            osg::UserDataContainer* udc = empty_tile_group->getOrCreateUserDataContainer();
            udc->setUserValue("tile_key", subkey->str());
            group->addChild(empty_tile_group);
        }

        else if (
            sharedGeom.valid() &&
            !sharedGeom->empty() &&
            (
                (include_constrained && sharedGeom->hasConstraints()) ||
                (include_unconstrained && !sharedGeom->hasConstraints())
            ))
        {
            // This means we got some geometry.
            if (!group.valid())
                group = new osg::Group();

            osg::ref_ptr<osg::Drawable> drawable = sharedGeom.get();

            osg::UserDataContainer* udc = drawable->getOrCreateUserDataContainer();
            udc->setUserValue("tile_key", subkey->str());

            osg::ref_ptr<osg::Geometry> geom = sharedGeom->makeOsgGeometry();
            drawable = geom.get();

            drawable->setUserDataContainer(udc);

            // Burn elevation data into the vertex list
            if (model->elevation().texture())
            {
                // Clone the vertex array since it's shared and we're going to alter it
                geom->setVertexArray(osg::clone(geom->getVertexArray(), osg::CopyOp::DEEP_COPY_ALL));

                // Apply the elevation model to the verts, noting that the texture coordinate
                // runs [0..1] across the tile and the normal is the up vector at each vertex.
                osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
                osg::Vec3Array* ups = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
                osg::Vec3Array* tileCoords = dynamic_cast<osg::Vec3Array*>(geom->getTexCoordArray(0));

                const osg::HeightField* hf = model->elevation().heightField();
                const osg::Matrix& hfmatrix = model->elevation().matrix();

                // Tile coords must be transformed into the local tile's space
                // for elevation grid lookup:
                osg::Matrix scaleBias;
                subkey->getExtent().createScaleBias(model->key().getExtent(), scaleBias);

                // Apply elevation to each vertex.
                for (unsigned i = 0; i < verts->size(); ++i)
                {
                    osg::Vec3& vert = (*verts)[i];
                    osg::Vec3& up = (*ups)[i];
                    osg::Vec3& tileCoord = (*tileCoords)[i];

                    // Skip verts on a masking boundary since their elevations are hard-wired.
                    if ((VERTEX_HAS_ELEVATION & (int)tileCoord.z()) == 0) // if VERTEX_HAS_ELEVATION bit not set
                    {
                        osg::Vec3d n = osg::Vec3d(tileCoord.x(), tileCoord.y(), 0);
                        n = n * scaleBias;
                        n = n * hfmatrix;

                        float z = HeightFieldUtils::getHeightAtNormalizedLocation(hf, n.x(), n.y());
                        if (z != NO_DATA_VALUE)
                        {
                            vert += up * z;
                        }
                    }
                }

                verts->dirty();
            }

            // Establish a local reference frame for the tile:
            GeoPoint centroid = subkey->getExtent().getCentroid();

            osg::Matrix local2world;
            centroid.createLocalToWorld(local2world);

            osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
            xform->addChild(drawable.get());

            group->addChild(xform);
        }
    }

    return group.release();
}