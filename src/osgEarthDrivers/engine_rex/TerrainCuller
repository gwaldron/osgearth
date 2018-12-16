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
#ifndef OSGEARTH_REX_TERRAIN_CULLER_H
#define OSGEARTH_REX_TERRAIN_CULLER_H 1

#include "EngineContext"
#include "TerrainRenderData"
#include "SelectionInfo"

#include <osg/NodeVisitor>
#include <osgUtil/CullVisitor>


using namespace osgEarth;

namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
{
    struct LayerExtent
    {
        LayerExtent() : _computed(false) { }
        bool _computed;
        GeoExtent _extent;
    };
    typedef std::vector<LayerExtent> LayerExtentVector;

    /**
     * Node visitor responsible for assembling a TerrainRenderData that 
     * contains all the information necessary to render the terrain.
     */
    class TerrainCuller : public osg::NodeVisitor, public osg::CullStack
    {
    public:
        TerrainRenderData _terrain;
        EngineContext* _context;
        osg::Camera* _camera;
        TileNode* _currentTileNode;
        DrawTileCommand* _firstDrawCommandForTile;
        unsigned _orphanedPassesDetected;
        osgUtil::CullVisitor* _cv;
        LayerExtentVector* _layerExtents;
        bool _isSpy;

    public:
        /** A new terrain culler */
        TerrainCuller(osgUtil::CullVisitor* cullVisitor, EngineContext* context);

        /** Initialize the culler with a map and a set of render bindings. */
        void setup(const Map* map, LayerExtentVector& layerExtents, const RenderBindings& bindings);

        /** The active camera */
        osg::Camera* getCamera() { return _camera; }

        /** Access to terrain engine resources */
        EngineContext* getEngineContext() { return _context; }

        /** The CullVIsitor that parents this culler. */
        osgUtil::CullVisitor& getParent() { return *_cv; }

        bool isCulledToBBox(osg::Transform* node, const osg::BoundingBox& box);

    public: // osg::NodeVisitor
        void apply(osg::Node& node);
        void apply(TileNode& node);
        void apply(SurfaceNode& node);
        
        float getDistanceToViewPoint(const osg::Vec3& pos, bool withLODScale) const;

    private:

        DrawTileCommand* addDrawCommand(
            UID sourceUID, 
            const TileRenderModel* model, 
            const RenderingPass* pass, 
            TileNode* node);

    };

} } } // namespace 

#endif // OSGEARTH_REX_TERRAIN_CULLER_H
