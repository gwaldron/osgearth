/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#ifndef OSGEARTH_TERRAIN_ENGINE_NODE_H
#define OSGEARTH_TERRAIN_ENGINE_NODE_H 1

#include <osgEarth/Map>
#include <osgEarth/Terrain>
#include <osgEarth/TerrainEffect>
#include <osgEarth/TerrainTileModelFactory>
#include <osgEarth/TerrainTileNode>
#include <osgEarth/TerrainEngineRequirements>
#include <osgEarth/TerrainResources>
#include <osgEarth/ShaderUtils>
#include <osgEarth/Progress>
#include <osgEarth/TileKey>
#include <osg/CoordinateSystemNode>
#include <osg/Geode>
#include <osg/NodeCallback>
#include <osg/BoundingBox>
#include <osgUtil/RenderBin>
#include <set>

#define OSGEARTH_ENV_TERRAIN_ENGINE_DRIVER "OSGEARTH_TERRAIN_ENGINE"


namespace osgUtil {
    class CullVisitor;
}

namespace osgEarth
{
    class TerrainEffect;
    class VirtualProgram;

    /**
     * Interface for and object that can create a new terrain tile models
     */
    class /*interface*/ TerrainEngine
    {
    public:
        /**
         * Callback for new node creation.
         */
        class NodeCallback : public osg::Referenced
        {
        public:
            virtual void operator()(const TileKey& key, osg::Node* node) =0;

        protected:
            NodeCallback() { }
            virtual ~NodeCallback() { }
        };

    public:
        /**
         * Unique ID of this engine.
         */
        virtual UID getUID() const =0;

        /**
         * Creates a new data model for a terrain tile, calling any registered
         * CreateTileModelCallbacks before returning the new object.
         *
         * @param frame  Map frame from which to create the new tile model
         * @param key    Key for which to create the new tile model
         * @param layers UIDs of layers to fetch data for; NULL => all layers
         * @param progress Optional progress/stats tracking callback
         */
        virtual TerrainTileModel* createTileModel(
            const Map*                   map,
            const TileKey&               key,
            const CreateTileModelFilter& filter,
            ProgressCallback*            progress ) =0;

        
        /**
         * Notify the engine of a completed tile node creation, so it can 
         * activate callbacks. (This might happen in a pager thread.)
         *
         * @param key  TileKey corresponding to the new Node
         * @param node The new terrain node.
         */
        virtual void notifyOfTerrainTileNodeCreation(
            const TileKey& key,
            osg::Node*     node) =0;
    };



    /**
     * A callback that lets you customize the computed range for a terrain tile.
     * Providing a custom range can let applications better control how paging behaves.
     */
    struct ComputeRangeCallback : public osg::Referenced
    {
        virtual float operator()(osg::Node* node, osg::NodeVisitor& nv) = 0;
    };


    /**
     * TerrainEngineNode is the base class and interface for map engine implementations.
     *
     * A map engine lives under a MapNode and is responsible for generating the
     * actual geometry representing the Earth.
     */
    class OSGEARTH_EXPORT TerrainEngineNode : public osg::CoordinateSystemNode,
                                              public TerrainEngine,
                                              public TerrainEngineRequirements
    {
    public:
        /** Gets the map that this engine is rendering. */
        const Map* getMap() const { return _map.get(); }

        /** Gets the Terrain interface for interacting with the scene graph */
        Terrain* getTerrain() { return _terrainInterface.get(); }
        const Terrain* getTerrain() const { return _terrainInterface.get(); }

        /** Gets the property set in use by this map engine. */
        virtual const TerrainOptions& getTerrainOptions() const =0;

        /** Accesses the object that keeps track of GPU resources in use */
        TerrainResources* getResources() const;

        /** Adds a terrain effect */
        void addEffect( TerrainEffect* effect );

        /** Removes a terrain effect */
        void removeEffect( TerrainEffect* effect );

        /**
         * Marks the terrain tiles intersecting the provied extent as invalid.
         * If the terrain engine supports incremental update, it will reload
         * invalid tiles. If not, it will simple regenerate all tiles in the 
         * terrain (which might be slow).
         */
        virtual void invalidateRegion(
            const GeoExtent& extent,
            unsigned         minLevel,
            unsigned         maxLevel) { }

        // See invalidateRegion() above.
        void invalidateRegion(const GeoExtent& extent) {
            invalidateRegion(extent, 0u, INT_MAX);
        }

        /** Whether the implementation should generate normal map rasters. */
        void requireNormalTextures();
        
        /** Whether the implementation should generate elevation map rasters. */
        void requireElevationTextures();

        /** Whether the implementation should generate parent color textures. */
        void requireParentTextures();

        /** Access the stateset used to render the terrain. */
        virtual osg::StateSet* getSurfaceStateSet() { return getOrCreateStateSet(); }

        /** Gets the ComputeRangeCallback for this TerrainEngineNode */
        ComputeRangeCallback* getComputeRangeCallback() const;

        /** Sets the ComputeRangeCallback for this TerrainEngineNode */
        void setComputeRangeCallback(ComputeRangeCallback* computeRangeCallback);

        // Request that the terrain tiles be rebuilt.
        virtual void dirtyTerrain();

    public:
        class OSGEARTH_EXPORT ModifyTileBoundingBoxCallback : public osg::Referenced
        {
        public:
            virtual void modifyBoundingBox(const TileKey& key, osg::BoundingBox& box) const =0;
        };

        void addModifyTileBoundingBoxCallback(ModifyTileBoundingBoxCallback* callback);
        void removeModifyTileBoundingBoxCallback(ModifyTileBoundingBoxCallback* callback);

    public: // TerrainEngine

        TerrainTileModel* createTileModel(
            const Map*                   map,
            const TileKey&               key,
            const CreateTileModelFilter& filter,
            ProgressCallback*            progress);

        void notifyOfTerrainTileNodeCreation(
            const TileKey& key, 
            osg::Node*     node);

    public:

        /**
         * Callback for customizing the TileModel.
         */
        class CreateTileModelCallback : public osg::Referenced
        {
        public:
            virtual void onCreateTileModel(
                TerrainEngineNode* engine,
                TerrainTileModel*  model) = 0;
        };

        /**
         * Adds a TileModel creation callback, so you can add custom data
         * to the TileModel after it's created.
         */
        void addCreateTileModelCallback(CreateTileModelCallback* callback);
        void removeCreateTileModelCallback(CreateTileModelCallback* callback);


    public: // TerrainEngineRequirements

        bool normalTexturesRequired() const { return _requireNormalTextures; }
        bool elevationTexturesRequired() const { return _requireElevationTextures; }
        bool parentTexturesRequired() const { return _requireParentTextures; }
        bool elevationBorderRequired() const { return _requireElevationBorder; }
        bool fullDataAtFirstLodRequired() const { return _requireFullDataAtFirstLOD; }

    protected:
        TerrainEngineNode();
        virtual ~TerrainEngineNode();

    public: // osg::Node overrides
        virtual osg::BoundingSphere computeBound() const;
        virtual void traverse( osg::NodeVisitor& );

    protected:
        friend class MapNode;
        friend class TerrainEngineNodeFactory;

        virtual void setMap(const Map* map, const TerrainOptions& options);

        // signals that a redraw is needed because something changed.
        virtual void requestRedraw();

        // Request the state setup be refreshed because something has changed that requires new
        // shaders, state, etc.
        virtual void dirtyState() { }

        osg::ref_ptr<TerrainResources> _textureResourceTracker;

        bool _requireElevationTextures;
        bool _requireNormalTextures;
        bool _requireParentTextures;
        bool _requireElevationBorder;
        bool _requireFullDataAtFirstLOD;
        
        osg::ref_ptr<const Map> _map;

        // called by addTileNodeCallback to notify any already-existing nodes
        // of the new callback.
        virtual void notifyExistingNodes(TerrainEngine::NodeCallback* cb) { }

    public: // utility

        /**
         * Utility function that will return an osg::Node representing the geometry
         * for a tile key. The node is standalone; it has no ability to load children
         * or receive updates.
         *
         * TODO: deprecate in favor of a separate utility for creating simple
         * geometry out of a TerrainTileModel. -gw
         */
        virtual osg::Node* createTile(const TileKey& key) =0;

        //! Flags that affect the createTile behavior.
        enum CreateTileFlags
        {
            CREATE_TILE_INCLUDE_TILES_WITH_MASKS     = 1,
            CREATE_TILE_INCLUDE_TILES_WITHOUT_MASKS  = 2,
            CREATE_TILE_DEFAULT_FLAGS                = ~0 /* all */
        };

        /**
         * Utility for creating standalone tile geometry from a tile model.
         * Note: experimental.
         * @param model Tile model for which to build a tile
         * @param flags OR of the CreateTileFlags enums
         * @param referenceLOD If non-zero, adjust the vertex dimensions of the returned tile
         *        to match this LOD. Example: ask for a tile at tileKey.lod=15, tile is 17x17.
         *        Specific a referenceLOD=16, tile will be 33x33.
         */
        virtual osg::Node* createTile(
            const TerrainTileModel* model,
            int createTileFlags =CREATE_TILE_DEFAULT_FLAGS,
            unsigned referenceLOD =0u) { return 0L; }

    private:
        friend struct TerrainEngineNodeCallbackProxy;
        friend struct MapNodeMapLayerController;

        void ctor();
        void onMapInfoEstablished( const MapInfo& mapInfo ); // not virtual!
        void onMapModelChanged( const MapModelChange& change );
        virtual void updateTextureCombining() { }

    private:
        
        struct ImageLayerController : public ImageLayerCallback
        {
            ImageLayerController( TerrainEngineNode* engine );
            void onColorFiltersChanged( ImageLayer* layer ); 

        private:
            TerrainEngineNode* _engine;
            friend class TerrainEngineNode;
        };

        osg::ref_ptr<ImageLayerController> _imageLayerController;
        bool                               _redrawRequired;
        float                              _verticalScale;
        osg::ref_ptr<Terrain>              _terrainInterface;
        unsigned                           _dirtyCount;
        bool                               _updateScheduled;
        
        enum InitStage {
            INIT_NONE,
            INIT_PREINIT_COMPLETE,
            INIT_POSTINIT_COMPLETE
        };
        InitStage _initStage;

        typedef std::vector<osg::ref_ptr<TerrainEffect> > TerrainEffectVector;
        TerrainEffectVector effects_;

        typedef std::vector<osg::ref_ptr<TerrainEngine::NodeCallback> > NodeCallbackVector;
        NodeCallbackVector _tileNodeCallbacks;
        mutable Threading::Mutex _tileNodeCallbacksMutex;
        
        typedef std::vector<osg::ref_ptr<CreateTileModelCallback> > CreateTileModelCallbacks;
        CreateTileModelCallbacks _createTileModelCallbacks;
        mutable Threading::ReadWriteMutex _createTileModelCallbacksMutex;
        
        typedef std::vector<osg::ref_ptr<ModifyTileBoundingBoxCallback> > ModifyTileBoundingBoxCallbacks;
        ModifyTileBoundingBoxCallbacks _modifyTileBoundingBoxCallbacks;

        osg::ref_ptr<TerrainTileModelFactory> _tileModelFactory;

        osg::ref_ptr<ComputeRangeCallback> _computeRangeCallback;

    public:

        void fireModifyTileBoundingBoxCallbacks(const TileKey& key, osg::BoundingBox& box);

    public:

        /** Access a typed effect. */
        template<typename T>
        T* getEffect() {
            for(TerrainEffectVector::iterator i = effects_.begin(); i != effects_.end(); ++i ) {
                T* e = dynamic_cast<T*>(i->get());
                if ( e ) return e;
            }
            return 0L;
        }
    };

    /**
     * Factory class for creating terrain engine instances.
     */
    class TerrainEngineNodeFactory
    {
    public:
        static TerrainEngineNode* create(const TerrainOptions& options );
    };

} // namespace osgEarth

#endif // OSGEARTH_TERRAIN_ENGINE_NODE_H
