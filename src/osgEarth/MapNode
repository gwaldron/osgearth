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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef OSGEARTH_MAPNODE_H
#define OSGEARTH_MAPNODE_H 1

#include <osgEarth/Common>
#include <osgEarth/Map>
#include <osgEarth/MapNodeOptions>
#include <osgEarth/Extension>

namespace osgEarth
{
    class OverlayDecorator;
    class Terrain;
    class TerrainEngineNode;
    class MapNodeCullData;
    class SpatialReference;
    class ResourceReleaser;
    class DrapingManager;
    class ClampingManager;
    class CascadeDrapingDecorator;

    /**
     * OSG Node that forms the root of an osgEarth map. This node is a "view" component
     * that renders data from a "Map" data model.
     */
    class OSGEARTH_EXPORT MapNode : public osg::Group
    {
    public:
        /**
         * Attempts to load a MapNode from a ".earth" file in the arguments list
         */
        static MapNode* load(
            class osg::ArgumentParser& arguments);

        /**
         * Attempts to load a MapNode from an ".earth" file, with a set of default
         * map and terrain options.
         */
        static MapNode* load(
            class osg::ArgumentParser& arguments,
            const MapNodeOptions&      defaultOptions);

    public:
        /**
         * Creates an empty map node.
         */
        MapNode();

        /**
         * Creates an empty map node.
         * 
         * @param options
         *      Tile creation and rendering properties
         */
        MapNode( const MapNodeOptions& options );

        /**
         * Creates a new map node.
         *
         * @param map
         *      Map data that this map node will render.
         */
        MapNode( Map* map );

        /**
         * Creates a new map node.
         *
         * @param map
         *      Map data that this map node will render.
         * @param options
         *      Tile creation and rendering properties
         */
        MapNode( Map* map, const MapNodeOptions& options );

    public:

        virtual const char* libraryName() const { return "osgEarth"; }
        virtual const char* className() const { return "MapNode"; }

        /**
         * Gets the Map that this MapNode is rendering.
         */
        Map* getMap();
        const Map* getMap() const;

        /**
         * Gets the spatial reference system of the underlying map.
         * Convenience function.
         */
        const SpatialReference* getMapSRS() const;

        /**
         * Gets an interface for querying the in-memory terrain scene graph directly.
         */
        Terrain* getTerrain();
        const Terrain* getTerrain() const;

        /**
         * Finds the topmost Map node in the specified scene graph, or returns NULL if
         * no Map node exists in the graph.
         *
         * @param graph
         *      Node graph in which to search for a MapNode
         * @param travMask
         *      Traversal mask to apply while searching
         */
        static MapNode* findMapNode( osg::Node* graph, unsigned travMask =~0 );
        static MapNode* get( osg::Node* graph, unsigned travMask =~0 ) { return findMapNode(graph, travMask); }

        /**
         * Returns true if the realized terrain model is geocentric, false if
         * it is flat/projected.
         */
        bool isGeocentric() const;

        /**
         * Accesses the group node that contains all the nodes added by Layers.
         */
        osg::Group* getLayerNodeGroup() const;

        /**
         * Accesses the root node for a specific ModelLayer.
         */
        osg::Node* getLayerNode(Layer* layer) const;
        
        /**
         * Gets the overlay decorator in this mapnode. Usually you do not need to
         * access this directly. Instead install a DrapeableNode in the scene graph.
         */
        OverlayDecorator* getOverlayDecorator() { return _overlayDecorator; }

        /**
         * Gets the engine properties associated with this node. The engine
         * properties dictate how the map engine will create scene graph elements.
         */
        const MapNodeOptions& getMapNodeOptions() const;

        /**
         * Gets the underlying terrain engine that renders the terrain surface of the map.
         */
        TerrainEngineNode* getTerrainEngine() const;

        /**
         * Gets a service that you can use to release GL objects.
         */
        ResourceReleaser* getResourceReleaser() const;

        /**
         * Gets the Config object serializing external data. External data is information
         * that osgEarth itself does not control, but that an app can include in the
         * MapNode for the purposes of including it in a .earth file.
         */
        Config& externalConfig() { return _externalConf; }
        const Config& externalConfig() const { return _externalConf; }

        /**
         * Adds an Extension and calls its startup method with this MapNode.
         */
        void addExtension(Extension* extension, const osgDB::Options* options =0L);

        /**
         * Removes an extension, and calls its shutdown method with this MapNode.
         */
        void removeExtension(Extension* extension);

        /**
         * Removes all extensions, calling each one's shutdown method this this MapNode.
         */
        void clearExtensions();

        /**
         * Access the extensions vector.
         */
        const std::vector< osg::ref_ptr<Extension> >& getExtensions() const { return _extensions; }

        /**
         * Find an extension by type and return it.
         */
        template<typename T>
        T* getExtension() const {
            for(std::vector< osg::ref_ptr<Extension> >::const_iterator i = _extensions.begin(); i != _extensions.end(); ++i) {
                T* e = dynamic_cast<T*>(i->get());
                if ( e ) return e;
            }
            return 0L;
        }

        /**
         * Opens all layers that are not already open.
         */
        void openMapLayers();

        //! Serializes the MapNode into a Config object
        Config getConfig() const;


    public: // special purpose

        /**
         * Constructs a mapnode, optionally specifying whether th intialize the
         * data sources in the map. Typically you would only use this CTOR if you are
         * strictly using the MapNode for serialization.
         */
        MapNode( Map* map, const MapNodeOptions& options, bool initMap );


    public: //override osg::Node

        virtual osg::BoundingSphere computeBound() const;

        virtual void traverse( class osg::NodeVisitor& nv );

        virtual void resizeGLObjectBuffers(unsigned maxSize);

        virtual void releaseGLObjects(osg::State* state) const;

    protected:    

        virtual ~MapNode();

    private:

        osg::ref_ptr<Map>  _map;
        osg::Group*        _layerNodes;
        OverlayDecorator*  _overlayDecorator;
        MapNodeOptions     _mapNodeOptions;
        Config             _externalConf;
        osg::Group*        _maskLayerNode;
        unsigned           _lastNumBlacklistedFilenames;
        TerrainEngineNode* _terrainEngine;
        bool               _terrainEngineInitialized;
        osg::Group*        _terrainEngineContainer;
        ResourceReleaser*  _resourceReleaser;
        DrapingManager*    _drapingManager;
        ClampingManager*   _clampingManager;
        CascadeDrapingDecorator* _cascadeDrapingDecorator;

        std::vector< osg::ref_ptr<Extension> > _extensions;

        Revisioned _mapRevisionMonitor;

    public: // MapCallback proxy

        void onLayerAdded(Layer* layer, unsigned index);
        void onLayerRemoved(Layer* layer, unsigned index);
        void onLayerMoved(Layer* layer, unsigned oldIndex, unsigned newIndex);

        // internal
        osg::Node* getDrapingDump();
        CascadeDrapingDecorator* getCascadeDrapingDecorator() const;

    private:

        osg::ref_ptr< MapCallback > _mapCallback;
    
        void init();

        DrapingManager* getDrapingManager();
        friend class DrapingTechnique;
        friend class DrapeableNode;
        
        ClampingManager* getClampingManager();
        friend class ClampingTechnique;
        friend class ClampableNode;
    };

} // namespace osgEarth

#endif // OSGEARTH_MAPNODE_H
