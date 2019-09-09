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

#ifndef OSGEARTH_MAP_H
#define OSGEARTH_MAP_H 1

#include <osgEarth/Common>
#include <osgEarth/GeoData>
#include <osgEarth/Profile>
#include <osgEarth/MapOptions>
#include <osgEarth/MapCallback>
#include <osgEarth/Revisioning>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/ElevationPool>
#include <osgDB/Options>

namespace osgEarth
{
    class MapInfo;

    /**
     * Map is the main data model that the MapNode will render. It is a
     * container for all Layer objects (that contain the actual data) and
     * the rendering options.
     */
    class OSGEARTH_EXPORT Map : public osg::Object
    {
    public:
        META_Object(osgEarth, Map);

        /** Construct a new, empty map. */
        Map();

        /** Construct a new empty map with options */
        Map(const MapOptions& options);

    public:
        /**
         * Gets this Map's unique ID
         */
        UID getUID() const { return _uid; }

        /**
         * Gets the options governing this map.
         */
        const MapOptions& getMapOptions() const { return _mapOptions; }

        /**
         * Gets the options with which this map was initially created.
         */
        const MapOptions& getInitialMapOptions() const { return _initMapOptions; }

        /**
         * Gets the map's master profile, which defines its SRS and tiling structure.
         */
        const Profile* getProfile() const;

        /**
         * Gets the SRS of the map's profile (convenience)
         */
        const SpatialReference* getSRS() const { return _profile.valid() ? _profile->getSRS() : 0L; }

        /**
         * Gets the SRS of the world (scene)
         */
        const SpatialReference* getWorldSRS() const;

        /**
         * Adds a Layer to the map.
         */
        void addLayer(Layer* layer);

        /**
         * Inserts a Layer at a specific index in the Map.
         */
        void insertLayer(Layer* layer, unsigned index);

        /**
         * Removes a layer from the map.
         */
        void removeLayer(Layer* layer);

        /**
         * Moves a layer to another position in the Map.
         */
        void moveLayer(Layer* layer, unsigned index);

        /**
         * Gets the index of the specified layer, or returns
         * getNumLayers() if the layer is not found.
         */
        unsigned getIndexOfLayer(const Layer* layer) const;

        /**
         * Copies references of the map layers into the output list.
         * This method is thread safe. It returns the map revision that was
         * in effect when the data was copied.
         */
        Revision getLayers(LayerVector& out_layers) const;

        /**
         * Gets the number of layers in the map.
         */
        unsigned getNumLayers() const;

        /**
         * Gets a layer by name.
         */
        Layer* getLayerByName(const std::string& name) const;

        template<typename T> T* getLayerByName(const std::string& name) const;

        /**
         * Gets an image layer by its unique ID.
         */
        Layer* getLayerByUID(UID layerUID) const;

        template<typename T> T* getLayerByUID(UID layerUID) const;

        /**
         * Gets the layer at the specified index.
         */
        Layer* getLayerAt(unsigned index) const;

        template<typename T> T* getLayerAt(unsigned index) const;

        /**
         * Fills the vector with references to all layers of the specified type.
         */
        template<typename T>
        Revision getLayers(std::vector< osg::ref_ptr<T> >& output) const;

        template<typename T>
        Revision getLayers(osg::MixinVector< osg::ref_ptr<T> >& output) const;

        /**
         * Gets the first layer of the specified type. This is useful when you
         * know there in only one layer of the type you are looking for.
         */
        template<typename T> T* getLayer() const;

        /**
         * Adds a map layer callback to this map. This will be notified whenever layers are
         * added, removed, or re-ordered.
         */
        MapCallback* addMapCallback(MapCallback* callback) const;

        /**
         * Removes a callback previously added with addMapCallback.
         */
        void removeMapCallback(MapCallback* callback) const;

        /**
         * Begin a batch-update operation. Call this if you intend to add multiple
         * layers at once; then call endUpdate() to complete the operation.
         * Between calls to beginUpdate() and endUpdate(), Map will not invoke
         * any callbacks you added wtih addMapCallback.
         */
        void beginUpdate();

        /**
         * Complete a batch update operation that started with a call to
         * beginUpdate(). Fires all callbacks for operations that occurred
         * since the call to beginUpdate().
         */
        void endUpdate();

        /**
         * Clear all layers from this map.
         */
        void clear();

        /**
         * Replaces the layers in this Map with layers from the specified Map
         * (except for terrain mask layers)
         */
        void setLayersFromMap( const Map* map );

        /**
         * Gets the user-provided options structure stored in this map.
         */
        const osgDB::Options* getGlobalOptions() const;
        void setGlobalOptions( const osgDB::Options* options );

        /**
         * Sets the readable name of this map.
         */
        void setMapName( const std::string& name );

        /**
         * Gets the readable name of this map.
         */
        const std::string& getMapName() const { return _name; }

        /**
         * Sets the Cache for this Map. Set to NULL for no cache.
         */
        void setCache( Cache* cache );

        /**
         * Gets the Cache for this Map
         */
        Cache* getCache() const;

        /**
         * Gets the revision # of the map. The revision # changes every time
         * you add, remove, or move layers. You can use this to track changes
         * in the map model (as a alternative to installing a MapCallback).
         */
        Revision getDataModelRevision() const;

        /**
         * Convenience function that returns TRUE if the map cs type is
         * geocentric.
         */
        bool isGeocentric() const;

        /**
         * Gets the database options associated with this map.
         */
        const osgDB::Options* getReadOptions() const { return _readOptions.get(); }

        /**
         * Gets a version of the map profile without any vertical datum
         */
        const Profile* getProfileNoVDatum() const { return _profileNoVDatum.get(); }

        /**
         * Access to an elevation sampling pool tied to this map
         */
        ElevationPool* getElevationPool() const;

        /**
         * Returns true if data for the given tile key and layers is likely
         * to generate data quickly. This is used for pager thread pool shunting.
         */
        bool isFast(const TileKey& key, const LayerVector& layers) const;

        /**
         * List of attribution strings to be displayed by the application
         */
        void getAttributions(StringSet& attributions) const;

    protected:

        virtual ~Map();

        Map(const Map& rhs, const osg::CopyOp& copy) { }

    private:

        UID _uid;
        MapOptions _mapOptions;
        const MapOptions _initMapOptions;
        std::string _name;
        LayerVector _layers;
        mutable MapCallbackList _mapCallbacks;
        osg::ref_ptr<const osgDB::Options> _globalOptions;
        mutable Threading::ReadWriteMutex _mapDataMutex;
        osg::ref_ptr<const Profile> _profile;
        osg::ref_ptr<const Profile> _profileNoVDatum;
        Revision _dataModelRevision;
        osg::ref_ptr<osgDB::Options> _readOptions;
        osg::ref_ptr<ElevationPool> _elevationPool;

        struct VisibleLayerCB : public VisibleLayerCallback {
            osg::observer_ptr<Map> _map;
            VisibleLayerCB(Map*);
            void onVisibleChanged(VisibleLayer* layer);
        };
        osg::ref_ptr<VisibleLayerCB> _visibleLayerCB;
        friend struct VisibleLayerCB;
        void notifyLayerVisibleChanged(VisibleLayer*);

        struct LayerCB : public LayerCallback {
            LayerCB(Map*);
            osg::observer_ptr<Map> _map;
            void onEnabledChanged(Layer* layer);
        };
        osg::ref_ptr<LayerCallback> _layerCB;
        friend struct LayerCB;
        void notifyOnLayerEnabledChanged(Layer*);

        void installLayerCallbacks(Layer*);
        void uninstallLayerCallbacks(Layer*);
        void openLayer(Layer*);
        void closeLayer(Layer*);


    private:
        void ctor();
        void calculateProfile();

        friend class MapInfo;
    };




    // Templated inline methods

    template<typename T> T* Map::getLayerByName(const std::string& name) const {
        return dynamic_cast<T*>(getLayerByName(name));
    }

    template<typename T> T* Map::getLayerByUID(UID layerUID) const {
        return dynamic_cast<T*>(getLayerByUID(layerUID));
    }

    template<typename T> T* Map::getLayerAt(unsigned index) const {
        return dynamic_cast<T*>(getLayerAt(index));
    }

    template<typename T>
    Revision Map::getLayers(std::vector< osg::ref_ptr<T> >& output) const {
        Threading::ScopedReadLock lock(_mapDataMutex);
        for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i) {
            T* obj = dynamic_cast<T*>(i->get());
            if (obj) output.push_back(obj);
        }
        return _dataModelRevision;
    }

    template<typename T>
    Revision Map::getLayers(osg::MixinVector< osg::ref_ptr<T> >& output) const {
        Threading::ScopedReadLock lock(_mapDataMutex);
        for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i) {
            T* obj = dynamic_cast<T*>(i->get());
            if (obj) output.push_back(obj);
        }
        return _dataModelRevision;
    }

    template<typename T> T* Map::getLayer() const {        
        Threading::ScopedReadLock lock(_mapDataMutex);
        for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i) {
            T* obj = dynamic_cast<T*>(i->get());
            if (obj) return obj;
        }
        return 0L;
    }
}

#endif // OSGEARTH_MAP_H
