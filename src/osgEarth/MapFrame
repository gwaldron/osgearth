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

#ifndef OSGEARTH_MAP_FRAME_H
#define OSGEARTH_MAP_FRAME_H 1

#include <osgEarth/Common>
#include <osgEarth/Containers>
#include <osgEarth/MapInfo>
#include <osgEarth/Revisioning>
#include <osgEarth/Layer>
#include <osgEarth/ElevationLayer>

namespace osgEarth
{
    class Profile;
    class Map;
    class MapOptions;
    class TileKey;
    class ElevationPool;
    class ProgressCallback;

    /**
     * A "snapshot in time" of a Map model revision. Use this class to get a safe "copy" of
     * the map model lists that you can use in a multi-threaded environment 
     * without worrying about the model changing underneath you from another thread.
     *
     * Note: a MapFrame can "lose sync" with the actual Map at any time, if the underlying
     * Map is destroyed. So be sure to check return values from the methods below.
     *
     * @deprecated (move to MP engine driver?)
     */
    class OSGEARTH_EXPORT MapFrame
    {
    public:
        //! Creates a new empty map frame that's not connected to any map.
        MapFrame();

        //! Copy constructor
        MapFrame(const MapFrame& rhs);

        //! Creates a new map frame from an existing map
        MapFrame(const Map* map);

        //! Destructor
        virtual ~MapFrame() { }

        //! Whether the frame is connected to a valid map. Note: a return value of 
        //! false is actionable and means the frame's data is invalid. But a return
        //! value of TRUE only indicates a momentary validity -- the frame could 
        //! become invalid at any time!
        bool valid() const;

        //! Connect this frame to a new map object
        void setMap(const Map* map);

        //! Synchronizes this frame to the last-synced map. Returns true if changes occurred.
        bool sync();

        //! True is a sync() would acquire new data.
        bool needsSync() const;

        //! Releases any references held by this frame. Call sync() to restablish
        void release();

        //! Accesses the profile/rendering info about the source map
        const MapInfo& getMapInfo() const { return _mapInfo; }

        //! Convenience method to access the map's profile. This could be NULL
        //! so be sure to check the result.
        const Profile* getProfile() const { return _mapInfo.getProfile(); }

        //! Layers in the map frame
        const LayerVector& layers() const { return _layers; }
        
        //! Copy all the layers of the template type to a vector
        template<typename T>
        unsigned getLayers(std::vector<osg::ref_ptr<T> >& output) const;

        //! Copy all the layers of the template type to a mix-in vector
        template<typename T>
        unsigned getLayers(osg::MixinVector< osg::ref_ptr<T> >& output) const;

        //! Whether the frame contains an enabled layer with matching UID.
        bool containsEnabledLayer(UID uid) const;

        //! Returns the first layer with a given name.
        template<typename T>
        T* getLayerByName(const std::string& name) const;

        //! Returns the layer with the matching UID (or NULL if not found)
        template<typename T>
        T* getLayerByUID(const UID uid) const;

        //! The elevation layer stack snapshot
        const ElevationLayerVector& elevationLayers() const { return _elevationLayers; }

        //! The the map data model revision with which this frame is currently sync'd
        Revision getRevision() const { return _mapDataModelRevision; }

        //! Checks whether all the data for the specified key is cached.
        bool isCached( const TileKey& key ) const;

        //! Access to the map's options
        const MapOptions& getMapOptions() const;

        //! The highest set minLevel() amongst all image and elevation layers
        unsigned getHighestMinLevel() const { return _highestMinLevel; }

        //! Equivalent to the Map::populateHeightField() method, but operates on the
        //! elevation stack snapshot in this MapFrame.
        bool populateHeightField(
            osg::ref_ptr<osg::HeightField>& hf,
            const TileKey&                  key,
            bool                            expressHeightsAsHAE,
            ProgressCallback*               progress) const;

        bool populateHeightFieldAndNormalMap(
            osg::ref_ptr<osg::HeightField>& hf,
            osg::ref_ptr<NormalMap>&        normalMap,
            const TileKey&                  key,
            bool                            expressHeightsAsHAE,
            ProgressCallback*               progress) const;

        //! The map's elevation pool. Could be NULL if the MapFrame has lost sync.
        osg::ref_ptr<ElevationPool> getElevationPool() const;

    private:
        bool _initialized;
        osg::observer_ptr<const Map> _map;
        std::string _name;
        MapInfo _mapInfo;
        Revision _mapDataModelRevision;
        LayerVector _layers;
        ElevationLayerVector _elevationLayers;
        unsigned _highestMinLevel;
        //osg::ref_ptr<osg::Referenced> _pool;

        friend class Map;

        void refreshComputedValues();
    };

    //...............................................................
    // template implementations
    
    template<typename T>
    unsigned MapFrame::getLayers(std::vector<osg::ref_ptr<T> >& output) const
    {
        for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            T* t = dynamic_cast<T*>(i->get());
            if (t) output.push_back(t);
        }
        return output.size();
    }

    template<typename T>
    unsigned MapFrame::getLayers(osg::MixinVector< osg::ref_ptr<T> >& output) const
    {
        for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            T* obj = dynamic_cast<T*>(i->get());
            if (obj) output.push_back(obj);
        }
        return output.size();
    }

    template<typename T>
    T* MapFrame::getLayerByName(const std::string& name) const
    {
        for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            if (i->get()->getName() == name)
                return dynamic_cast<T*>(i->get());
        }
        return 0L;
    }

    template<typename T>
    T* MapFrame::getLayerByUID(const UID uid) const
    {
        for (LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            if (i->get()->getUID() == uid)
                return dynamic_cast<T*>(i->get());
        }
        return 0L;
    }

}

#endif // OSGEARTH_MAP_FRAME_H
