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

#ifndef OSGEARTH_TERRAIN_LAYER_H
#define OSGEARTH_TERRAIN_LAYER_H 1

#include <osgEarth/Common>
#include <osgEarth/CachePolicy>
#include <osgEarth/Config>
#include <osgEarth/VisibleLayer>
#include <osgEarth/TileSource>
#include <osgEarth/Profile>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/Status>

namespace osgEarth
{
    class Cache;
    class CacheBin;
    class MemCache;

    /**
     * Initialization (and serializable) options for a terrain layer.
     */
    class OSGEARTH_EXPORT TerrainLayerOptions : public VisibleLayerOptions
    {
    public:
        //! Construct empty (default) options.
        TerrainLayerOptions();

        //! Deserialize options from a Config structure
        TerrainLayerOptions(const ConfigOptions& options);

        //! Construct empty named options.
        TerrainLayerOptions(const std::string& name);

        //! Construct empty options with TileSource configuration.
        TerrainLayerOptions(const std::string& name, const TileSourceOptions& driverOptions);

        /**
         * Gets the explicity vertical datum identifier that will override a vertical
         * datum specified by the tile source.
         */
        optional<std::string>& verticalDatum() { return _vertDatum; }
        const optional<std::string>& verticalDatum() const { return _vertDatum; }

        /**
         * Options for the underlyint tile source driver.
         */
        optional<TileSourceOptions>& driver() { return _driver; }
        const optional<TileSourceOptions>& driver() const { return _driver; }

        /**
         * Gets or sets the minimum of detail for which this layer should generate data.
         */
        optional<unsigned>& minLevel() { return _minLevel; }
        const optional<unsigned>& minLevel() const { return _minLevel; }

        /**
         * Gets or sets the minimum resolution for which this layer should generate data.
         * The value is in units per pixel, using the base units of the layer's source data.
         */
        optional<double>& minResolution() { return _minResolution; }
        const optional<double>& minResolution() const { return _minResolution; }

        /**
         * The maximum level of detail for which this layer should generate data.
         * Data from this layer will not appear in map tiles above the maxLevel.
         */
        optional<unsigned>& maxLevel() { return _maxLevel; }
        const optional<unsigned>& maxLevel() const { return _maxLevel; }

        /**
         * The maximum level resolution for which this layer should generate data.
         * The value is in units per pixel, using the base units of the layer's source data.
         */
        optional<double>& maxResolution() { return _maxResolution; }
        const optional<double>& maxResolution() const { return _maxResolution; }

        /**
         * The maximum level of detail for which this layer should generate new data.
         * Data from this layer will be upsampled in map tiles above the maxDataLevel.
         */
        optional<unsigned>& maxDataLevel() { return _maxDataLevel; }
        const optional<unsigned>& maxDataLevel() const { return _maxDataLevel; }

        /**
         * Whether to use exact cropping if image cropping is necessary
         */
        optional<bool>& exactCropping() { return _exactCropping; }
        const optional<bool>& exactCropping() const { return _exactCropping; }

        /**
         * The desired tile size to reproject imagery to if necessary.
         * TODO: evaluate for possible deprecation
         */
        optional<unsigned int>& reprojectedTileSize() { return _reprojectedTileSize; }
        const optional<unsigned int>& reprojectedTileSize() const { return _reprojectedTileSize; }

        /**
         * The ratio used to expand the extent of a tile when the layer
         * needs to be mosaiced to projected.  This can be used to increase the
         * number of tiles grabbed to ensure that enough data is grabbed to
         * overlap the incoming tile.
         */
        optional<double>& edgeBufferRatio() { return _edgeBufferRatio;}
        const optional<double>& edgeBufferRatio() const { return _edgeBufferRatio; }

        //! The hostname/port of a proxy server to use for HTTP communications on this layer
        //! Default = no proxy.
        optional<ProxySettings>& proxySettings() { return _proxySettings; }
        const optional<ProxySettings>& proxySettings() const { return _proxySettings; }

        //! Number of samples in each dimension.
        optional<unsigned>& tileSize() { return _tileSize; }
        const optional<unsigned>& tileSize() const { return _tileSize; }

        //! Value to treat as a "no data" marker.
        optional<float>& noDataValue() { return _noDataValue; }
        const optional<float>& noDataValue() const { return _noDataValue; }

        //! Treat any value less than this as a "no data" marker.
        optional<float>& minValidValue() { return _minValidValue; }
        const optional<float>& minValidValue() const { return _minValidValue; }

        //! Treat any value greater than this as a "no data" marker.
        optional<float>& maxValidValue() { return _maxValidValue; }
        const optional<float>& maxValidValue() const { return _maxValidValue; }

    public:
        virtual Config getConfig() const;

        virtual void mergeConfig( const Config& conf );

    private:
        void setDefaults();
        void fromConfig( const Config& conf );

        optional<std::string>       _vertDatum;
        optional<TileSourceOptions> _driver;
        optional<unsigned>          _minLevel;
        optional<unsigned>          _maxLevel;
        optional<double>            _minResolution;
        optional<double>            _maxResolution;
        optional<unsigned>          _maxDataLevel;
        optional<bool>              _exactCropping;
        optional<unsigned>          _reprojectedTileSize;
        optional<double>            _edgeBufferRatio;
        optional<unsigned>          _tileSize;
        optional<ProxySettings>     _proxySettings;
        optional<float>             _noDataValue;
        optional<float>             _minValidValue;
        optional<float>             _maxValidValue;
    };


    struct TerrainLayerCallback : public VisibleLayerCallback
    {
        typedef void(TerrainLayerCallback::*MethodPtr)(class TerrainLayer*);
    };

    /**
     * A layer that comprises the terrain skin (image or elevation layer)
     */
    class OSGEARTH_EXPORT TerrainLayer : public VisibleLayer
    {
    public:
        META_Layer(osgEarth, TerrainLayer, TerrainLayerOptions, terrain);

    protected:
        //! Construct from subclass with a pointer to the concerete options structure.
        TerrainLayer(TerrainLayerOptions* =0L);

        //! Construct from subclass with pointer to concrete options and premade tile source
        TerrainLayer(TerrainLayerOptions*, TileSource*);

        //! DTOR
        virtual ~TerrainLayer();

    public: // Layer

        /** Opens the layer and initializes the data source. */
        virtual const Status& open();

        //! Cache ID for this layer
        virtual std::string getCacheID() const;


    public:

        /**
         * Gets the profile of this layer
         */
        const Profile* getProfile() const;

        /**
         * Gets the underlying TileSource engine that serves this map layer. Use with caution.
         */
        TileSource* getTileSource() const;

        /**
         * Gets the size (i.e. number of samples in each dimension) or the source
         * data for this layer.
         */
        virtual unsigned getTileSize() const;

        /**
         * Whether the layer represents dynamic data, i.e. it generates data that requires
         * an update traversal.
         */
        virtual bool isDynamic() const;

        /** Attribution to be displayed by the application */
        virtual std::string getAttribution() const;

        /**
         * Whether the data for the specified tile key is in the cache.
         */
        virtual bool isCached(const TileKey& key) const;

        /**
         * Gives the terrain layer a hint as to what the target profile of
         * images will be. This is optional, but it may allow the layer to enable
         * certain optimizations since it has more information as to how the
         * data will be used.
         */
        virtual void setTargetProfileHint( const Profile* profile );

        /**
         * Disable this layer, setting an error status.
         */
        void disable(const std::string& msg);

        /**
         * Sets the I/O options to use. This data may include cache information.
         */
        void setReadOptions(const osgDB::Options* readOptions);


    public: // Data availability methods

        /**
         * Given a TileKey, returns a TileKey representing the best known available.
         * For example, if the input TileKey exceeds the layer's max LOD, the return
         * value will be an ancestor key at that max LOD.
         *
         * If a setting that effects the visible range of this layer is set (minLevel, maxLevel, minResolution or maxResolution)
         * then any key passed in that falls outside of the valid range for the layer will return TileKey::INVALID.
         */
        virtual TileKey getBestAvailableTileKey(const TileKey& key) const;

        /**
         * Whether the layer possibly has real data for the provided TileKey.
         * Best guess given available information.
         */
        virtual bool mayHaveData(const TileKey& key) const;

        /**
         * Whether the given key falls within the range limits set in the options;
         * i.e. min/maxLevel or min/maxResolution. (This does not mean that the key
         * will result in data.)
         */
        virtual bool isKeyInLegalRange(const TileKey& key) const;

        /**
         * Same as isKeyInLegalRange, but ignores the "maxDataLevel" setting
         * since that does NOT affect visibility of a tile.
         */
        virtual bool isKeyInVisualRange(const TileKey& key) const;

        /**
         * Data Extents reported for this layer are copied into output.
         * Returns true on success, false is there are no extents to report.
         */
        const DataExtentList& getDataExtents() const;

        /**
         * Gets an extent that is the union of all the extents in getDataExtents().
         */
        const GeoExtent& getDataExtentsUnion() const;


    public: // Data interpretation methods

        //! Special value representing "no data" in a location
        virtual float getNoDataValue() const;

        //! Values less than this will be reinterpreted as "no data"
        virtual float getMinValidValue() const;

        //! Values greater than this weill be reinterpreted as "no data"
        virtual float getMaxValidValue() const;


    public: // Layer interface

        virtual SequenceControl* getSequenceControl();

    public:

        /**
         * Metadata about the terrain layer that is stored in the cache, and read
         * when the cache is opened.
         */
        struct OSGEARTH_EXPORT CacheBinMetadata : public osg::Referenced
        {
            CacheBinMetadata();

            CacheBinMetadata( const CacheBinMetadata& rhs );

            CacheBinMetadata( const Config& conf );

            bool isOK() const { return _valid; }

            Config getConfig() const;

            bool                     _valid;
            optional<std::string>    _cacheBinId;
            optional<std::string>    _sourceName;
            optional<std::string>    _sourceDriver;
            optional<int>            _sourceTileSize;
            optional<ProfileOptions> _sourceProfile;
            optional<ProfileOptions> _cacheProfile;
            optional<TimeStamp>      _cacheCreateTime;
            DataExtentList           _dataExtents;
        };

        /**
         * Access to information about the cache
         */
        CacheBinMetadata* getCacheBinMetadata(const Profile* profile);

        /**
         * Cache Settings for this layer - guaranteed to return an object
         */
        CacheSettings* getCacheSettings() const;

    protected: // Layer

        // CTOR initialization; call from subclass.
        virtual void init();

        //! Extent of this layer's data.
        virtual const GeoExtent& getExtent() const;

    protected:

        // Creates the driver the supplies the actual data.
        // By default, this function will either return the tile source passed
        // into the CTOR or it will try to load it from a plugin based on the
        // driver specification in the TileSourceOptions. You can override this
        // method to create your own tile source.
        virtual TileSource* createTileSource();

        void applyProfileOverrides();

        CacheBin* getCacheBin(const Profile* profile);

        DataExtentList& dataExtents();

        //! Call this if you call dataExtents() and modify it.
        void dirtyDataExtents();

    protected:

        osg::ref_ptr<const Profile>    _targetProfileHint;
        unsigned                       _tileSize;
        osg::ref_ptr<MemCache>         _memCache;
        bool _openCalled;

        // profile from tile source or cache, before any overrides applied
        mutable osg::ref_ptr<const Profile> _profileOriginal;

        // profile to use
        mutable osg::ref_ptr<const Profile> _profile;

        // cache key for metadata
        std::string getMetadataKey(const Profile*) const;

        // Called by a subclass before open() to indicate whether this
        // layer should try to open a tile source and fail if unsuccesful.
        void setTileSourceExpected(bool value) { _tileSourceExpected = value; }

        //! Whether this layer expected a tilesource to be installed
        bool isTileSourceExpected() const { return _tileSourceExpected; }

        //! Subclass can set a profile on this layer before opening
        void setProfile(const Profile* profile);

    private:
        bool                     _tileSourceExpected;
        mutable Threading::Mutex _initTileSourceMutex;
        osg::ref_ptr<TileSource> _tileSource;
        DataExtentList           _dataExtents;
        mutable GeoExtent        _dataExtentsUnion;

        // The cache ID used at runtime. This will either be the cacheId found in
        // the TerrainLayerOptions, or a dynamic cacheID generated at runtime.
        std::string _runtimeCacheId;

        // cache policy that may be automatically set by the layer and will
        // override the runtime options policy if set.
        optional<CachePolicy> _runtimeCachePolicy;

        typedef std::map<std::string, osg::ref_ptr<CacheBinMetadata> > CacheBinMetadataMap;
        CacheBinMetadataMap _cacheBinMetadata;

        mutable Threading::Mutex _mutex;

        mutable osg::ref_ptr<CacheSettings> _cacheSettings;

        // methods accesible by Map:
        friend class Map;
        void storeProxySettings( osgDB::Options* );

        // read the tile source's cache policy hint and apply as necessary
        void refreshTileSourceCachePolicyHint(TileSource*);

        TileSource* createAndOpenTileSource();

        // Figure out the cache settings for this layer.
        void establishCacheSettings();

    protected:
        /** Closes the layer, deleting its tile source and any other resources. */
        virtual void close();

    };

    typedef std::vector<osg::ref_ptr<TerrainLayer> > TerrainLayerVector;

} // namespace TerrainLayer

#endif // OSGEARTH_TERRAIN_LAYER_H
