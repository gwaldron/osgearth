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

#ifndef OSGEARTH_ELEVATION_TERRAIN_LAYER_H
#define OSGEARTH_ELEVATION_TERRAIN_LAYER_H 1

#include <osgEarth/TerrainLayer>
#include <osg/MixinVector>

namespace osgEarth
{
    /**
     * Initialization and serialization options for an elevation layer.
     */
    class OSGEARTH_EXPORT ElevationLayerOptions : public TerrainLayerOptions
    {
    public:
        /** Constructs new elevation layer options. */
        ElevationLayerOptions();
        
        /** Deserializes new elevation layer options. */
        ElevationLayerOptions(const ConfigOptions& options);

        // Constructs new options with a layer name
        ElevationLayerOptions(const std::string& name);

        /** Constructs new elevation layer options, given the underlying driver options. */
        ElevationLayerOptions(const std::string& name, const TileSourceOptions& driverOptions);

        /** dtor */
        virtual ~ElevationLayerOptions() { }

    public:

        optional<bool>& offset() { return _offset; }
        const optional<bool>& offset() const { return _offset; }

        /** Policy for dealing with NO_DATA values in elevation */
        optional<ElevationNoDataPolicy>& noDataPolicy() { return _noDataPolicy; }
        const optional<ElevationNoDataPolicy>& noDataPolicy() const { return _noDataPolicy; }

    public:
        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );
        
    private:
        void fromConfig( const Config& conf );
        void setDefaults();

        optional<bool>                  _offset;
        optional<ElevationNoDataPolicy> _noDataPolicy;
    };
}
OSGEARTH_SPECIALIZE_CONFIG(osgEarth::ElevationLayerOptions);
    

namespace osgEarth
{
    struct ElevationLayerCallback : public TerrainLayerCallback
    {
        //EMPTY
        typedef void (ElevationLayerCallback::*MethodPtr)(class ElevationLayer*);
    };

    /**
     * A map terrain layer containing elevation grid heightfields.
     */
    class OSGEARTH_EXPORT ElevationLayer : public TerrainLayer
    {
    public:
        META_Layer(osgEarth, ElevationLayer, ElevationLayerOptions, elevation);

        /**
         * Constructs a blank Elevation Layer;
         * Use options() to set up before calling open or adding to a Map.
         */
        ElevationLayer();

        /**
         * Constructs a new elevation layer with the specified options. It expects
         * the layer options to contain a reference to the neccesary driver options.
         */
        ElevationLayer( const ElevationLayerOptions& options );

        /**
         * Constructs a new elevation layer with the specific name and driver options.
         * The layer will load its driver by using the tilesource options.
         */
        ElevationLayer( const std::string& name, const TileSourceOptions& driverOptions );

        /**
         * Constructs a new elevation layer with the specified layer options and with a custom
         * TileSource instance created by the user.
         *
         * Note: the ElevationLayerOptions contains a driver() member for configuring a 
         * TileSource. But in this constructor, you are passing in an existing TileSource,
         * and thus the driver() member in ElevationLayerOptions will not be used.
         */
        ElevationLayer( const ElevationLayerOptions& options, TileSource* tileSource );

        /** dtor */
        virtual ~ElevationLayer() { }

    public: // methods
        
        /**
         * Creates a GeoHeightField for this layer that corresponds to the extents and LOD 
         * in the specified TileKey. The returned HeightField will always match the geospatial
         * extents of that TileKey.
         *
         * @param key TileKey for which to create a heightfield.
         */
        GeoHeightField createHeightField(const TileKey& key);

        /**
         * Creates a GeoHeightField for this layer that corresponds to the extents and LOD 
         * in the specified TileKey. The returned HeightField will always match the geospatial
         * extents of that TileKey.
         *
         * @param key TileKey for which to create a heightfield.
         * @param progress Callback for tracking progress and cancelation
         */
        GeoHeightField createHeightField(const TileKey& key, ProgressCallback* progress);

        /**
         * Whether this layer contains offsets instead of absolute heights
         */
        bool isOffset() const;

    protected: // Layer

        virtual void init();

    protected:

        // ctor called by a subclass that owns the options structure
        ElevationLayer(ElevationLayerOptions* optionsPtr);

        //! Subclass can override this by calling setTileSourceExpected(false).
        //! You can create your own normal map, but usually ElevationLayer
        //! will do this for you. Only do it if you really need to create
        //! one by hand (for example, if you are compositing elevaition layers).
        virtual void createImplementation(
            const TileKey& key,
            osg::ref_ptr<osg::HeightField>& out_hf,
            osg::ref_ptr<NormalMap>& out_normalMap,
            ProgressCallback* progress);
        
    private:

        mutable osg::ref_ptr<TileSource::HeightFieldOperation> _preCacheOp;

        TileSource::HeightFieldOperation* getOrCreatePreCacheOp();
        Threading::Mutex _mutex;
        
        // creates a geoHF directly from the tile source
        osg::HeightField* createHeightFieldFromTileSource( 
            const TileKey&    key, 
            ProgressCallback* progress);

        void assembleHeightField(
            const TileKey& key,
            osg::ref_ptr<osg::HeightField>& out_hf,
            osg::ref_ptr<NormalMap>& out_normalMap,
            ProgressCallback* progress);
    };


    /**
     * Vector of elevation layers, with added methods.
     */
    class OSGEARTH_EXPORT ElevationLayerVector : public osg::MixinVector< osg::ref_ptr<ElevationLayer> >
    {
    public:
        /**
         * Populates an existing height field (hf must already exist) with height
         * values from the elevation layers.
         */
        bool populateHeightFieldAndNormalMap(
            osg::HeightField*      hf,
            NormalMap*             normalMap,
            const TileKey&         key,
            const Profile*         haeProfile,
            ElevationInterpolation interpolation,
            ProgressCallback*      progress ) const;

    public:
        /** Default ctor */
        ElevationLayerVector();

        /** Copy ctor */
        ElevationLayerVector(const ElevationLayerVector& rhs);
    };

} // namespace osgEarth

#endif // OSGEARTH_ELEVATION_TERRAIN_LAYER_H
