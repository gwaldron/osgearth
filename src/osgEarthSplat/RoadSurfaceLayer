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
#ifndef OSGEARTH_SPLAT_ROAD_SURFACE_LAYER
#define OSGEARTH_SPLAT_ROAD_SURFACE_LAYER 1

#include <osgEarth/ImageLayer>
#include <osgEarth/TileRasterizer>
#include <osgEarth/LayerListener>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/FeatureSourceLayer>
#include <osgEarthSymbology/StyleSheet>
#include "Export"

namespace osgEarth { namespace Splat
{
    using namespace osgEarth;
    using namespace osgEarth::Features;

    /**
     * Configuration options for the land use tile source
     */
    class OSGEARTHSPLAT_EXPORT RoadSurfaceLayerOptions : public ImageLayerOptions    
    {
    public:
        RoadSurfaceLayerOptions(const ConfigOptions& options = ConfigOptions())
            : ImageLayerOptions(options)
        {
            fromConfig(_conf);
        }

    public:

        //! Inline feature specification
        optional<FeatureSourceOptions>& features() { return _featureSourceOptions; }
        const optional<FeatureSourceOptions>& features() const { return _featureSourceOptions; }

        //! Name of feature source layer containing features
        optional<std::string>& featureSourceLayer() { return _featureSourceLayer; }
        const optional<std::string>& featureSourceLayer() const { return _featureSourceLayer; }

        //! Buffer around the road vector for querying linear data (should be at least road width/2)
        optional<Distance>& featureBufferWidth() { return _bufferWidth; }
        const optional<Distance>& featureBufferWidth() const { return _bufferWidth; }

        //! Style for rendering the road
        osg::ref_ptr<StyleSheet>& styles() { return _styles; }
        const osg::ref_ptr<StyleSheet>& styles() const { return _styles; }

    public:
        Config getConfig() const
        {
            Config conf = ImageLayerOptions::getConfig();
            conf.set("features",  _featureSourceOptions);
            conf.set("feature_source", _featureSourceLayer);
            conf.set("buffer_width", _bufferWidth);
            conf.set("styles", _styles);
            return conf;
        }

    protected:
        void mergeConfig( const Config& conf ) {
            ImageLayerOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf )
        {
            conf.get("features",  _featureSourceOptions);
            conf.get("feature_source", _featureSourceLayer);
            conf.get("buffer_width", _bufferWidth);
            conf.get("styles", _styles);
        }
        
    private:
        optional<FeatureSourceOptions> _featureSourceOptions;
        optional<std::string> _featureSourceLayer;
        optional<Distance> _bufferWidth;
        osg::ref_ptr<StyleSheet> _styles;
    };

    /**
     * Tile source that will read from ANOTHER tile source and perform
     * various pre-processing syntheses operations like warping and detailing.
     */
    class OSGEARTHSPLAT_EXPORT RoadSurfaceLayer : public osgEarth::ImageLayer
    {
    public:
        META_Layer(osgEarth, RoadSurfaceLayer, RoadSurfaceLayerOptions, road_surface);

        //! Create a blank layer to be configured with options()
        RoadSurfaceLayer();

        //! Create a layer from deserialized options
        RoadSurfaceLayer(const RoadSurfaceLayerOptions& options);

    public:

        //! Sets the map layer from which to pull feature data. Call either
        //! This or setFeatureSource
        void setFeatureSourceLayer(FeatureSourceLayer* layer);

        //! Sets the feature source to get road data from; call either this
        //! or setFeatureSourceLayer
        void setFeatureSource(FeatureSource* source);

    public: // ImageLayer

        // Opens the layer and returns a status
        virtual const Status& open();

        // Creates an image for a tile key
        virtual GeoImage createImageImplementation(const TileKey& key, ProgressCallback* progress);

    protected: // Layer

        // Called by Map when it adds this layer
        virtual void addedToMap(const class Map*);

        // Called by Map when it removes this layer
        virtual void removedFromMap(const class Map*);

        // post-ctor initialization
        virtual void init();

        // A node to add to the scene graph for this layer.
        virtual osg::Node* getNode() const;

    protected:

        virtual ~RoadSurfaceLayer() { }

    private:
        osg::ref_ptr<FeatureSource> _features;
        osg::ref_ptr<Session> _session;
        osg::ref_ptr<TileRasterizer> _rasterizer;

        LayerListener<RoadSurfaceLayer, FeatureSourceLayer> _layerListener;
    };

} } // namespace osgEarth::Splat

#endif // OSGEARTH_SPLAT_ROAD_SURFACE_LAYER
