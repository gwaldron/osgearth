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

#ifndef OSGEARTH_FEATURES_IMAGE_TO_FEATURE_LAYER_H
#define OSGEARTH_FEATURES_IMAGE_TO_FEATURE_LAYER_H 1

#include <osgEarthFeatures/FeatureSourceLayer>
#include <osgEarth/LayerListener>

namespace osgEarth {
    class ImageLayer;
}

namespace osgEarth { namespace Features
{
    /** Options structure for serialization of the FeatureSourceLayer */
    class ImageToFeatureLayerOptions : public FeatureSourceLayerOptions
    {
    public:
        ImageToFeatureLayerOptions(const ConfigOptions& co = ConfigOptions()) : FeatureSourceLayerOptions(co),
            _level(0u),
            _attribute("value")
        {
            fromConfig(_conf);
        }

        //! Name of the image layer to convert to feature data
        optional<std::string>& imageLayer() { return _imageLayerName; }
        const optional<std::string>& imageLayer() const { return _imageLayerName; }

        //! Level of detail from which to extract features
        optional<unsigned>& level() { return _level; }
        const optional<unsigned>& level() const { return _level; }

        //! Attribute name to set with feature value (default is "value")
        optional<std::string>& attribute() { return _attribute; }
        const optional<std::string>& attribute() const { return _attribute; }

    public:
        Config getConfig() const {
            Config conf = FeatureSourceLayerOptions::getConfig();
            conf.set("image", _imageLayerName);
            conf.set("level", _level);
            conf.set("attribute", _attribute);
            return conf;
        }

        void fromConfig(const Config& conf) {
            conf.get("image", _imageLayerName);
            conf.get("level", _level);
            conf.get("attribute", _attribute);
        }

        virtual void mergeConfig( const Config& conf ) {
            FeatureSourceLayerOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        optional<std::string> _imageLayerName;
        optional<unsigned> _level;
        optional<std::string> _attribute;
    };


    /**
     * A FeatureSourceLayer that extracts features from raster data in
     * an image layer.
     */
    class OSGEARTHFEATURES_EXPORT ImageToFeatureLayer : public FeatureSourceLayer
    {
    public:
        META_Layer(osgEarth, ImageToFeatureLayer, ImageToFeatureLayerOptions, image_to_feature);

        //! Create an empty layer
        ImageToFeatureLayer();

        //! Create a new layer with initialization options
        ImageToFeatureLayer(const ImageToFeatureLayerOptions& options);

        //! Image layer to use to convert raster data to features
        void setImageLayer(ImageLayer* layer);

    public: // Layer

        virtual void init();

        virtual void addedToMap(const Map*);

        virtual void removedFromMap(const Map*);

    protected:
        
        LayerListener<ImageToFeatureLayer, ImageLayer> _imageLayerListener;
    };

} } // namespace osgEarth::Features

#endif // OSGEARTH_FEATURES_IMAGE_TO_FEATURE_LAYER_H
