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

#ifndef OSGEARTH_FEATURES_FEATURE_SOURCE_LAYER_H
#define OSGEARTH_FEATURES_FEATURE_SOURCE_LAYER_H 1

#include <osgEarthFeatures/FeatureSource>
#include <osgEarth/Layer>

namespace osgEarth { namespace Features
{
    /** Options structure for serialization of the FeatureSourceLayer */
    class FeatureSourceLayerOptions : public LayerOptions
    {
    public:
        FeatureSourceLayerOptions(const ConfigOptions& co = ConfigOptions()) : LayerOptions(co) {
            _featureSource = co;
        }

        /** Feature source configuration */
        optional<FeatureSourceOptions>& featureSource() { return _featureSource; }
        const optional<FeatureSourceOptions>& featureSource() const { return _featureSource; }

    public:
        virtual Config getConfig() const {
            Config conf = LayerOptions::getConfig();
            if (_featureSource.isSet())
                conf.merge(_featureSource->getConfig());
            return conf;
        }
    private:
        optional<FeatureSourceOptions> _featureSource;
    };

    /**
     * Layer holding a FeatureSource. Create one of these in the Map, and 
     * other Layers can find it and access the shared FeatureSource.

     * <feature_source>...</feature_source>
     */
    class OSGEARTHFEATURES_EXPORT FeatureSourceLayer : public Layer
    {
    public:
        META_Layer(osgEarth, FeatureSourceLayer, FeatureSourceLayerOptions, feature_source);

        /**
         * Construct an empty feature source layer.
         * Call setFeatureSource or configure with options() before adding to a Map.
         */
        FeatureSourceLayer();

        /**
         * Construct a feature source layer with initialization options.
         */
        FeatureSourceLayer(const FeatureSourceLayerOptions& options);

        /**
         * The underlying feature source.
         * open() must be called before getFeatureSource becomes available
         */
        void setFeatureSource(FeatureSource* value);
        FeatureSource* getFeatureSource() const;

    public: // Layer

        /**
         * Open the feature source set this layer's status to its status.
         */
        virtual const Status& open();


    protected:
        osg::ref_ptr<FeatureSource> _featureSource;

        // constructor for subclasses
        FeatureSourceLayer(FeatureSourceLayerOptions* options);
    };


} } // namespace osgEarth::Features

#endif // OSGEARTH_FEATURES_FEATURE_SOURCE_LAYER_H
