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

#ifndef OSGEARTH_FEATURE_MASK_LAYER_H
#define OSGEARTH_FEATURE_MASK_LAYER_H 1

#include <osgEarthFeatures/Common>
#include <osgEarthFeatures/FeatureSourceLayer>

#include <osgEarth/MaskLayer>
#include <osgEarth/LayerListener>

namespace osgEarth { namespace Features
{
    /**
     * Configuration options for a FeatureMaskLayer.
     */
    class OSGEARTHFEATURES_EXPORT FeatureMaskLayerOptions : public MaskLayerOptions
    {
    public:
        FeatureMaskLayerOptions(const ConfigOptions& options =ConfigOptions());

        /** dtor */
        virtual ~FeatureMaskLayerOptions() { }
        
        /** Map layer containing the feature data */
        optional<std::string>& featureSourceLayer() { return _featureSourceLayer; }
        const optional<std::string>& featureSourceLayer() const { return _featureSourceLayer; }

        /** Embedded feature data definition */
        optional<FeatureSourceOptions>& featureSource() { return _featureSource; }
        const optional<FeatureSourceOptions>& featureSource() const { return _featureSource; }

    public:
        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );

    private:
        void fromConfig( const Config& conf );

        optional<std::string> _featureSourceLayer;
        optional<FeatureSourceOptions> _featureSource;
    };


    /**
     * MaskLayer whose boundary geometry comes from a feature source.
     */
    class OSGEARTHFEATURES_EXPORT FeatureMaskLayer : public MaskLayer
    {
    public:
        META_Layer(osgEarth, FeatureMaskLayer, FeatureMaskLayerOptions, feature_mask);

        /**
         * Constructs a new mask layer based on a configuration setup.
         */
        FeatureMaskLayer(const FeatureMaskLayerOptions& options =FeatureMaskLayerOptions());

        /** The feature source layer from which to get boundary features */
        void setFeatureSourceLayer(FeatureSourceLayer* layer);

        /** The feature source from which to get boundary features */
        void setFeatureSource(FeatureSource* source);

    public: // MaskLayer

        /** 
         * Gets the geometric boundary polygon representing the area of the
         * terrain to mask out.
         */
        virtual osg::Vec3dArray* getOrCreateMaskBoundary(
            float                   heightScale,
            const SpatialReference* srs,
            ProgressCallback*       progress );

    public: // Layer

        virtual const Status& open();

    protected: // Layer

        virtual void addedToMap(const Map*);

        virtual void removedFromMap(const Map*);

    protected:

        /** Create from subclass. */
        FeatureMaskLayer(FeatureMaskLayerOptions*);

        /** dtor */
        virtual ~FeatureMaskLayer();

    private:

        LayerListener<FeatureMaskLayer, FeatureSourceLayer> _featureSourceLayerListener;

        void create();

        osg::ref_ptr<FeatureSource> _featureSource;
        osg::ref_ptr<osg::Vec3dArray> _boundary;
        OpenThreads::Mutex _boundaryMutex;
    };

} } // namespace osgEarth::Features

#endif // OSGEARTH_FEATURE_MASK_LAYER_H

