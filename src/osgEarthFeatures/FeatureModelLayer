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
#ifndef OSGEARTH_FEATURES_FEATURE_MODEL_LAYER
#define OSGEARTH_FEATURES_FEATURE_MODEL_LAYER 1

#include <osgEarthFeatures/Common>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/FeatureSourceLayer>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/Session>
#include <osgEarthSymbology/Style>
#include <osgEarth/Layer>
#include <osgEarth/LayerListener>

namespace osgEarth {
    class Map;
}

namespace osgEarth { namespace Features
{
    using namespace osgEarth;

    class FeatureNodeFactory;

    /**
     * Serializable options to configure a FeatureModelLayer
     */
    class OSGEARTHFEATURES_EXPORT FeatureModelLayerOptions : public VisibleLayerOptions,
                                                             public FeatureModelOptions,
                                                             public GeometryCompilerOptions
    {
    public:
        // constructor
        FeatureModelLayerOptions(const ConfigOptions& co = ConfigOptions());
        
        /** Name of the feature source layer to use for flattening. */
        optional<std::string>& featureSourceLayer() { return _featureSourceLayer; }
        const optional<std::string>& featureSourceLayer() const { return _featureSourceLayer; }

    public: // LayerOptions
        virtual Config getConfig() const;

    protected: // LayerOptions
        virtual void mergeConfig(const Config& conf);
        void fromConfig(const Config& conf);
        
    private:
        optional<std::string> _featureSourceLayer;
    };


    /**
     * Layer that creates a scene graph from feature data and symbology.
     */
    class OSGEARTHFEATURES_EXPORT FeatureModelLayer : public VisibleLayer
    {
    public:
        META_Layer(osgEarth, FeatureModelLayer, FeatureModelLayerOptions, feature_model);

        //! Create a feature layer with defaults. Use options() to set it up before adding
        //! the layer to a Map.
        FeatureModelLayer();

        //! Create a layer with initial options.
        FeatureModelLayer(const FeatureModelLayerOptions& options);

        //! Options used to initialize this layer.
        const FeatureModelLayerOptions& getFeatureModelLayerOptions() const { return *_options; }

        //! Sets the options to initilize the layer
        void setFeatureModelLayerOptions(const FeatureModelLayerOptions& options);

        //! Feature source layer from which to get a feature source. Use this OR setFeatureSource
        void setFeatureSourceLayer(FeatureSourceLayer* layer);
        
        //! The feature source from which to read geometry. Use this OR setFeatureLayer
        void setFeatureSource(FeatureSource* fs);
        FeatureSource* getFeatureSource() const;

        //! Forces a rebuild on this FeatureModelLayer.
        void dirty();

    public:
        class CreateFeatureNodeFactoryCallback : public osg::Referenced {
        public:
            virtual FeatureNodeFactory* createFeatureNodeFactory(const FeatureModelLayerOptions&) =0;
        };

        //! Callback that will create a new FeatureNodeFactory for this layer to use
        //! to transform feature data into OSG scene graphs
        void setCreateFeatureNodeFactoryCallback(CreateFeatureNodeFactoryCallback* callback);
        CreateFeatureNodeFactoryCallback* getCreateFeatureNodeFactoryCallback() const;


    public: // Layer

        // opens the layer and returns the status
        virtual const Status& open();

        // The Node representing this layer.
        virtual osg::Node* getNode() const;

        //! Extent of the feature layer, if available (INVALID if not)
        virtual const GeoExtent& getExtent() const;

    protected: // Layer
        
        // called by the map when this layer is added
        virtual void addedToMap(const Map*);

        // called by the map when this layer is removed
        virtual void removedFromMap(const Map*);

        // post-ctor initialization
        virtual void init();

    protected:

        virtual ~FeatureModelLayer();

        //! You can override this as an alternative to setting the CreateFeatureNodeFactoryCallback
        virtual FeatureNodeFactory* createFeatureNodeFactoryImplementation() const;

    private:
        osg::ref_ptr<FeatureSource> _featureSource;
        LayerListener<FeatureModelLayer, FeatureSourceLayer> _featureSourceLayerListener;
        osg::ref_ptr<class Session> _session;
        osg::ref_ptr<osg::Group> _root;
        bool _graphDirty;
        osg::ref_ptr<CreateFeatureNodeFactoryCallback> _createFactoryCallback;
        
        void create();
        FeatureNodeFactory* createFeatureNodeFactory();
    };

} } // namespace osgEarth::Features

#endif // OSGEARTH_FEATURES_FEATURE_MODEL_LAYER
