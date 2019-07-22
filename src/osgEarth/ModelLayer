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

#ifndef OSGEARTH_MODEL_LAYER_H
#define OSGEARTH_MODEL_LAYER_H 1

#include <osgEarth/Common>
#include <osgEarth/VisibleLayer>
#include <osgEarth/Cache>
#include <osgEarth/Config>
#include <osgEarth/ModelSource>
#include <osgEarth/MaskSource>
#include <osgEarth/ShaderUtils>
#include <osgEarth/Containers>
#include <osg/Node>
#include <osg/Array>
#include <vector>

namespace osgEarth
{
    class Map;

    /**
     * Configuration options for a ModelLayer.
     */
    class OSGEARTH_EXPORT ModelLayerOptions : public VisibleLayerOptions
    {
    public:        
        //! Construct a new, empty options structure
        ModelLayerOptions();

        /** Construct or deserialize new model layer options. */
        ModelLayerOptions(const ConfigOptions& options);

        /** Construct or deserialize new model layer options. */
        ModelLayerOptions(
            const std::string& name, 
            const ModelSourceOptions& driverOptions);

        /**
         * Options for the underlying model source driver.
         */
        optional<ModelSourceOptions>& driver() { return _driver; }
        const optional<ModelSourceOptions>& driver() const { return _driver; }

        /**
         * Whether to enable OpenGL lighting on the model node.
         */
        optional<bool>& lightingEnabled() { return _lighting; }
        const optional<bool>& lightingEnabled() const { return _lighting; }

        /**
         * Masking options for cutting a hole in the terrain to accommodate this model.
         * Note; the mask will NOT honor any visibility or opacity settings on the
         * model layer.
         */
        optional<MaskSourceOptions>& mask() { return _maskOptions; }
        const optional<MaskSourceOptions>& mask() const { return _maskOptions; }

        /**
         * Minimum terrain LOD at which to apply the mask (if there if one)
         */
        optional<unsigned>& maskMinLevel() { return _maskMinLevel; }
        const optional<unsigned>& maskMinLevel() const { return _maskMinLevel; }


    public:
        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );

    private:
        void fromConfig( const Config& conf );
        void setDefaults();

        optional<ModelSourceOptions> _driver;
        optional<bool>               _lighting;
        optional<MaskSourceOptions>  _maskOptions;
        optional<unsigned>           _maskMinLevel;
        optional<CachePolicy>        _cachePolicy;
        optional<std::string>        _cacheId;
    };

    struct ModelLayerCallback : public VisibleLayerCallback
    {
        // Nothing - placeholder
        typedef void (ModelLayerCallback::*MethodPtr)(class ModelLayer* layer);
    };


    /**
     * Layer that contains an OSG scene graph
     */
    class OSGEARTH_EXPORT ModelLayer : public VisibleLayer
    {
    public:
        META_Layer(osgEarth, ModelLayer, ModelLayerOptions, model);

        /**
         * Blank ModelLayer. Use options() to setup before calling open.
         */
        ModelLayer();

        /**
         * Constructs a new model layer.
         */
        ModelLayer(const ModelLayerOptions& options);

        /**
         * Constructs a new model layer with a user-provided driver options.
         */
        ModelLayer(const std::string& name, const ModelSourceOptions& options);
        
        /**
         * Constructs a new model layer with a user-provided model source.
         *
         * Note: the ModelLayerOptions contains a driver() member for configuring a 
         * TileSource. But in this constructor, you are passing in an existing TileSource,
         * and thus the driver() member in ModelLayerOptions will not be used.
         */
        ModelLayer(const ModelLayerOptions& options, ModelSource* source );

        /**
         * Constructs a new model layer with a user provided name and an existing node
         */
        ModelLayer(const std::string& name, osg::Node* node);


    public:

        /**
         * Access the underlying model source.
         */
        ModelSource* getModelSource() const { return _modelSource.get(); }

        /**
         * The underlying mask source, if one exists.
         */
        MaskSource* getMaskSource() const { return _maskSource.get(); }

        /**
         * The minimum terrain LOD at which to apply the mask.
         */
        unsigned getMaskMinLevel() const { return options().maskMinLevel().get(); }
        
        /**
         * The boundary geometry for the mask.
         */
        osg::Vec3dArray* getOrCreateMaskBoundary(
            float                   heightScale,
            const SpatialReference* srs,
            ProgressCallback*       progress);

    public: // properties

        /** whether to apply lighting to the model layer's root node */
        void setLightingEnabled( bool value );
        bool isLightingEnabled() const;

    public: // Layer

        //! Open the layer and return its status
        virtual const Status& open();

        //! Called when this layer is added to a Map
        virtual void addedToMap(const Map*);

        //! Called when this layer is removed from a Map
        virtual void removedFromMap(const Map*);

        //! Node created by this model layer
        virtual osg::Node* getNode() const;

        //! Generate a cache ID for this layer
        virtual std::string getCacheID() const;

    protected:

        /** post-ctor initialization */
        virtual void init();


    protected:

        virtual ~ModelLayer();

        osg::ref_ptr<ModelSource>     _modelSource;
        osg::ref_ptr<MaskSource>      _maskSource;
        Revision                      _modelSourceRev;
        osg::ref_ptr<osg::Vec3dArray> _maskBoundary;
        osg::ref_ptr<CacheSettings>   _cacheSettings;
        osg::ref_ptr<osg::Group>      _root;

        typedef fast_map<UID, osg::ref_ptr<osg::Node> > Graphs;
        Graphs _graphs;

        mutable Threading::Mutex _mutex; // general-purpose mutex.

        void fireCallback(ModelLayerCallback::MethodPtr method);

        void setLightingEnabledNoLock(bool value);
    };

    typedef std::vector< osg::ref_ptr<ModelLayer> > ModelLayerVector;
}

#endif // OSGEARTH_MODEL_LAYER_H
