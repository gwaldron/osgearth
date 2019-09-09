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
#ifndef OSGEARTH_UTIL_FLATTENING_LAYER
#define OSGEARTH_UTIL_FLATTENING_LAYER 1

#include <osgEarthUtil/Common>
#include <osgEarth/TileSource>
#include <osgEarth/ElevationLayer>
#include <osgEarth/ElevationPool>
#include <osgEarth/LayerListener>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/FeatureSourceLayer>
#include <osgEarthFeatures/ScriptEngine>
#include <osgEarthSymbology/StyleSheet>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;
    using namespace osgEarth::Features;
    using namespace osgEarth::Symbology;

    /**
     * Serializable options to configure a FlatteningLayer.
     */
    class OSGEARTHUTIL_EXPORT FlatteningLayerOptions : public ElevationLayerOptions
    {
    public:
        // constructor
        FlatteningLayerOptions(const ConfigOptions& co = ConfigOptions()) :
            ElevationLayerOptions(co)
        {
            _fill.init(false);
            _lineWidth.init(40);
            _bufferWidth.init(40);
            mergeConfig(_conf);
        }
        
        /** Name of the feature source layer to use for flattening. */
        optional<std::string>& featureSourceLayer() { return _featureSourceLayer; }
        const optional<std::string>& featureSourceLayer() const { return _featureSourceLayer; }

        /** Features that will drive the flattening process. */
        optional<FeatureSourceOptions>& featureSource() { return _featureSource; }
        const optional<FeatureSourceOptions>& featureSource() const { return _featureSource; }

        /** For line features, the width around the line to flatten. */
        optional<NumericExpression>& lineWidth() { return _lineWidth; }
        const optional<NumericExpression>& lineWidth() const { return _lineWidth; }

        /** Width of the buffer between the flattened terrain and the natural terrain,
            which will serve as a transition area. */
        optional<NumericExpression>& bufferWidth() { return _bufferWidth; }
        const optional<NumericExpression>& bufferWidth() const { return _bufferWidth; }

        //! Whether to write all samples (default=false) with source elev instead of
        //! writing NO_DATA_VALUE where no features exist
        optional<bool>& fill() { return _fill; }
        const optional<bool>& fill() const { return _fill; }

        StyleSheet::ScriptDef* getScript() const { return _script.get(); }

    public:
        Config getConfig() const
        {
            Config conf = ElevationLayerOptions::getConfig();
            conf.set("features",  _featureSource);
            conf.set("feature_source", _featureSourceLayer);
            conf.set("line_width", _lineWidth);
            conf.set("buffer_width", _bufferWidth);
            conf.set("fill", _fill);

            if ( _script.valid() )
            {
                Config scriptConf("script");

                if ( !_script->name.empty() )
                    scriptConf.set( "name", _script->name );
                if ( !_script->language.empty() )
                    scriptConf.set( "language", _script->language );
                if ( _script->uri.isSet() )
                    scriptConf.set( "url", _script->uri->base() );
                if ( !_script->profile.empty() )
                    scriptConf.set( "profile", _script->profile );
                else if ( !_script->code.empty() )
                    scriptConf.setValue(_script->code);

                conf.add( scriptConf );
            }

            return conf;
        }

    protected:
        void mergeConfig( const Config& conf )
        {
            URIContext uriContext = URIContext( conf.referrer() );

            conf.get("features",  _featureSource);
            conf.get("feature_source", _featureSourceLayer);
            conf.get("line_width", _lineWidth);
            conf.get("buffer_width", _bufferWidth);
            conf.get("fill", _fill);

            // TODO:  Separate out ScriptDef from Stylesheet and include it as a standalone class, along with this loading code.
            ConfigSet scripts = conf.children( "script" );
            for( ConfigSet::iterator i = scripts.begin(); i != scripts.end(); ++i )
            {
                _script = new StyleSheet::ScriptDef();

                // load the code from a URI if there is one:
                if ( i->hasValue("url") )
                {
                    _script->uri = URI( i->value("url"), uriContext );
                    OE_INFO << "Loading script from \"" << _script->uri->full() << std::endl;
                    _script->code = _script->uri->getString();
                }
                else
                {
                    _script->code = i->value();
                }

                // name is optional and unused at the moment
                _script->name = i->value("name");

                std::string lang = i->value("language");
                _script->language = lang.empty() ? "javascript" : lang;

                std::string profile = i->value("profile");
                _script->profile = profile;
            }
        }
        
    private:
        optional<FeatureSourceOptions>  _featureSource;
        optional<std::string> _featureSourceLayer;
        optional<NumericExpression> _lineWidth;
        optional<NumericExpression> _bufferWidth;
        optional<bool> _fill;
        osg::ref_ptr< StyleSheet::ScriptDef > _script;
    };

    /**
     * Elevation layer that overlays modified elevation samples intended to
     * flatten the terrain around vector features. The use case is to make
     * roads flat or prevent rivers and lakes from sloping with the terrain.
     */
    class OSGEARTHUTIL_EXPORT FlatteningLayer : public ElevationLayer
    {
    public:
        META_Layer(osgEarth, FlatteningLayer, FlatteningLayerOptions, flattened_elevation);

        // Create a layer with initial options.
        FlatteningLayer(const FlatteningLayerOptions& options = FlatteningLayerOptions());

        // Feature source layer to get a FeatureSource from
        void setFeatureSourceLayer(FeatureSourceLayer* layer);    
        
        // the feature source from which to read flattening geometry
        void setFeatureSource(FeatureSource* fs);

    public: // ElevationLayer

        virtual void init();

        // opens the layer and returns the status
        virtual const Status& open();

    protected: // ElevationLayer

        virtual void createImplementation(
            const TileKey& key,
            osg::ref_ptr<osg::HeightField>& hf,
            osg::ref_ptr<NormalMap>& normalMap,
            ProgressCallback* progres);

        //! called by the map when this layer is added
        virtual void addedToMap(const class Map*);

        //! called by the map when this layer is removed
        virtual void removedFromMap(const class Map*);

    protected:

        virtual ~FlatteningLayer();

    private:

        osg::ref_ptr<ElevationPool> _pool;
        osg::ref_ptr<FeatureSource> _featureSource;
        osg::ref_ptr<ScriptEngine> _scriptEngine;
        LayerListener<FlatteningLayer, FeatureSourceLayer> _featureLayerListener;
        osg::observer_ptr< const Map > _map;
    };

    REGISTER_OSGEARTH_LAYER(flattened_elevation, FlatteningLayer);

} } // namespace osgEarth::Util

#endif // OSGEARTH_UTIL_FLATTENING_LAYER
