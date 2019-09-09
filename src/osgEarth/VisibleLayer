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

#ifndef OSGEARTH_VISIBLE_LAYER_H
#define OSGEARTH_VISIBLE_LAYER_H 1

#include <osgEarth/Layer>

namespace osgEarth
{
    /**
     * Serializable configuration options for a VisibleLayer.
     */
    class OSGEARTH_EXPORT VisibleLayerOptions : public LayerOptions
    {
    public:
        enum Blend {
            BLEND_INTERPOLATE,
            BLEND_MODULATE
        };

    public:
        VisibleLayerOptions();
        VisibleLayerOptions(const ConfigOptions& options);

        /** dtor */
        virtual ~VisibleLayerOptions() { }

        /** Whether the layer is initially visible. */
        optional<bool>& visible() { return _visible; }
        const optional<bool>& visible() const { return _visible; }

        /** Opacity of the visible layer [0..1] */
        optional<float>& opacity() { return _opacity; }
        const optional<float>& opacity() const { return _opacity; }
        
        //! The initial minimum camera range at which this layer is visible.
        optional<float>& minVisibleRange() { return _minRange; }
        const optional<float>& minVisibleRange() const { return _minRange; }

        //! The initial maximum camera range at which this layer is visible.
        optional<float>& maxVisibleRange() { return _maxRange; }
        const optional<float>& maxVisibleRange() const { return _maxRange; }

        //! Distance (m) over which to ramp min and max range blending
        optional<float>& attenuationRange() { return _attenuationRange; }
        const optional<float>& attenuationRange() const { return _attenuationRange; }

        //! Blending function (default = interpolate)
        optional<Blend>& blend() { return _blend; }
        const optional<Blend>& blend() const { return _blend; }

    public:
        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );
        void fromConfig(const Config& conf);

    private:
        void setDefaults();       
        optional<bool> _visible;
        optional<float> _opacity;
        optional<float> _minRange;
        optional<float> _maxRange;
        optional<float> _attenuationRange;
        optional<Blend> _blend;
    };


    struct VisibleLayerCallback : public LayerCallback
    {
        virtual void onVisibleChanged(class VisibleLayer* layer) { }
        virtual void onOpacityChanged(class VisibleLayer* layer) { }
        virtual void onVisibleRangeChanged(class VisibleLayer* layer) { }
        typedef void (VisibleLayerCallback::*MethodPtr)(class VisibleLayer* layer);
    };


    /**
     * Base class for a layer supporting visibility control.
     */
    class OSGEARTH_EXPORT VisibleLayer : public Layer
    {
    public:
        META_Layer(osgEarth, VisibleLayer, VisibleLayerOptions, visible_layer);

    protected:
        // Construcable from a base class only.
        VisibleLayer(VisibleLayerOptions* ptr =0L);

        virtual ~VisibleLayer();

    public:
        /** Whether to draw this layer. */
        virtual void setVisible(bool value);
        bool getVisible() const;

        /** Opacity with which to draw this layer. */
        virtual void setOpacity(float value);
        float getOpacity() const;

        //! Minimum camera range at which this image layer is visible (if supported)
        float getMinVisibleRange() const;
        void setMinVisibleRange( float minVisibleRange );
        
        //! Maximum camera range at which this image layer is visible (if supported)
        float getMaxVisibleRange() const;
        void setMaxVisibleRange( float maxVisibleRange );
        
        //! Distance (m) over which to ramp min and max range blending
        float getAttenuationRange() const;
        void setAttenuationRange( float value );

    public: // Layer

        virtual const Status& open();

    protected: // Layer

        virtual void init();

        void fireCallback(VisibleLayerCallback::MethodPtr);

        //! Subclass can call this to install a default shader for applying
        //! opacity when the layer doesn't handle it itself
        void installDefaultOpacityShader();

    private: 
        osg::ref_ptr<osg::Uniform> _opacityU;
        osg::ref_ptr<osg::Uniform> _rangeU;
        void initializeBlending();
        void initializeMinMaxRangeOpacity();
    };

    typedef std::vector< osg::ref_ptr<VisibleLayer> > VisibleLayerVector;

} // namespace TerrainLayer

#endif // OSGEARTH_RENDER_LAYER_H
