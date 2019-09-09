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
#ifndef OSGEARTH_UTIL_SIMPLE_OCEAN_LAYER
#define OSGEARTH_UTIL_SIMPLE_OCEAN_LAYER 1

#include <osgEarthUtil/Common>
#include <osgEarth/Layer>
#include <osgEarth/LayerListener>
#include <osgEarth/URI>
#include <osgEarthUtil/Ocean>
#include <osgEarthSymbology/Color>

namespace osgEarth {
    class ImageLayer;
}

namespace osgEarth { namespace Util
{
    using namespace osgEarth::Symbology;

    /**
     * Serializable configuration options for the SimpleOceanLayer.
     */
    class OSGEARTHUTIL_EXPORT SimpleOceanLayerOptions : public VisibleLayerOptions
    {
    public:
        /** Color of the ocean surface */
        optional<Color>& color() { return _color; }
        const optional<Color>& color() const { return _color; }

        /** Maximum altitude at which the ocean is visible */
        optional<float>& maxAltitude() { return _maxAltitude; }
        const optional<float>& maxAltitude() const { return _maxAltitude; }

        /** Name of a Map Layer to use as an ocean mask. */
        optional<std::string>& maskLayer() { return _maskLayer; }
        const optional<std::string>& maskLayer() const { return _maskLayer; }

        /** Whether to sample the terrain bathymetry and only draw the ocean
        where it's negative (default = true) */
        optional<bool>& useBathymetry() { return _useBathymetry; }
        const optional<bool>& useBathymetry() const { return _useBathymetry; }

        /** URI of surface texture to apply */
        optional<URI>& texture() { return _textureURI; }
        const optional<URI>& texture() const { return _textureURI; }

        /** LOD at which to apply surface texture 1:1 (default = 13) */
        optional<unsigned>& textureLOD() { return _textureLOD; }
        const optional<unsigned>& textureLOD() const { return _textureLOD; }

    public:
        SimpleOceanLayerOptions(const ConfigOptions& op =ConfigOptions()) : VisibleLayerOptions(op) {
            _color.init(Color("#1D2C4FFF"));
            _maxAltitude.init(1500000.f);
            _useBathymetry.init(true);
            _textureLOD.init(13u);
            mergeConfig(_conf);
        }

        void mergeConfig(const Config& conf) {
            conf.get("color", _color);
            conf.get("max_altitude", _maxAltitude);
            conf.get("mask_layer", _maskLayer);
            conf.get("use_bathymetry", _useBathymetry);
            conf.get("texture", _textureURI);
            conf.get("texture_lod", _textureLOD);
        }

        Config getConfig() const {
            Config conf = VisibleLayerOptions::getConfig();
            conf.set("color", _color);
            conf.set("max_altitude", _maxAltitude);
            conf.set("mask_layer", _maskLayer);
            conf.set("use_bathymetry", _useBathymetry);
            conf.set("texture", _textureURI);
            conf.set("texture_lod", _textureLOD);
            return conf;
        }

    private:
        optional<Color> _color;
        optional<float> _maxAltitude;
        optional<std::string> _maskLayer;
        optional<bool> _useBathymetry;
        optional<URI> _textureURI;
        optional<unsigned> _textureLOD;
    };


    /**
     * A Rex map layer that renders a simple ocean surface.
     * This layer requires that the map include bathymetric data (ocean floor).
     */
    class OSGEARTHUTIL_EXPORT SimpleOceanLayer : public VisibleLayer
    {
    public:
        META_Layer(osgEarth, SimpleOceanLayer, SimpleOceanLayerOptions, simple_ocean);

        /** Constructs a new ocean layer */
        SimpleOceanLayer();

        /** Constructs a new layer from an options structure. */
        SimpleOceanLayer(const SimpleOceanLayerOptions& options);

    public:

        /** Ocean surface color (including transparency in the alpha channel) */
        void setColor(const Color& color);
        const Color& getColor() const;

        /** Maximum altitude at which the ocean layer is visible */
        void setMaxAltitude(float altitude_m);
        float getMaxAltitude() const;

        /** Sets a masking layer, or pass NULL to clear the masking layer. Returns true upon success */
        void setMaskLayer(const ImageLayer* layer);

    public: // Ocean
        
        void setSeaLevel(float seaLevel);
        float getSeaLevel() const;

    public: // Layer

        /** callback that ensures proper culling */
        void modifyTileBoundingBox(const TileKey& key, osg::BoundingBox& box) const;
        
        void setTerrainResources(TerrainResources* res);

    protected: // Layer

        virtual void addedToMap(const class Map*);

        virtual void removedFromMap(const class Map*);

        virtual void init();

    protected:

        virtual ~SimpleOceanLayer() { }

    private:

        LayerListener<SimpleOceanLayer, const ImageLayer> _layerListener;

        float _seaLevel;

        TextureImageUnitReservation _texReservation;
    };
    

} } // namespace osgEarth::Util

#endif // OSGEARTH_UTIL_SIMPLE_OCEAN_LAYER
