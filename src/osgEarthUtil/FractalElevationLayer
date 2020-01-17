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
#ifndef OSGEARTH_UTIL_FRACTAL_ELEVATION_LAYER
#define OSGEARTH_UTIL_FRACTAL_ELEVATION_LAYER 1

#include <osgEarthUtil/Common>
#include <osgEarth/TileSource>
#include <osgEarth/ElevationLayer>
#include <osgEarth/LayerListener>
#include <osgEarth/URI>
#include <osgEarth/LandCoverLayer>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;

    struct FractalElevationLayerLandCoverMapping
    {
        std::string className;
        optional<float> amplitude;
    };

    typedef std::map<std::string, FractalElevationLayerLandCoverMapping> FractalElevationLayerLandCoverMap;


    /**
     * Serializable options to configure a FractalElevationLayer.
     */
    class OSGEARTHUTIL_EXPORT FractalElevationLayerOptions : public ElevationLayerOptions
    {

    public:
        // constructor
        FractalElevationLayerOptions(const ConfigOptions& co = ConfigOptions());

        //! Simplex noise frequency
        optional<float>& frequency() { return _frequency; }
        const optional<float>& frequency() const { return _frequency; }

        //! Simplex noise persistence
        optional<float>& persistence() { return _persistence; }
        const optional<float>& persistence() const { return _persistence; }
        
        //! Simplex noise lacunarity
        optional<float>& lacunarity() { return _lacunarity; }
        const optional<float>& lacunarity() const { return _lacunarity; }
        
        //! Reference LOD
        optional<unsigned>& baseLOD() { return _baseLOD; }
        const optional<unsigned>& baseLOD() const { return _baseLOD; }

        //! Maximum change in elevation (total)
        optional<float>& amplitude() { return _amplitude; }
        const optional<float>& amplitude() const { return _amplitude; }

        //! Mappings from land cover classes to amplitude values (optional)
        FractalElevationLayerLandCoverMap& landCoverMap() { return _lcMap; }
        const FractalElevationLayerLandCoverMap& landCoverMap() const { return _lcMap; }
        
        //! URI of a noise texture to use to augment to simplex noise
        optional<URI>& noiseImageURI() { return _noiseImageURI; }
        const optional<URI>& noiseImageURI() const { return _noiseImageURI; }

    public:
        Config getConfig() const;

    protected:
        void mergeConfig(const Config& conf) {
            ElevationLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }

        void fromConfig(const Config& conf);
        
    private:
        optional<float> _frequency;
        optional<float> _persistence;
        optional<float> _lacunarity;
        optional<unsigned> _octaves;
        optional<unsigned> _baseLOD;
        optional<float> _amplitude;
        optional<URI> _noiseImageURI;
        FractalElevationLayerLandCoverMap _lcMap;

    };


    /**
     * Elevation layer that adds fractal noise offset values.
     */
    class OSGEARTHUTIL_EXPORT FractalElevationLayer : public ElevationLayer
    {
    public:
        META_Layer(osgEarth, FractalElevationLayer, FractalElevationLayerOptions, fractal_elevation);

        //! Create a blank layer to be configurated through options().
        FractalElevationLayer();

        //! Create a layer with initial options.
        FractalElevationLayer(const FractalElevationLayerOptions& options);

    public: // ElevationLayer

        // opens the layer and returns the status
        virtual const Status& open();

        virtual void init();

    protected: // Layer

        // called by the map when this layer is added/removed
        virtual void addedToMap(const class Map*);

        virtual void removedFromMap(const class Map*);

        virtual void createImplementation(
            const TileKey& key,
            osg::ref_ptr<osg::HeightField>& out_hf,
            osg::ref_ptr<NormalMap>& out_normalMap,
            ProgressCallback* progress);

    protected:

        virtual ~FractalElevationLayer();

        osg::observer_ptr<LandCoverLayer> _landCover;
        osg::observer_ptr<LandCoverDictionary> _landCoverDict;

        bool _debug;
        osg::ref_ptr<osg::Image> _noiseImage1;
        osg::ref_ptr<osg::Image> _noiseImage2;

        const FractalElevationLayerLandCoverMapping* getMapping(const LandCoverClass*) const;
    };

} } // namespace osgEarth::Util

#endif // OSGEARTH_UTIL_FRACTAL_ELEVATION_LAYER
