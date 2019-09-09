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
#ifndef OSGEARTH_PROCEDURAL_GroundCover
#define OSGEARTH_PROCEDURAL_GroundCover 1

#include "Export"
#include <osgEarth/Config>
#include <osgEarth/ImageLayer>
#include <osgEarth/LandCoverLayer>
#include <osgEarthSymbology/ResourceLibrary>
#include <osgEarthSymbology/Symbol>
#include <osg/BoundingBox>
#include <osg/Shader>

namespace osgDB {
    class Options;
}
namespace osgEarth {
    class Map;
}

namespace osgEarth { namespace Splat
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    class Coverage;
    class SplatCoverageLegend;


    class OSGEARTHSPLAT_EXPORT GroundCoverBiomeOptions : public ConfigOptions
    {
    public:
        GroundCoverBiomeOptions(const ConfigOptions& conf = ConfigOptions()) : ConfigOptions(conf) {
            fromConfig( _conf );
        }

        /** Name of the biome classes to use. This is one or more class names from the legend,
          * separated by whitespace. e.g.: "forest grassland swamp" */
        optional<std::string>& biomeClasses() { return _biomeClasses; }
        const optional<std::string>& biomeClasses() const { return _biomeClasses; }

        /** Symbology used to conigure rendering in this biome */
        SymbolVector& symbols() { return _symbols; }
        const SymbolVector& symbols() const { return _symbols; }

    protected:
        optional<std::string> _biomeClasses;
        SymbolVector _symbols;

    public:    
        void fromConfig(const Config& conf);
        Config getConfig() const;
        virtual void mergeConfig( const Config& conf ) {
            ConfigOptions::mergeConfig( conf );
            fromConfig( conf );
        }
    };
    typedef std::vector<GroundCoverBiomeOptions> GroundCoverBiomeOptionsVector;


    class OSGEARTHSPLAT_EXPORT GroundCoverOptions : public ConfigOptions
    {
    public:
        GroundCoverOptions(const ConfigOptions& conf = ConfigOptions());

        /** Name of the land cover layer */
        optional<std::string>& name() { return _name; }
        const optional<std::string>& name() const { return _name; }

        /** Equivalent terrain LOD at which this layer will render */
        optional<unsigned>& lod() { return _lod; }
        const optional<unsigned>& lod() const { return _lod; }

        /** Maximum viewing distance from camera */
        optional<float>& maxDistance() { return _maxDistance; }
        const optional<float>& maxDistance() const { return _maxDistance; }

        /** Wind speed [0..1] */
        optional<float>& wind() { return _wind; }
        const optional<float>& wind() const { return _wind; }

        /** Metric that controls the number of land cover instances will render
          * in a given area. [1..5] */
        optional<float>& density() { return _density; }
        const optional<float>& density() const { return _density; }

        /** Percentage of land that this layer's instances will cover [0..1]. Lower
          * values will result is more "patchiness" of placement. */
        optional<float>& fill() { return _fill; }
        const optional<float>& fill() const { return _fill; }

        /** Brightness factor for rendering [1..], 1 = default */
        optional<float>& brightness() { return _brightness; }
        const optional<float>& brightness() const { return _brightness; }

        /** Contrast factor for rendering [0..], 0 = default */
        optional<float>& contrast() { return _contrast; }
        const optional<float>& contrast() const { return _contrast; }

        /** Biomes comprising this layer. */
        GroundCoverBiomeOptionsVector& biomes() { return _biomes; }
        const GroundCoverBiomeOptionsVector& biomes() const { return _biomes; }

    public:
        virtual Config getConfig() const;
    protected:
        virtual void mergeConfig(const Config& conf) {
            ConfigOptions::mergeConfig(conf);
            fromConfig(conf);
        }
        void fromConfig(const Config& conf);

    protected:
        optional<std::string> _name;
        optional<unsigned> _lod;
        optional<float> _maxDistance;
        optional<float> _density;
        optional<float> _fill;
        optional<float> _wind;
        optional<float> _brightness;
        optional<float> _contrast;
        GroundCoverBiomeOptionsVector _biomes;
    };

    //! Base class for any ground cover object
    class OSGEARTHSPLAT_EXPORT GroundCoverObject : public osg::Referenced
    {
    public:
        enum Type {
            TYPE_BILLBOARD
        };
        virtual Type getType() const =0;
    };

    //! Collection of ground cover objects
    typedef std::vector<osg::ref_ptr<GroundCoverObject> > GroundCoverObjects;

    /**
     * Billboard object - camera-facing side image coupled with an optional
     * top-down image.
     */
    class OSGEARTHSPLAT_EXPORT GroundCoverBillboard : public GroundCoverObject
    {
    public:
        GroundCoverBillboard(osg::Image* sideImage, osg::Image* topImage, float width, float height)
            : _sideImage(sideImage), _topImage(topImage), _width(width), _height(height) { }

        virtual Type getType() const { return TYPE_BILLBOARD; }

        osg::ref_ptr<osg::Image> _sideImage;
        osg::ref_ptr<osg::Image> _topImage;
        float                    _width;
        float                    _height;
    };

    /**
     * A Biome is a collection of ground cover objects corresponding
     * to a set of land cover classes. For example, the "forest" biome
     * might map to three different tree billboards.
     */
    class OSGEARTHSPLAT_EXPORT GroundCoverBiome : public osg::Referenced
    {
    public:
        GroundCoverBiome() : _code(0) { }

        /** classification names for this biome (space separated) */
        void setClasses( const std::string& value ) { _classNames = value; }
        const std::string& getClasses() const { return _classNames; }

        /** Objects that may be used to render land cover in this biome */
        GroundCoverObjects& getObjects() { return _objects; }
        const GroundCoverObjects& getObjects() const { return _objects; }

    protected:
        virtual ~GroundCoverBiome() { }

        std::string    _classNames;
        int            _code;
        GroundCoverObjects _objects;

    public:
        typedef std::map<URI, osg::ref_ptr<osg::Image> > ImageCache;
        bool configure(const ConfigOptions& conf, const osgDB::Options* dbo, ImageCache& cache);
    };

    typedef std::vector< osg::ref_ptr<GroundCoverBiome> > GroundCoverBiomes;


    /**
     * Interface for controlling ground cover appearance.
     */
    class OSGEARTHSPLAT_EXPORT GroundCover : public osg::Referenced
    {
    public:
        GroundCover(const GroundCoverOptions& options);
        
        GroundCoverOptions& options() { return _options; }
        const GroundCoverOptions& options() const { return _options; }

        /** Name of this layer */
        //void setName(const std::string& name) { _name = name; }
        const std::string& getName() const { return options().name().get(); }

        /** Biomes comprising this layer. */
        GroundCoverBiomes& getBiomes() { return _biomes; }
        const GroundCoverBiomes& getBiomes() const { return _biomes; }

        //! Wind factor
        void setWind(float wind);
        float getWind() const;

        //! Density
        void setDensity(float density);
        float getDensity() const;

        //! Fill
        void setFill(float fill);
        float getFill() const;

        //! Max draw distance
        void setMaxDistance(float distance);
        float getMaxDistance() const;

        //! Brightness
        void setBrightness(float value);
        float getBrightness() const;

        //! Constrast
        void setContrast(float value);
        float getContrast() const;

    public:

        //! Total number of individual objects this class can render
        int getTotalNumObjects() const;

        /** Creates the shader that contains the GLSL APOI for accessing this layer's information */
        osg::Shader* createShader() const;

        /** Creates the shader that resolves land cover information into billboard data. */
        //osg::Shader* createPredicateShader(const Coverage*) const;
        osg::Shader* createPredicateShader(LandCoverDictionary*, LandCoverLayer*) const;

        /** Builds the texture object containing all the data for this layer. */
        osg::Texture* createTexture() const;

        /** The stateset containing the shaders and state for rendering this layer. */
        osg::StateSet* getOrCreateStateSet();

        osg::StateSet* getStateSet() const { return _stateSet.get(); }

        void resizeGLObjectBuffers(unsigned maxSize);

        void releaseGLObjects(osg::State* state) const;

    protected:
        virtual ~GroundCover() { }

        GroundCoverOptions _options;
        GroundCoverBiomes _biomes;

        osg::ref_ptr<osg::StateSet> _stateSet;

    public:
        bool configure(const osgDB::Options* readOptions);
    };

} } // namespace osgEarth::Splat

OSGEARTH_SPECIALIZE_CONFIG(osgEarth::Splat::GroundCoverOptions);
OSGEARTH_SPECIALIZE_CONFIG(osgEarth::Splat::GroundCoverBiomeOptions);

#endif // OSGEARTH_PROCEDURAL_GroundCover
