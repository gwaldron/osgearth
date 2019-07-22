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
#ifndef OSGEARTH_DRIVER_SIMPLE_SKY_OPTIONS
#define OSGEARTH_DRIVER_SIMPLE_SKY_OPTIONS 1

#include <osgEarthUtil/Sky>
#include <osgEarth/URI>

namespace osgEarth { namespace SimpleSky
{
    using namespace osgEarth;
    using namespace osgEarth::Util;

    /**
     * Options for creating a simple sky.
     */
    class SimpleSkyOptions : public SkyOptions
    {
    public:
        SimpleSkyOptions(const ConfigOptions& options =ConfigOptions()) :
          SkyOptions(options),
          _atmosphericLighting (true),
          _exposure            (3.3f),
          _daytimeAmbientBoost (5.0f),
          _allowWireframe      (false),
          _starSize            (14.0f),
          _moonScale           (2.0f),
          _moonImageURI("moon_1024x512.jpg"),
		  _sunVisible(true),
          _moonVisible(true),
          _starsVisible(true),
          _atmosphereVisible(true)
        {
            setDriver( "simple" );
            fromConfig( _conf );
        }
        virtual ~SimpleSkyOptions() { }

    public: // properties

        /** Use advanced atmospheric lighting on the terrain (instead of simple shading) */
        optional<bool>& atmosphericLighting() { return _atmosphericLighting; }
        const optional<bool>& atmosphericLighting() const { return _atmosphericLighting; }

        /** Exposure factor for simulated HDR ground lighting. Default is in CTOR */
        optional<float>& exposure() { return _exposure; }
        const optional<float>& exposure() const { return _exposure; }

        /** Factor for boosting the ambient lighting of the sun during the daytime. Without
          * this, geometry like buildings or steep hillsides can be lit only based on their
          * normals and can appear too dark during the daytime.
          * 0=off, 2-10 are reasonable values. */
        optional<float>& daytimeAmbientBoost() { return _daytimeAmbientBoost; }
        const optional<float>& daytimeAmbientBoost() const { return _daytimeAmbientBoost; }

        /** replacement star definition file */
        optional<std::string>& starFile() { return _starFile; }
        const optional<std::string>& starFile() const { return _starFile; }

        //! Point size to use for stars (default=14)
        optional<float>& starSize() { return _starSize; }
        const optional<float>& starSize() const { return _starSize; }

        /** Whether to permit wireframe/point polygonmode rendering. Default is false. */
        optional<bool>& allowWireframe() { return _allowWireframe; }
        const optional<bool>& allowWireframe() const { return _allowWireframe; }

		/** Whether the Sun is visible. This has no effect on lighting. */
        optional<bool>& sunVisible() { return _sunVisible; }
        const optional<bool>& sunVisible() const { return _sunVisible; }

        /** Whether the Moon is visible */
        optional<bool>& moonVisible() { return _moonVisible; }
        const optional<bool>& moonVisible() const { return _moonVisible; }

        /** Whether the stars are visible */
        optional<bool>& starsVisible() { return _starsVisible; }
        const optional<bool>& starsVisible() const { return _starsVisible; }

        /** Whether the Earth's atmosphere is visible */
        optional<bool>& atmosphereVisible() { return _atmosphereVisible; }
        const optional<bool>& atmosphereVisible() const { return _atmosphereVisible; }

        /** Scale factor for the moon to compensate for apparent/expected size. Default=2.0 */
        optional<float>& moonScale() { return _moonScale; }
        const optional<float>& moonScale() const { return _moonScale; }

        /** URI of texture to load for moon */
        optional<URI>& moonImageURI() { return _moonImageURI; }
        const optional<URI>& moonImageURI() const { return _moonImageURI; }

    public:
        Config getConfig() const {
            Config conf = SkyOptions::getConfig();
            conf.set("atmospheric_lighting", _atmosphericLighting);
            conf.set("exposure", _exposure);
            conf.set("daytime_ambient_boost", _daytimeAmbientBoost);
            conf.set("star_file", _starFile);
            conf.set("star_size", _starSize);
            conf.set("allow_wireframe", _allowWireframe);
			conf.set("sun_visible", _sunVisible);
            conf.set("moon_visible", _moonVisible);
            conf.set("stars_visible", _starsVisible);
            conf.set("atmosphere_visible", _atmosphereVisible);
            conf.set("moon_scale", _moonScale);
            conf.set("moon_image", _moonImageURI);
            return conf;
        }

    protected:
        void mergeConfig( const Config& conf ) {
            SkyOptions::mergeConfig( conf );
            fromConfig(conf);
        }

    private:
        void fromConfig( const Config& conf ) {
            conf.get("atmospheric_lighting", _atmosphericLighting);
            conf.get("exposure", _exposure);
            conf.get("daytime_ambient_boost", _daytimeAmbientBoost);
            conf.get("star_file", _starFile);
            conf.get("star_size", _starSize);
            conf.get("allow_wireframe", _allowWireframe);
			conf.get("sun_visible", _sunVisible);
            conf.get("moon_visible", _moonVisible);
            conf.get("stars_visible", _starsVisible);
            conf.get("atmosphere_visible", _atmosphereVisible);
            conf.get("moon_scale", _moonScale);
            conf.get("moon_image", _moonImageURI);
        }

        optional<bool>        _atmosphericLighting;
        optional<float>       _exposure;
        optional<float>       _daytimeAmbientBoost;
        optional<std::string> _starFile;
        optional<float>       _starSize;
        optional<bool>        _allowWireframe;
		optional<bool>        _sunVisible;
        optional<bool>        _moonVisible;
        optional<bool>        _starsVisible;
        optional<bool>        _atmosphereVisible;
        optional<float>       _moonScale;
        optional<URI>         _moonImageURI;
    };

} } // namespace osgEarth::SimpleSky

#endif // OSGEARTH_DRIVER_SIMPLE_SKY_OPTIONS

