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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#ifndef OSGEARTHUTIL_SKY
#define OSGEARTHUTIL_SKY

#include <osgEarthUtil/Common>
#include <osgEarthUtil/Ephemeris>
#include <osgEarth/DateTime>
#include <osgEarth/GeoData>
#include <osgEarth/Config>
#include <osgEarth/SpatialReference>
#include <osg/Group>
#include <osg/Uniform>
#include <osg/View>
#include <osgDB/ReaderWriter>

namespace osgEarth {
    class MapNode;
}
namespace osgDB {
    class Options;
}

namespace osgEarth { namespace Util
{
    using namespace osgEarth;


    /**
     * Base Options structure for loading an environment node from
     * a plugin.
     */
    class SkyOptions : public DriverConfigOptions
    {
    public:
        enum CoordinateSystem 
        {
            COORDSYS_ECEF,
            COORDSYS_ECI
        };

        //! Coordinate system for whole-earth maps (default = ECEF)
        optional<int>& coordinateSystem() { return _coordsys; }
        const optional<int>& coordinateSystem() const { return _coordsys; }

        /** Time of day - Hours [0..24] component of DateTime */
        optional<float>& hours() { return _hours; }
        const optional<float>& hours() const { return _hours; }

        /** Ambient light level [0..1] */
        optional<float>& ambient() { return _ambient; }
        const optional<float>& ambient() const { return _ambient; }

    public:
        SkyOptions( const ConfigOptions& options =ConfigOptions() ) : DriverConfigOptions(options) {
            _coordsys.init(COORDSYS_ECEF);
            fromConfig(_conf);
        }
        virtual ~SkyOptions() { }
        virtual Config getConfig() const {
            Config conf = DriverConfigOptions::getConfig();
            conf.set("coordsys", _coordsys);
            conf.set("hours", _hours);
            conf.set("ambient", _ambient);
            return conf;
        }

    protected:
        virtual void mergeConfig( const Config& conf ) {
            ConfigOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf ) {
            conf.get("coordsys", _coordsys);
            conf.get("hours", _hours);
            conf.get("ambient", _ambient);
        }

        optional<int> _coordsys;
        optional<float> _hours;
        optional<float> _ambient;
    };


    /**
    * Interface for classes that provide sky, lighting, and other
    * environmental effect.
    */
    class OSGEARTHUTIL_EXPORT SkyNode : public osg::Group
    {
    public:
        //! Creates a SkyNode with the default implementation.
        static SkyNode* create();

        //! Creates a SkyNode with custom options.
        static SkyNode* create(const SkyOptions&);

        //! Creates a SkyNode from a raw driver name.
        static SkyNode* create(const std::string& driver);

    public: // DEPRECATED API

        //! @deprecated Use create()
        static SkyNode* create(osgEarth::MapNode* mapNode) {
            return create();
        }

        //! @deprecated Use create(SkyOptions&)
        static SkyNode* create(const SkyOptions& options, osgEarth::MapNode* mapNode) {
            return create(options);
        }

        //! @deprecated Use create(const std::string&)
        static SkyNode* create(const std::string& driver, osgEarth::MapNode* mapNode) {
            return create(driver);
        }

    protected:
        // CTOR (abstract base class)
        SkyNode();

        // CTOR (abstract base class)
        SkyNode(const SkyOptions& options);

        // protected DTOR (heap-only)
        virtual ~SkyNode();

    public:
        /**
         * The ephemeris reference point for projected maps. Not applicable
         * for geocentric maps. Setting this also informs the skynode that is
         * should operate in projected-map mode if possible.
         */
        void setReferencePoint(const GeoPoint& point);
        const GeoPoint& getReferencePoint() const { return *_refpoint; }

        /**
         * Gets/Sets the Ephemeris used to position the sun and the moon
         * based on date/time.
         */
        void setEphemeris(Ephemeris* ephemeris);
        const Ephemeris* getEphemeris() const;

        /**
         * Whether the sky lights its subgraph.
         */
        void setLighting(osg::StateAttribute::OverrideValue value);
        osg::StateAttribute::OverrideValue getLighting() const { return _lightingValue; }

        /**
         * Gets the date/time for which the environment is configured.
         * Pass in an optional View to get the date/time specific to
         * that View.
         */
        void setDateTime(const DateTime& dt);
        const DateTime& getDateTime() const { return _dateTime; }

        /** Whether the sun is visible */
        void setSunVisible(bool value);
        bool getSunVisible() const { return _sunVisible; }

        /** Whether the moon is visible */
        void setMoonVisible(bool value);
        bool getMoonVisible() const { return _moonVisible; }

        /** Whether the stars are visible */
        void setStarsVisible(bool value);
        bool getStarsVisible() const { return _starsVisible; }

        /** Whether the atmosphere is visible */
        void setAtmosphereVisible(bool value);
        bool getAtmosphereVisible() const { return _atmosphereVisible; }

        /** Access the osg::Light representing the sun */
        virtual osg::Light* getSunLight() const = 0;

        /** @deprecated. use getSunLight()->setAmbient instead */
        void setMinimumAmbient(const osg::Vec4f& ambient) { if (getSunLight()) getSunLight()->setAmbient(ambient); }
        /** @deprecated; use getSunLight()->getAmbient instead */
        const osg::Vec4 getMinimumAmbient() const { if (!getSunLight()) return osg::Vec4(0,0,0,0); return getSunLight()->getAmbient(); }

    public:

        /** Attaches this sky node to a view (placing a sky light). Optional */
        virtual void attach(osg::View* view, int lightNum) { }
        void attach(osg::View* view) { attach(view, 0); }

    protected:

        // impl class can override these events.
        virtual void onSetEphemeris() { }
        virtual void onSetDateTime() { }
        virtual void onSetReferencePoint() { }
        virtual void onSetMoonVisible() { }
        virtual void onSetStarsVisible() { }
        virtual void onSetSunVisible() { }
        virtual void onSetAtmosphereVisible() { }

    private:

        osg::ref_ptr<Ephemeris> _ephemeris;
        DateTime                _dateTime;
        bool                    _sunVisible;
        bool                    _moonVisible;
        bool                    _starsVisible;
        bool                    _atmosphereVisible;
        osg::Vec4f              _minimumAmbient;
        optional<GeoPoint>      _refpoint;

        osg::StateAttribute::OverrideValue _lightingValue;
        osg::ref_ptr<osg::Uniform>         _lightingUniform;

        void baseInit(const SkyOptions&);
    };


    /**
     * Base class for an sky driver plugin implementation.
     */
    class OSGEARTHUTIL_EXPORT SkyDriver : public osgDB::ReaderWriter
    {
    protected:
        const SkyOptions& getSkyOptions(const osgDB::Options* opt) const;
    };

    /**
     * Factory interface that sky extensions need to implement. Someday we will
     * convert the sky extensions into proper NodeKits, at which point this method
     * will probably be no longer necessary since you can just link with the library
     * and create the SkyNode normally.
     */
    class /*header-only*/ SkyNodeFactory
    {
    public:
        virtual SkyNode* createSkyNode() =0;
    };

} } // namespace osgEarth::Util

#endif // OSGEARTHUTIL_SKY
