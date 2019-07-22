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
#ifndef OSGEARTH_DRIVER_WMS_DRIVEROPTIONS
#define OSGEARTH_DRIVER_WMS_DRIVEROPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/TileSource>


namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;

    class WMSOptions : public TileSourceOptions // NO EXPORT; header only
    {
    public:
        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }

        optional<URI>& capabilitiesUrl() { return _capabilitiesUrl; }
        const optional<URI>& capabilitiesUrl() const { return _capabilitiesUrl; }

        optional<URI>& tileServiceUrl() { return _tileServiceUrl; }
        const optional<URI>& tileServiceUrl() const { return _tileServiceUrl; }

        optional<std::string>& layers() { return _layers; }
        const optional<std::string>& layers() const { return _layers; }

        optional<std::string>& style() { return _style; }
        const optional<std::string>& style() const { return _style; }

        optional<std::string>& format() { return _format; }
        const optional<std::string>& format() const { return _format; }

        optional<std::string>& wmsFormat() { return _wmsFormat; }
        const optional<std::string>& wmsFormat() const { return _wmsFormat; }

        optional<std::string>& wmsVersion() { return _wmsVersion; }
        const optional<std::string>& wmsVersion() const { return _wmsVersion; }

        optional<std::string>& elevationUnit() { return _elevationUnit; }
        const optional<std::string>& elevationUnit() const { return _elevationUnit; }

        optional<std::string>& srs() { return _srs; }
        const optional<std::string>& srs() const { return _srs; }

        optional<std::string>& crs() { return _crs; }
        const optional<std::string>& crs() const { return _crs; }

        optional<bool>& transparent() { return _transparent; }
        const optional<bool>& transparent() const { return _transparent; }

        optional<std::string>& times() { return _times; }
        const optional<std::string>& times() const { return _times; }

        optional<double>& secondsPerFrame() { return _secondsPerFrame; }
        const optional<double>& secondsPerFrame() const { return _secondsPerFrame; }

    public:
        WMSOptions( const TileSourceOptions& opt =TileSourceOptions() ) : TileSourceOptions( opt ),
            _wmsVersion( "1.1.1" ),
            _elevationUnit( "m" ),
            _transparent( true ),
            _secondsPerFrame( 1.0 )
        {
            setDriver( "wms" );
            fromConfig( _conf );
        }

        /** dtor */
        virtual ~WMSOptions() { }

    public:
        Config getConfig() const {
            Config conf = TileSourceOptions::getConfig();
            conf.set("url", _url);
            conf.set("capabilities_url", _capabilitiesUrl);
            conf.set("tile_service_url", _tileServiceUrl);
            conf.set("layers", _layers);
            conf.set("style", _style);
            conf.set("format", _format);
            conf.set("wms_format", _wmsFormat);
            conf.set("wms_version", _wmsVersion);
            conf.set("elevation_unit", _elevationUnit);
            conf.set("srs", _srs);
            conf.set("crs", _crs);
            conf.set("transparent", _transparent);
            conf.set("times", _times);
            conf.set("seconds_per_frame", _secondsPerFrame );
            return conf;
        }

    protected:
        void mergeConfig( const Config& conf ) {
            TileSourceOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf ) {
            conf.get("url", _url);
            conf.get("capabilities_url", _capabilitiesUrl);
            conf.get("tile_service_url", _tileServiceUrl);
            conf.get("layers", _layers);
            conf.get("style", _style);
            conf.get("format", _format);
            conf.get("wms_format", _wmsFormat);
            conf.get("wms_version", _wmsVersion);
            conf.get("elevation_unit", _elevationUnit);
            conf.get("srs", _srs);
            conf.get("crs", _crs);
            conf.get("transparent", _transparent);
            conf.get("times", _times);
            conf.get("time", _times); // alternative
            conf.get("seconds_per_frame", _secondsPerFrame );
        }

        optional<URI>         _url;
        optional<URI>         _capabilitiesUrl;
        optional<URI>         _tileServiceUrl;
        optional<std::string> _layers;
        optional<std::string> _style;
        optional<std::string> _format;
        optional<std::string> _wmsFormat;
        optional<std::string> _wmsVersion;
        optional<std::string> _elevationUnit;
        optional<std::string> _srs;
        optional<std::string> _crs;
        optional<bool>        _transparent;
        optional<std::string> _times;
        optional<double>      _secondsPerFrame;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_WMS_DRIVEROPTIONS

