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
#ifndef OSGEARTH_DRIVER_CESIUMION_OPTIONS
#define OSGEARTH_DRIVER_CESIUMION_OPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/TileSource>
#include <osgEarth/URI>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;

    class CesiumIonOptions : public TileSourceOptions // NO EXPORT; header only
    {
    public:
        optional<URI>& server() { return _server; }
        const optional<URI>& server() const { return _server; }

        optional<std::string>& format() { return _format; }
        const optional<std::string>& format() const { return _format; }

        optional<std::string>& assetId() { return _assetId; }
        const optional<std::string>& assetId() const { return _assetId; }

        optional<std::string>& token() { return _token; }
        const optional<std::string>& token() const { return _token; }

    public:
        CesiumIonOptions( const TileSourceOptions& opt =TileSourceOptions() ) :
            TileSourceOptions( opt ),
            _server("https://api.cesium.com/"),
            _format("png")
        {
            setDriver( "cesiumion" );
            fromConfig( _conf );
        }

        virtual ~CesiumIonOptions() { }

    public:
        Config getConfig() const {
            Config conf = TileSourceOptions::getConfig();
            conf.set("server", _server);
            conf.set("asset_id", _assetId);
            conf.set("format", _format);
            conf.set("token", _token);
            return conf;
        }

    protected:
        void mergeConfig( const Config& conf ) {
            TileSourceOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf ) {
            conf.get( "server", _server );
            conf.get( "format", _format );
            conf.get( "asset_id", _assetId);
            conf.get( "token", _token);
        }

        optional<URI>         _server;
        optional<std::string> _format;
        optional<std::string> _assetId;
        optional<std::string> _token;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_CESIUMION_OPTIONS

