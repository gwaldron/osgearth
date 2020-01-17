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
#ifndef OSGEARTH_DRIVER_TMS_MAPLAYERFACTORY
#define OSGEARTH_DRIVER_TMS_MAPLAYERFACTORY 1

#include <osgEarth/Common>
#include <osgEarth/TileSource>
#include <osgEarth/URI>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;

    class TMSOptions : public TileSourceOptions // NO EXPORT; header only
    {
    public:
        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }

        optional<std::string>& tmsType() { return _tmsType; }
        const optional<std::string>& tmsType() const { return _tmsType; }

        optional<std::string>& format() { return _format; }
        const optional<std::string>& format() const { return _format; }

    public:
        TMSOptions( const TileSourceOptions& opt =TileSourceOptions() ) : TileSourceOptions( opt )
        {
            setDriver( "tms" );
            fromConfig( _conf );
        }

        TMSOptions( const std::string& inUrl ) : TileSourceOptions()
        {
            setDriver( "tms" );
            fromConfig( _conf );
            url() = inUrl;
        }

        /** dtor */
        virtual ~TMSOptions() { }

    public:
        static Config getMetadata() {
            return Config::readJSON( OE_MULTILINE(
                { "name" : "TMS (Tile Map Service)",
                  "properties": [
                    { "name": "url",      "description": "Location of the TMS repository", "type": "string", "default": "" },
                    { "name": "tms_type", "description": "Set to 'google' to invert the Y index", "type": "string", "default": "" },
                    { "name": "format",   "description": "Image format to assume (e.g. jpeg, png)", "type": "string", "default": "" }
                  ]
                }
            ) );
        }

        Config getConfig() const {
            Config conf = TileSourceOptions::getConfig();
            conf.set("url", _url);
            conf.set("tms_type", _tmsType);
            conf.set("format", _format);
            return conf;
        }

    protected:
        void mergeConfig( const Config& conf ) {
            TileSourceOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf ) {
            conf.get( "url", _url );
            conf.get( "format", _format );
            conf.get( "tms_type", _tmsType );
        }

        optional<URI>         _url;
        optional<std::string> _tmsType;
        optional<std::string> _format;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_TMS_MAPLAYERFACTORY

