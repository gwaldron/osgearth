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
#ifndef OSGEARTH_DRIVER_XYZ_OPTIONS
#define OSGEARTH_DRIVER_XYZ_OPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/TileSource>
#include <osgEarth/URI>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;

    class XYZOptions : public TileSourceOptions // NO EXPORT; header only
    {
    public:
        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }

        optional<bool>& invertY() { return _invertY; }
        const optional<bool>& invertY() const { return _invertY; }

        optional<std::string>& format() { return _format; }
        const optional<std::string>& format() const { return _format; }

        optional<std::string>& elevationEncoding() { return _elevationEncoding; }
        const optional<std::string>& elevationEncoding() const { return _elevationEncoding; }



    public:
        XYZOptions( const TileSourceOptions& opt =TileSourceOptions() ) : TileSourceOptions( opt )
        {
            setDriver( "xyz" );
            fromConfig( _conf );
        }

        XYZOptions( const std::string& inUrl ) : TileSourceOptions()
        {
            setDriver( "xyz" );
            fromConfig( _conf );
            url() = inUrl;
        }

        virtual ~XYZOptions() { }

    public:
        Config getConfig() const {
            Config conf = TileSourceOptions::getConfig();
            conf.set("url", _url);
            conf.set("format", _format);
            conf.set("invert_y", _invertY);
            conf.set("elevation_encoding", _elevationEncoding);
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
            conf.get( "invert_y", _invertY );
            conf.get( "elevation_encoding", _elevationEncoding );
        }

        optional<URI>         _url;
        optional<std::string> _format;
        optional<bool>        _invertY;
        optional<std::string> _elevationEncoding;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_XYZ_OPTIONS

