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
#ifndef OSGEARTH_DRIVER_WCS_DRIVEROPTIONS
#define OSGEARTH_DRIVER_WCS_DRIVEROPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/TileSource>
#include <osgEarth/URI>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;

    class WCSOptions : public TileSourceOptions // NO EXPORT; header only
    {
    public: // properties

        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }

        optional<std::string>& identifier() { return _identifier; }
        const optional<std::string>& identifier() const { return _identifier; }

        optional<std::string>& format() { return _format; }
        const optional<std::string>& format() const { return _format; }

        optional<std::string>& elevationUnit() { return _elevationUnit; }
        const optional<std::string>& elevationUnit() const { return _elevationUnit; }

        optional<std::string>& srs() { return _srs; }
        const optional<std::string>& srs() const { return _srs; }

        optional<std::string>& rangeSubset() { return _rangeSubset; }
        const optional<std::string>& rangeSubset() const { return _rangeSubset; }

    public:
        WCSOptions( const TileSourceOptions& opt =TileSourceOptions() ) :
          TileSourceOptions( opt ),
              _elevationUnit( "m" )
          {
              setDriver( "wcs" );
              fromConfig( _conf );
          }

        /** dtor */
        virtual ~WCSOptions() { }

    public:
        Config getConfig() const {
            Config conf = TileSourceOptions::getConfig();
            conf.set("url", _url);
            conf.set("identifier", _identifier);
            conf.set("format", _format);
            conf.set("elevation_unit", _elevationUnit);
            conf.set("srs", _srs);
            conf.set("range_subset", _rangeSubset);
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
            conf.get("identifier", _identifier);
            conf.get("format", _format);
            conf.get("elevation_unit", _elevationUnit);
            conf.get("srs", _srs);
            conf.get("range_subset", _rangeSubset);
        }

        optional<URI>         _url;
        optional<std::string> _identifier, _format, _elevationUnit, _srs, _rangeSubset;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_WCS_DRIVEROPTIONS

