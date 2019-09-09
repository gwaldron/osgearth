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
#ifndef OSGEARTH_DRIVER_TILEPACKAGE_DRIVEROPTIONS
#define OSGEARTH_DRIVER_TILEPACKAGE_DRIVEROPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/TileSource>
#include <osgEarth/URI>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;

    class TilePackageOptions : public TileSourceOptions // NO EXPORT; header only
    {
    public:
        /** URL of the MapService */
        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }        

    public:
        TilePackageOptions( const TileSourceOptions& opt =TileSourceOptions() ) : TileSourceOptions( opt )
        {
            setDriver( "tilepackage" );
            fromConfig( _conf );
        }

        /** dtor */
        virtual ~TilePackageOptions() { }

    public:
        Config getConfig() const {
            Config conf = TileSourceOptions::getConfig();
            conf.set("url", _url );
            return conf;
        }

        void mergeConfig( const Config& conf ) {
            TileSourceOptions::mergeConfig( conf );
            fromConfig( conf );
        }

        void fromConfig( const Config& conf ) {
            conf.get( "url", _url );
        }

    private:
        optional<URI>         _url;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_TILEPACKAGE_DRIVEROPTIONS

