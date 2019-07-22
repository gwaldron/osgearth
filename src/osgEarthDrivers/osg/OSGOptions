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
#ifndef OSGEARTH_DRIVER_OSG_DRIVEROPTIONS
#define OSGEARTH_DRIVER_OSG_DRIVEROPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/TileSource>
#include <osgEarth/URI>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;

    class OSGOptions : public TileSourceOptions // NO EXPORT; header only
    {
    public:
        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }

        optional<bool>& convertLuminanceToRGBA() { return _lum2rgba; }
        const optional<bool>& convertLuminanceToRGBA() const { return _lum2rgba; }

        optional<bool>& addAlpha() { return _addAlpha; }
        const optional<bool>& addAlpha() const { return _addAlpha; }

    public:
        OSGOptions( const TileSourceOptions& opt =TileSourceOptions() ) :
            TileSourceOptions( opt ),
            _lum2rgba( false ),
            _addAlpha( false )
        {
            setDriver( "osg" );
            fromConfig( _conf );
        }

        /** dtor */
        virtual ~OSGOptions() { }

    public:
        Config getConfig() const {
            Config conf = TileSourceOptions::getConfig();
            conf.set("url", _url );
            conf.set("luminance_to_rgba", _lum2rgba);
            conf.set("add_alpha", _addAlpha);
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
            conf.get( "luminance_to_rgba", _lum2rgba );
            conf.get( "add_alpha", _addAlpha);
        }

        optional<URI>  _url;
        optional<bool> _lum2rgba;
        optional<bool> _addAlpha;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_OSG_DRIVEROPTIONS

