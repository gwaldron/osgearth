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
#ifndef OSGEARTH_DRIVER_TFS_FEATURE_SOURCE_OPTIONS
#define OSGEARTH_DRIVER_TFS_FEATURE_SOURCE_OPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/URI>
#include <osgEarthFeatures/FeatureSource>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;
    using namespace osgEarth::Features;

    /**
     * Options for the TFS feature driver.
     */
    class TFSFeatureOptions : public FeatureSourceOptions // NO EXPORT; header only
    {
    public:
        /** Base URL of the TFS service */
        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }
                
        /** Data format extension of TFS data (json, gml) */
        optional<std::string>& format() { return _format; }
        const optional<std::string>& format() const { return _format; }        

        optional<bool>& invertY() { return _invertY; }
        const optional<bool>& invertY() const { return _invertY; }

        optional<int>& minLevel() { return _minLevel; }
        const optional<int>& minLevel() const { return _minLevel; }

        optional<int>& maxLevel() { return _maxLevel; }
        const optional<int>& maxLevel() const { return _maxLevel; }

    public:
        TFSFeatureOptions( const ConfigOptions& opt =ConfigOptions() ) :
          FeatureSourceOptions( opt ),
          _format("json")
          {
            setDriver( "tfs" );            
            fromConfig( _conf );
        }

        virtual ~TFSFeatureOptions() { }

    public:
        Config getConfig() const {
            Config conf = FeatureSourceOptions::getConfig();
            conf.set( "url", _url ); 
            conf.set( "format", _format );
            conf.set( "invert_y", _invertY);
            conf.set( "min_level", _minLevel);
            conf.set( "max_level", _maxLevel);
            return conf;
        }

    protected:
        void mergeConfig( const Config& conf ) {
            FeatureSourceOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf ) {
            conf.get( "url", _url );
            conf.get( "format", _format );
            conf.get( "invert_y", _invertY );
            conf.get( "min_level", _minLevel);
            conf.get( "max_level", _maxLevel);
        }

        optional<URI>         _url;        
        optional<std::string> _format;
        optional<bool>        _invertY;
        optional<int>         _minLevel;
        optional<int>         _maxLevel;
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_TFS_FEATURE_SOURCE_OPTIONS

