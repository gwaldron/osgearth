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
#ifndef OSGEARTH_DRIVER_WFS_FEATURE_SOURCE_OPTIONS
#define OSGEARTH_DRIVER_WFS_FEATURE_SOURCE_OPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/URI>
#include <osgEarthFeatures/FeatureSource>

namespace osgEarth { namespace Drivers
{
    using namespace osgEarth;
    using namespace osgEarth::Features;

    /**
     * Options for the WFS driver.
     */
    class WFSFeatureOptions : public FeatureSourceOptions // NO EXPORT; header only
    {
    public:
        /** Base URL of the WFS service (not including any WFS parameters) */
        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }

        /** WFS typename parameter (see WFS spec) */
        optional<std::string>& typeName() { return _typename; }
        const optional<std::string>& typeName() const { return _typename; }

        /** Cap on the number of features that the WFS service will return on a single request */
        optional<unsigned int>& maxFeatures() { return _maxFeatures; }
        const optional<unsigned int>& maxFeatures() const { return _maxFeatures;}

        /** Response format to request (geojson, gml) */
        optional<std::string>& outputFormat() { return _outputFormat; }
        const optional<std::string>& outputFormat() const { return _outputFormat; }

        /** Explicitly disable a "TFS" tiled feature source queries */
        optional<bool>& disableTiling() { return _disableTiling; }
        const optional<bool>& disableTiling() const { return _disableTiling;}

        /** The number of map units to buffer bounding box requests with 
         *  to ensure that enough data is returned.  This is useful when rendering buffered lines
         *  using the AGGLite driver.         
         */
        optional<double>& buffer() { return _buffer;}
        const optional<double>& buffer() const { return _buffer;}



    public:
        WFSFeatureOptions( const ConfigOptions& opt =ConfigOptions() ) :
          FeatureSourceOptions( opt ),
          _buffer( 0 )
        {
            setDriver( "wfs" );
            fromConfig( _conf );            
        }

        virtual ~WFSFeatureOptions() { }

    public:
        Config getConfig() const {
            Config conf = FeatureSourceOptions::getConfig();
            conf.set( "url", _url ); 
            conf.set( "typename", _typename );
            conf.set( "outputformat", _outputFormat);
            conf.set( "maxfeatures", _maxFeatures );
            conf.set( "disable_tiling", _disableTiling );
            conf.set( "request_buffer", _buffer);

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
            conf.get( "geometry_profile", _geometryProfileConf );
            conf.get( "typename", _typename);
            conf.get( "outputformat", _outputFormat );
            conf.get( "maxfeatures", _maxFeatures );
            conf.get( "disable_tiling", _disableTiling);
            conf.get( "request_buffer", _buffer);            
        }

        optional<URI>         _url;        
        optional<std::string> _typename;
        optional<Config>      _geometryProfileConf;
        optional<std::string> _outputFormat;
        optional<unsigned>    _maxFeatures;            
        optional<bool>    _disableTiling;            
        optional<double>  _buffer;            
    };

} } // namespace osgEarth::Drivers

#endif // OSGEARTH_DRIVER_WFS_FEATURE_SOURCE_OPTIONS

