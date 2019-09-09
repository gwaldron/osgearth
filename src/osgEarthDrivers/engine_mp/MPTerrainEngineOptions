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
#ifndef OSGEARTH_DRIVERS_MP_TERRAIN_ENGINE_OPTIONS
#define OSGEARTH_DRIVERS_MP_TERRAIN_ENGINE_OPTIONS 1

#include <osgEarth/Common>
#include <osgEarth/TerrainOptions>
#include <osgEarthSymbology/Color>
#include <osg/LOD>

namespace osgEarth { namespace Drivers { namespace MPTerrainEngine
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    /**
     * Options for configuring the MP Engine driver.
     */
    class MPTerrainEngineOptions : public TerrainOptions // NO EXPORT (header-only)
    {
    public:
        MPTerrainEngineOptions( const ConfigOptions& options =ConfigOptions() ) : TerrainOptions( options ),
            _skirtRatio        ( 0.05 ),
            _quickRelease      ( true ),
            _normalizeEdges    ( true ),
            _color             ( Color::White ),
            _smoothing         ( false ),
            _normalMaps        ( false ),
            _adaptivePolarRangeFactor( true )
         {
            setDriver( "mp" );
            fromConfig( _conf );
        }

        /** dtor */
        virtual ~MPTerrainEngineOptions() { }

    public:
        /** Ratio of terrain tile skirt height to tile radius */
        optional<float>& heightFieldSkirtRatio() { return _skirtRatio; }
        const optional<float>& heightFieldSkirtRatio() const { return _skirtRatio; }

        /** Whether to run a post-render process that releases GL objects as quickly as
          * possible, freeing up memory faster */
        optional<bool>& quickReleaseGLObjects() { return _quickRelease; }
        const optional<bool>& quickReleaseGLObjects() const { return _quickRelease; }

        /** Whether to average normal vectors on tile boundaries. Doing so reduces the
         *  the appearance of seams when using lighting, but requires extra CPU work. */
        optional<bool>& normalizeEdges() { return _normalizeEdges; }
        const optional<bool>& normalizeEdges() const { return _normalizeEdges; }

        /** The color of the globe surface where no images are applied */
        optional<Color>& color() { return _color; }
        const optional<Color>& color() const { return _color; }

        /**
         * Whether to create smooth transitions between elevation datasets at differing LODs
         * by inheriting low-resolution data from parent tiles and filling in the new data.
         * When smoothing is on, elevation may not be "true" in all locations, since the engine
         * will interpolate values from low LODs. Defaults to FALSE.
         */
        optional<bool>& elevationSmoothing() { return _smoothing; }
        const optional<bool>& elevationSmoothing() const { return _smoothing; }

        /** Whether to generate normal maps from elevation data. */
        optional<bool>& normalMaps() { return _normalMaps; }
        const optional<bool>& normalMaps() const { return _normalMaps; }

        /**
         * Whether to automatically adjust(reduce) the minTileRangeFactor with increase in
         * latitude. This prevents overtessellation in the polar regions. Only works with
         * a geocentric map. Defaults to false.
         */
        optional<bool>& adaptivePolarRangeFactor() { return _adaptivePolarRangeFactor; }
        const optional<bool>& adaptivePolarRangeFactor() const { return _adaptivePolarRangeFactor; }

    protected:
        virtual Config getConfig() const {
            Config conf = TerrainOptions::getConfig();
            conf.set( "skirt_ratio", _skirtRatio );
            conf.set( "quick_release_gl_objects", _quickRelease );
            conf.set( "normalize_edges", _normalizeEdges);
            conf.set( "color", _color );
            conf.set( "elevation_smoothing", _smoothing );
            conf.set( "normal_maps", _normalMaps );
            conf.set( "adaptive_polar_range_factor", _adaptivePolarRangeFactor);

            return conf;
        }

        virtual void mergeConfig( const Config& conf ) {
            TerrainOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf ) {
            conf.get( "skirt_ratio", _skirtRatio );
            conf.get( "quick_release_gl_objects", _quickRelease );
            conf.get( "normalize_edges", _normalizeEdges );
            conf.get( "color", _color );
            conf.get( "elevation_smoothing", _smoothing );
            conf.get( "normal_maps", _normalMaps );
            conf.get( "adaptive_polar_range_factor", _adaptivePolarRangeFactor);
       }

        optional<float>               _skirtRatio;
        optional<bool>                _quickRelease;
        optional<float>               _lodFallOff;
        optional<bool>                _normalizeEdges;
        optional<Color>               _color;
        optional<bool>                _smoothing;
        optional<bool>                _normalMaps;
        optional<bool>                _adaptivePolarRangeFactor;
    };

} } } // namespace osgEarth::Drivers::MPTerrainEngine

#endif // OSGEARTH_DRIVERS_MP_TERRAIN_ENGINE_OPTIONS
