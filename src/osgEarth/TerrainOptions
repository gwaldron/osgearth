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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#ifndef OSGEARTH_TERRAIN_OPTIONS_H
#define OSGEARTH_TERRAIN_OPTIONS_H 1

#include <osgEarth/Common>
#include <osgEarth/Config>
#include <osgEarth/GeoCommon>
#include <osg/Texture>
#include <osg/LOD>

namespace osgEarth
{    
    /**
     * Base class for the configuration for a terrain engine driver.
     */
    class OSGEARTH_EXPORT TerrainOptions : public DriverConfigOptions
    {
    public:
        TerrainOptions( const ConfigOptions& options =ConfigOptions() );

        /** dtor */
        virtual ~TerrainOptions() { }
       
        /**
         * Sets or gets the scale factor for height-field values.
         * Default = 1.0
         */
        optional<float>& verticalScale() { return _verticalScale; }
        const optional<float>& verticalScale() const { return _verticalScale; }

        /**
         * Sets or gets the offset for height-field values.
         * Default = 0.0
         */
        optional<float>& verticalOffset() { return _verticalOffset; }
        const optional<float>& verticalOffset() const { return _verticalOffset; }

        /**
         * Size of each dimension of each terrain tile, in verts. Ideally this 
         * will be a power of 2 plus 1, i.e.: a number X with that X = (2^Y)+1
         * where Y is an integer >= 1.
         *
         * Default = 17. 
         */
        optional<int>& tileSize() { return _tileSize; }
        const optional<int>& tileSize() const { return _tileSize; }

        /**
         * The minimum tile LOD range as a factor of the tile's radius.
         * Default = 7.0.
         */
        optional<float>& minTileRangeFactor() { return _minTileRangeFactor; }
        const optional<float>& minTileRangeFactor() const { return _minTileRangeFactor; }

        /**
         * Distance (m) over which to ramp min_range and max_range blending; Default = 0.
         * @deprecated (remove after 2.10) Use VisibleLayer::attenuation instead.
         */
        optional<float>& attenuationDistance() { return _attenuationDistance; }
        const optional<float>& attenuationDistance() const { return _attenuationDistance; }

        /**
         * Whether cluster culling is enabled on terrain tiles (default = true)
         */
        optional<bool>& clusterCulling() { return _clusterCulling; }
        const optional<bool>& clusterCulling() const { return _clusterCulling; }

        /**
         * The maximum level of detail to which the terrain should subdivide. If you leave this
         * unset, the terrain will subdivide until the map layers stop providing data (default
         * behavior). If you set a value, the terrain will stop subdividing at the specified LOD
         * even if higher-resolution data is available. (It still might stop subdividing before it
         * reaches this level if data runs out.)
         */
        optional<unsigned>& maxLOD() { return _maxLOD; }
        const optional<unsigned >& maxLOD() const { return _maxLOD; }

        /**
         * The minimum level of detail to which the terrain should subdivide (no matter what).
         * If you leave this unset, the terrain will subdivide until the map layers
         * stop providing data (default behavior). If you set a value, the terrain will subdivide
         * to the specified LOD no matter what (and may continue farther if higher-resolution
         * data is available).
         */
        optional<unsigned>& minLOD() { return _minLOD; }
        const optional<unsigned>& minLOD() const { return _minLOD; }

        /**
         * The lowest LOD to display. By default, the terrain begins at LOD 0. Set this
         * to start the terrain tile mesh at a higher LOD. Don't set this TOO high though
         * or you will run into memory problems.
         */
        optional<unsigned>& firstLOD() { return _firstLOD; }
        const optional<unsigned>& firstLOD() const { return _firstLOD; }

        /**
         * Whether to explicity enable or disable GL lighting on the map node.
         */
        optional<bool>& enableLighting() { return _enableLighting; }
        const optional<bool>& enableLighting() const { return _enableLighting; }
            
        /**
         * Whether to enable blending on the terrain. Default is true.
         */
        optional<bool>& enableBlending() { return _enableBlending; }
        const optional<bool>& enableBlending() const { return _enableBlending; }

        /**
         * Whether to compress the normal map
         */
        optional<bool>& compressNormalMaps() { return _compressNormalMaps; }
        const optional<bool>& compressNormalMaps() const { return _compressNormalMaps; }

        /**
         * Whether to enable the "fast path" for spherical Mercator image layers
         * Default = true. If enabled, Mercator image tiles will be rendered on a
         * geocentric map with no reprojection. The trade-off is higher texture
         * memory usage and NPOT texture usage.
         * @deprecated (remove after 2.10)
         */
        optional<bool>& enableMercatorFastPath() { return _mercatorFastPath; }
        const optional<bool>& enableMercatorFastPath() const { return _mercatorFastPath; }

        /**
         * The minimum level of detail at which to generate elevation-based normal maps,
         * assuming normal maps have been activated. This mitigates the overhead of 
         * calculating normal maps for very high altitude scenes where they are no
         * of much use. Default is 0 (zero).
         */
        optional<unsigned>& minNormalMapLOD() { return _minNormalMapLOD; }
        const optional<unsigned>& minNormalMapLOD() const { return _minNormalMapLOD; }

        /**
         * Whether the terrain engine will be using GPU tessellation shaders.
         */
        optional<bool>& gpuTessellation() { return _gpuTessellation; }
        const optional<bool>& gpuTessellation() const { return _gpuTessellation; }

        /**
         * debugging mode
         */
        optional<bool>& debug() { return _debug; }
        const optional<bool>& debug() const { return _debug; }

        optional<int>& binNumber() { return _binNumber; }
        const optional<int>& binNumber() const { return _binNumber; }

        /**
         * Terrain tile expiration
         */
        optional<int>& minExpiryFrames() { return _minExpiryFrames; }
        const optional<int>& minExpiryFrames() const { return _minExpiryFrames; }

        optional<double>& minExpiryTime() { return _minExpiryTime; }
        const optional<double>& minExpiryTime() const { return _minExpiryTime; }

        /**
         * Whether the terrain should cast shadows - default is false
         */
        optional<bool>& castShadows() { return _castShadows; }
        const optional<bool>& castShadows() const { return _castShadows; }

        /** Mode to use when calculating LOD switching distances.
         *  Choices are DISTANCE_FROM_EYE_POINT (default) or PIXEL_SIZE_ON_SCREEN */
        optional<osg::LOD::RangeMode>& rangeMode() { return _rangeMode;}
        const optional<osg::LOD::RangeMode>& rangeMode() const { return _rangeMode;}

        /** The size of the tile, in pixels, when using rangeMode = PIXEL_SIZE_ON_SCREEN */
        optional<float>& tilePixelSize() { return _tilePixelSize; }
        const optional<float>& tilePixelSize() const { return _tilePixelSize; }
   
    public:
        virtual Config getConfig() const;

    protected:
        virtual void mergeConfig( const Config& conf ) {
            DriverConfigOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf );
                    
        optional<float> _verticalScale;
        optional<float> _verticalOffset;
        optional<int>   _tileSize;
        optional<float> _minTileRangeFactor;        
        optional<unsigned> _minLOD;
        optional<unsigned> _maxLOD;
        optional<unsigned> _firstLOD;
        optional<bool> _enableLighting;
        optional<float> _attenuationDistance;
        optional<bool> _clusterCulling;
        optional<bool> _enableBlending;
        optional<bool> _compressNormalMaps;
        optional<bool> _mercatorFastPath;
        optional<ElevationInterpolation> _elevationInterpolation;
        optional<unsigned> _minNormalMapLOD;
        optional<bool> _gpuTessellation;
        optional<bool> _debug;
        optional<int> _binNumber;
        optional<int> _minExpiryFrames;
        optional<double> _minExpiryTime;
        optional<bool> _castShadows;
        optional<osg::LOD::RangeMode> _rangeMode;
        optional<float> _tilePixelSize;
    };
}

#endif // OSGEARTH_TERRAIN_OPTIONS_H
