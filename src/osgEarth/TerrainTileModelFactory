/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#ifndef OSGEARTH_TERRAIN_TILE_MODEL_FACTORY_H
#define OSGEARTH_TERRAIN_TILE_MODEL_FACTORY_H 1

#include <osgEarth/TerrainTileModel>
#include <osgEarth/TerrainOptions>
#include <osgEarth/TerrainEngineRequirements>
#include <osgEarth/ImageLayer>
#include <osgEarth/Progress>

namespace osgEarth
{
    /**
     * Filter you can pass to TerrainTileModelFactory::createTileModel
     */
    class CreateTileModelFilter
    {
    public:
        CreateTileModelFilter() : _elevation(false) { }

        /** Whether to fetch elevation data and normal maps */
        optional<bool>& elevation() { return _elevation; }
        const optional<bool>& elevation() const { return _elevation; }

        /** List of other layers to accept */
        std::set<UID>& layers() { return _layers; }
        const std::set<UID>& layers() const { return _layers; }

        bool accept(const Layer* layer) const {
            return empty() || _layers.find(layer->getUID()) != _layers.end();
        }

        bool empty() const {
            return !_elevation.isSet() && _layers.empty();
        }

        void clear() {
            _elevation.unset();
            _layers.clear();
        }
    protected:
        optional<bool> _elevation;
        std::set<UID> _layers;
    };

    /**
     * Builds a TerrainTileModel from a map frame.
     */
    class OSGEARTH_EXPORT TerrainTileModelFactory : public osg::Referenced
    {
    public:
        /** Constructor */
        TerrainTileModelFactory(
            const TerrainOptions& options);

        /**
         * Creates a tile model and populates it with data from the map.
         *
         * @param frame        Map frame from which to read source data
         * @param key          Tile key for which to create the model
         * @param layers       Set of layer UIDs for which to fetch data; NULL => all layers
         * @param requirements Hints that tell the factory what types of data to add
         * @param progress     Progress tracking callback
         */
        virtual TerrainTileModel* createTileModel(
            const Map*                       map,
            const TileKey&                   key,
            const CreateTileModelFilter&     filter,
            const TerrainEngineRequirements* requirements,
            ProgressCallback*                progress);

    protected:

        virtual void addColorLayers(
            TerrainTileModel*                model,
            const Map*                       map,
            const TerrainEngineRequirements* reqs,
            const TileKey&                   key,
            const CreateTileModelFilter&     filter,
            ProgressCallback*                progress);

        virtual void addElevation(
            TerrainTileModel*            model,
            const Map*                   map,
            const TileKey&               key,
            const CreateTileModelFilter& filter,
            unsigned                     border,
            ProgressCallback*            progress);

        virtual void addPatchLayers(
            TerrainTileModel*            model,
            const Map*                   map,
            const TileKey&               key,
            const CreateTileModelFilter& filter,
            ProgressCallback*            progress);

    protected:

        /** Find a heightfield in the cache, or fetch it from the source. */
        bool getOrCreateHeightField(
            const Map*                      map,
            const TileKey&                  key,
            ElevationSamplePolicy           samplePolicy,
            ElevationInterpolation          interpolation,
            unsigned                        border,
            osg::ref_ptr<osg::HeightField>& out_hf,
            osg::ref_ptr<NormalMap>&        out_normalMap,
            ProgressCallback*               progress);

        osg::Texture* createImageTexture(
            osg::Image*       image,
            const ImageLayer* layer) const;

        osg::Texture* createCoverageTexture(
            osg::Image*       image,
            const ImageLayer* layer) const;

        osg::Texture* createElevationTexture(
            osg::Image* image) const;

        osg::Texture* createNormalTexture(
            osg::Image* image,
            bool compress
            ) const;

        const TerrainOptions _options;
        

        /** Key into the height field cache */
        struct HFCacheKey 
        {
            TileKey               _key;
            Revision              _revision;
            ElevationSamplePolicy _samplePolicy;

            bool operator < (const HFCacheKey& rhs) const {
                if ( _key < rhs._key ) return true;
                if ( rhs._key < _key ) return false;
                if ( _revision < rhs._revision ) return true;
                if ( _revision > rhs._revision ) return false;
                return _samplePolicy < rhs._samplePolicy;
            }
        };

        struct HFCacheValue
        {
            osg::ref_ptr<osg::HeightField> _hf;
            osg::ref_ptr<NormalMap> _normalMap;
        };
        //typedef osg::ref_ptr<osg::HeightField> HFCacheValue;
        typedef LRUCache<HFCacheKey, HFCacheValue> HFCache;
        HFCache _heightFieldCache;
        bool    _heightFieldCacheEnabled;
        osg::ref_ptr<osg::Texture> _emptyTexture;
    };
}

#endif // OSGEARTH_TERRAIN_TILE_MODEL_FACTORY_H
