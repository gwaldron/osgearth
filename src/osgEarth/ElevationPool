/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#ifndef OSGEARTH_ELEVATION_POOL_H
#define OSGEARTH_ELEVATION_POOL_H

#include <osgEarth/Common>
#include <osgEarth/ElevationLayer>
#include <osgEarth/GeoData>
#include <osgEarth/TileKey>
#include <osgEarth/ThreadingUtils>
#include <osg/Timer>
#include <map>

namespace osgEarth
{
    using namespace Threading;

    // defined at the end of this file
    class ElevationEnvelope;
    class Map;

    //! Result of an elevation query.
    struct ElevationSample : public osg::Referenced
    {
        float elevation;
        float resolution;
        ElevationSample(float a, float b) : elevation(a), resolution(b) { }
    };

    /**
     * A pool of elevation data that can be used to manage regional elevation
     * data queries. To use this, call createEnvelope() and use that object
     * to make your elevation queries at a particular LOD. The queries can
     * be anywhere geographically, but the algorithm will optimize lots of
     * queries in a particular area (like a terrain tile).
     *
     * Each Map contains an ElevationPool you can access for queries against
     * that Map.
     *
     * ElevationEnvelope* envelope = pool->createEnvelope(srs, lod);
     * float z = envelope->getElevation(point);
     */
    class OSGEARTH_EXPORT ElevationPool : public osg::Referenced
    {
    public:
        /** ctor */
        ElevationPool();

        /** Sets the map from which to read elevation data. */
        void setMap(const Map* map);

        /**
         * Sets a vector of ElevationLayers from which to read elevation data.
         * Omit this is you just want to use all the elevation layes in the Map
         * that you set with setMap().
         */
        void setElevationLayers(const ElevationLayerVector& layers);
        const ElevationLayerVector& getElevationLayers() const { return _layers; }

        /** Sets the dimension of heightfield tiles to read from the map */
        void setTileSize(unsigned size);
        unsigned getTileSize() const { return _tileSize; }

        /** Creates a query envelope to use for elevation queries in a certain area. */
        ElevationEnvelope* createEnvelope(const SpatialReference* srs, unsigned lod);

        //! Queries the elevation at a GeoPoint for a given LOD.
        Future<ElevationSample> getElevation(const GeoPoint& p, unsigned lod=23);

        /** Maximum number of elevation tiles to cache */
        void setMaxEntries(unsigned maxEntries) { _maxEntries = maxEntries; }
        unsigned getMaxEntries() const          { return _maxEntries; }

        /** Clears any cached tiles from the elevation pool. */
        void clear();
        
        void stopThreading();

    protected:

        osg::observer_ptr<const Map> _map;

        ElevationLayerVector _layers;

        enum Status
        {
            STATUS_EMPTY = 0u,
            STATUS_IN_PROGRESS = 1u,
            STATUS_AVAILABLE = 2u,
            STATUS_FAIL = 3u
        };

        // Single elevation tile along with its load status
        class Tile : public osg::Referenced
        {
        public:
            Tile() : _status(STATUS_EMPTY) { }
            TileKey             _key;           // key used to request this tile
            Bounds              _bounds;
            GeoHeightField      _hf;
            OpenThreads::Atomic _status;
            osg::Timer_t        _loadTime;
        };

        // Custom comparator for Tile that sorts Tiles in a set from
        // highest resolution to lowest resolution; This ensures we are
        // sampling overlapping tiles in the proper order
        struct TileSortHiResToLoRes
        {
            bool operator()(
                const osg::ref_ptr<Tile>& lhs,
                const osg::ref_ptr<Tile>& rhs) const
            {
                // works because the KEY less-than function checks the LOD number
                // first, which corresponds to the resolution. Higher LOD = higher res.
                return rhs->_key < lhs->_key;
            }
        };
                
        // Keeps track of the most-recently-used tiles, in order from MRU (front)
        // to LRU (back). Multiple pointers to the same Tile may exist in the MRU list.
        // Once all pointers to a Tile disappear from the MRU (by popping off the back)
        // the corresponding Tile observer in the Tiles data structure will go NULL
        // and the Tile will destruct.
        typedef std::list<osg::ref_ptr<Tile> > MRU;
        MRU _mru;

        // Cached set of tiles, sorted by TileKey. These are observer pointers; the 
        // actual references are held in the MRU. That way when all pointers drop off
        // the back of the MRU, the Tile is destroyed and the main observer goes to 
        // NULL and is removed.
        typedef std::map<TileKey, osg::observer_ptr<Tile> > Tiles;
        Tiles _tiles;
        Threading::Mutex  _tilesMutex;

        // Track the number of entries in the MRU manually since std::list::size
        // can be O(n) on some platforms
        unsigned _entries;
        unsigned _maxEntries;

        // dimension of sampling heightfield
        unsigned _tileSize;

        // QuerySet is a collection of Tiles, sorted from high to low resolution,
        // that a ElevationEnvelope uses for a terrain sampling opteration.
        typedef std::set<osg::ref_ptr<Tile>, TileSortHiResToLoRes> QuerySet;

        // Asynchronous elevation query operation
        struct GetElevationOp : public osg::Operation {
            GetElevationOp(ElevationPool*, const GeoPoint&, unsigned lod);
            osg::observer_ptr<ElevationPool> _pool;
            GeoPoint _point;
            unsigned _lod;
            Promise<ElevationSample> _promise;
            void operator()(osg::Object*);
        };
        friend struct GetElevationOp;
        osg::ref_ptr<osg::OperationQueue> _opQueue;
        std::vector< osg::ref_ptr<osg::OperationThread> > _opThreads;

        virtual ~ElevationPool();

    protected:

        // safely popluate the tile; called when Tile._status = IN_PROGRESS
        bool fetchTileFromMap(const TileKey& key, const ElevationLayerVector& layers, Tile* tile);
        
        // safely fetch a tile from the central repo, loading from map if necessary
        bool getTile(const TileKey& key, const ElevationLayerVector& layers, osg::ref_ptr<Tile>& output);
        
        // safely fetch a tile from the central repo, loading from map if necessary
        bool tryTile(const TileKey& key, const ElevationLayerVector& layers, osg::ref_ptr<Tile>& output);

        // safely remove the oldest item on the MRU
        void popMRU();

        // clears and resets the pool.
        void clearImpl();

        friend class ElevationEnvelope;
    };


    /**
     * Set of terrain tiles corresponding to a geographic extent that the pager
     * uses to clamp a feature set. You cannot create this object directly;
     * instead call ElevationPool::createEnvelope().
     */
    class OSGEARTH_EXPORT ElevationEnvelope : public osg::Referenced
    {
    public:

        /**
         * Gets a single elevation, or returns NO_DATA_VALUE upon failure.
         * The Z value is ignored.
         */
        float getElevation(double x, double y);

        /**
         * Gets a single elevation along with the resolution of the data from
         * which that sample was taken.
         */
        std::pair<float, float> getElevationAndResolution(double x, double y);

        /**
         * Gets a elevation value for each input point and puts them in output.
         * Returns the number of successful elevations. Failed queries are set to
         * NO_DATA_VALUE in the output vector.
         */
        unsigned getElevations(
            const std::vector<osg::Vec3d>& input,
            std::vector<float>& output);

        /**
         * Gets the elevation extrema over a collection of point data.
         * Returns false if the points don't fall inside the envelope
         */
        bool getElevationExtrema(
            const std::vector<osg::Vec3d>& points, 
            float& out_min, float& out_max);

        /**
         * The SRS that this envelope expects query points to be in
         */
        const SpatialReference* getSRS() const;

        /**
         * LOD at which this envelope will try to sample the elevation
         */
        unsigned getLOD() const { return _lod; }

    protected:
        ElevationEnvelope();
        virtual ~ElevationEnvelope();

        ElevationLayerVector _layers;
        ElevationPool::QuerySet _tiles;
        osg::ref_ptr<const SpatialReference> _inputSRS;
        unsigned _lod;
        osg::observer_ptr<ElevationPool> _pool;
        osg::ref_ptr<const Profile> _mapProfile;
        friend class ElevationPool;

    private:
        bool sample(double x, double y, float& out_elevation, float& out_resolution);
    };

} // namespace

#endif // OSGEARTH_ELEVATION_POOL_H
