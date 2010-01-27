#include <osgEarthUtil/ElevationManager>
#include <osgEarth/EarthTerrainTechnique>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

using namespace osgEarthUtil;
using namespace osgEarth;
using namespace OpenThreads;

ElevationManager::ElevationManager( Map* map ) :
_map( map )
{
    postCTOR();
}

//ElevationManager::ElevationManager( MapNode* mapNode ) :
//_mapNode( mapNode ),
//_map( mapNode? mapNode->getMap() : NULL ),
//_mapEngine( mapNode? mapNode->getEngine() : NULL )
//{
//    postCTOR();
//}

void
ElevationManager::postCTOR()
{
    _lastMapRevision = -1;
    _tileSize = 0;
    _maxDataLevel = 0;
    _maxCacheSize = 100;
    _technique = TECHNIQUE_GEOMETRIC;

    if ( !_mapEngine.valid() )
    {
        _mapEngine = new MapEngine();
    }

    checkForMapUpdates();
}

void
ElevationManager::checkForMapUpdates()
{
    int mapRev = _map->getDataModelRevision();
    if ( _lastMapRevision != mapRev )
    {
        ScopedReadLock lock( _map->getMapDataMutex() );

        _tileSize = 0;
        _maxDataLevel = 0;

        const MapLayerList& hflayers = _map->getHeightFieldMapLayers();
        for( MapLayerList::const_iterator i = hflayers.begin(); i != hflayers.end(); i++ )
        {
            // we need the maximum tile size
            int layerTileSize = i->get()->getTileSource()->getPixelsPerTile();
            if ( layerTileSize > _tileSize )
                _tileSize = layerTileSize;

            // we also need the maximum available data level.
            unsigned int layerMaxDataLevel = i->get()->getTileSource()->getMaxDataLevel();
            if ( layerMaxDataLevel > _maxDataLevel )
                _maxDataLevel = layerMaxDataLevel;
        }

        _lastMapRevision = mapRev;
    }
}

Map*
ElevationManager::getMap() const
{
    return _map.get();
}

ElevationManager::Technique
ElevationManager::getTechnique() const
{
    return _technique;
}

void
ElevationManager::setTechnique( ElevationManager::Technique technique )
{
    _technique = technique;
}

void
ElevationManager::setMaxTilesToCache( int value )
{
    _maxCacheSize = value;
}

int
ElevationManager::getMaxTilesToCache() const
{
    return _maxCacheSize;
}

bool
ElevationManager::getElevation(double x, double y,
                               double resolution,
                               const SpatialReference* srs,
                               double& out_elevation,
                               double& out_resolution )
{
    checkForMapUpdates();

    if ( _maxDataLevel == 0 || _tileSize == 0 )
    {
        // this means there are no heightfields.
        out_elevation = 0.0;
        return true;
    }
   
    // this is the ideal LOD for the requested resolution:
    unsigned int idealLevel = resolution > 0.0
        ? _map->getProfile()->getLevelOfDetailForHorizResolution( resolution, _tileSize )
        : _maxDataLevel;        

    // based on the heightfields available, this is the best we can theorically do:
    unsigned int bestAvailLevel = osg::minimum( idealLevel, _maxDataLevel );
    
    // transform the input coords to map coords:
    double map_x = x, map_y = y;
    if ( srs && !srs->isEquivalentTo( _map->getProfile()->getSRS() ) )
    {
        if ( !srs->transform( x, y, _map->getProfile()->getSRS(), map_x, map_y ) )
        {
            osg::notify(osg::WARN) << "[osgEarth] ElevationManager: coord transform failed" << std::endl;
            return false;
        }
    }

    osg::ref_ptr<TileKey> key;
    osg::ref_ptr<osg::HeightField> hf;
    osg::ref_ptr<osgTerrain::TerrainTile> tile;

    // get the tilekey corresponding to the tile we need:
    key = _map->getProfile()->createTileKey( map_x, map_y, bestAvailLevel );
    if ( !key.valid() )
    {
        osg::notify(osg::WARN) << "[osgEarth] ElevationManager: coords fall outside map" << std::endl;
        return false;
    }

    // now, see if we already have this tile loaded somewhere:
    osgTerrain::TileID tileId = key->getTileId();

    //if ( _mapNode.valid() )
    //{
    //    // first look in the map node's default terrain:
    //    tile = _mapNode->getTerrain(0)->getTile( tileId );

    //    // if it's in the map, remove it from the local tile cache
    //    if ( tile.valid() )
    //        _tileCache.erase( tileId );
    //}

    if ( !tile.valid() )
    {
        // next check the local tile cache:
        TileTable::const_iterator i = _tileCache.find( tileId );
        if ( i != _tileCache.end() )
            tile = i->second.get();
    }

         
    // if we found it, make sure it has a heightfield in it:
    if ( tile.valid() )
    {
        osgTerrain::HeightFieldLayer* layer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
        if ( layer )
        {
            hf = layer->getHeightField();
        }
        if ( !hf.valid() )
        {
            tile = NULL;
        }
    }

    // if we didn't find it (or it didn't have heightfield data), build it.
    if ( !tile.valid() )
    {
        //osg::notify(osg::NOTICE) << "[osgEarth] ElevationManager: cache miss" << std::endl;

        // generate the heightfield corresponding to the tile key, automatically falling back
        // on lower resolution if necessary:
        hf = _map->createHeightField( key.get(), true );

        // bail out if we could not make a heightfield a all.
        if ( !hf.valid() )
        {
            osg::notify(osg::WARN) << "[osgEarth] ElevationManager: unable to create heightfield" << std::endl;
            return false;
        }

        osgTerrain::Locator* locator = GeoLocator::createForKey( key.get(), _map.get() );

        tile = new osgTerrain::TerrainTile();
        tile->setTileID( tileId );
        tile->setLocator( locator );

        osgTerrain::HeightFieldLayer* layer = new osgTerrain::HeightFieldLayer( hf.get() );
        layer->setLocator( locator );

        tile->setElevationLayer( layer );
        tile->setRequiresNormals( false );
        tile->setTerrainTechnique( new EarthTerrainTechnique() );

        // store it in the local tile cache.
        // TODO: limit the size of the cache with a parallel FIFO list.
        _tileCache[tileId] = tile.get();
        _tileCacheFIFO.push_back( tileId );

        // prune the cache. this is a terrible pruning method.
        if ( _tileCache.size() > _maxCacheSize )
        {
            osgTerrain::TileID id = _tileCacheFIFO.front();
            _tileCacheFIFO.pop_front();
            if ( tileId != id )
                _tileCache.erase( id );
        }
    }


    // see what the actual resolution of the heightfield is.
    out_resolution = (double)hf->getXInterval();


    // finally it's time to get a height value:
    if ( _technique = TECHNIQUE_PARAMETRIC )
    {
        const GeoExtent& extent = key->getGeoExtent();
        double xInterval = extent.width()  / (double)(hf->getNumColumns()-1);
        double yInterval = extent.height() / (double)(hf->getNumRows()-1);
        out_elevation = (double) HeightFieldUtils::getHeightAtLocation( hf.get(), map_x, map_y, extent.xMin(), extent.yMin(), xInterval, yInterval );
        return true;
    }
    else // ( _technique == TECHNIQUE_GEOMETRIC )
    {
        osg::Vec3d start, end, zero;

        if ( _map->getCoordinateSystemType() == Map::CSTYPE_GEOCENTRIC )
        {
            _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( map_y ),
                osg::DegreesToRadians( map_x ),
                50000,
                start.x(), start.y(), start.z() );

            _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( map_y ),
                osg::DegreesToRadians( map_x ),
                -50000,
                end.x(), end.y(), end.z() );

            _map->getProfile()->getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( map_y ),
                osg::DegreesToRadians( map_x ),
                0.0,
                zero.x(), zero.y(), zero.z() );
        }
        else // PROJECTED
        {
            start.x() = map_x; start.y() = map_y; start.z() = 50000;
            end.x() = map_x; end.y() = map_y; end.z() = -50000;
            zero.x() = map_x; zero.y() = map_y; zero.z() = 0;
        }

        osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );
        osgUtil::IntersectionVisitor iv;
        iv.setIntersector( i );

        tile->accept( iv );

        osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
        if ( !results.empty() )
        {
            const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
            osg::Vec3d isectPoint = result.getWorldIntersectPoint();
            out_elevation = (isectPoint-end).length2() > (zero-end).length2()
                ? (isectPoint-zero).length()
                : -(isectPoint-zero).length();
            return true;            
        }

        osg::notify(osg::WARN) << "[osgEarth] ElevationManager: no intersections" << std::endl;
        return false;
    }
}


bool 
ElevationManager::getPlacementMatrix(double x, double y, double z,
                                     double resolution,
                                     const SpatialReference* srs,
                                     osg::Matrixd& out_matrix,
                                     double& out_elevation,
                                     double& out_resolution)
{
    // transform the input coords to map coords:
    double map_x = x, map_y = y;
    if ( srs && !srs->isEquivalentTo( _map->getProfile()->getSRS() ) )
    {
        if ( !srs->transform( x, y, _map->getProfile()->getSRS(), map_x, map_y ) )
        {
            osg::notify(osg::WARN) << "[osgEarth] ElevationManager: coord transform failed" << std::endl;
            return false;
        }
    }

    // get the elevation under those coordinates:
    if ( !getElevation( map_x, map_y, resolution, _map->getProfile()->getSRS(), out_elevation, out_resolution ) )
    {
        osg::notify(osg::WARN) << "[osgEarth] ElevationManager::getElevation() failed" << std::endl;
        return false;
    }

    if ( _map->getCoordinateSystemType() == Map::CSTYPE_PROJECTED )
    {
        out_matrix = osg::Matrixd::translate( x, y, out_elevation + z );
    }
    else
    {
        _map->getProfile()->getSRS()->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(
            osg::DegreesToRadians( map_y ),
            osg::DegreesToRadians( map_x ),
            out_elevation + z,
            out_matrix );
    }

    return true;
}