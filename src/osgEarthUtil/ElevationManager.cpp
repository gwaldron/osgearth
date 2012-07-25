#include <osgEarthUtil/ElevationManager>
#include <osgEarth/Locators>
#include <osgEarth/HeightFieldUtils>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[ElevationManager] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace OpenThreads;

ElevationManager::ElevationManager( Map* map ) :
_mapf( map, Map::ELEVATION_LAYERS )
{
    postCTOR();
}

void
ElevationManager::postCTOR()
{
    _tileSize = 0;
    _maxDataLevel = 0;
    _maxCacheSize = 100;
    _technique = TECHNIQUE_PARAMETRIC;
    _interpolation = INTERP_BILINEAR;
    _maxLevelOverride = -1;
}

void
ElevationManager::sync()
{
    if ( _mapf.sync() || _tileSize == 0 || _maxDataLevel == 0 )
    {
        _tileSize = 0;
        _maxDataLevel = 0;

        for( ElevationLayerVector::const_iterator i = _mapf.elevationLayers().begin(); i != _mapf.elevationLayers().end(); ++i )
        {
            // we need the maximum tile size
            int layerTileSize = i->get()->getTileSize();
            if ( layerTileSize > _tileSize )
                _tileSize = layerTileSize;

            // we also need the maximum available data level.
            unsigned int layerMaxDataLevel = i->get()->getMaxDataLevel();
            if ( layerMaxDataLevel > _maxDataLevel )
                _maxDataLevel = layerMaxDataLevel;
        }
    }
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

void
ElevationManager::setMaxLevelOverride(int maxLevelOverride)
{
    _maxLevelOverride = maxLevelOverride;
}

int
ElevationManager::getMaxLevelOverride() const
{
    return _maxLevelOverride;
}

void
ElevationManager::setInterpolation( ElevationInterpolation interp)
{
    _interpolation = interp;
}

ElevationInterpolation
ElevationManager::getElevationInterpolation() const
{
    return _interpolation;
}

bool
ElevationManager::getElevation(double x, double y,
                               double resolution,
                               const SpatialReference* srs,
                               double& out_elevation,
                               double& out_resolution)
{
    sync();
    return getElevationImpl(x, y, resolution, srs, out_elevation, out_resolution);
}

bool
ElevationManager::getElevationImpl(double x, double y,
                                   double resolution,
                                   const SpatialReference* srs,
                                   double& out_elevation,
                                   double& out_resolution)
{
    if ( _maxDataLevel == 0 || _tileSize == 0 )
    {
        // this means there are no heightfields.
        out_elevation = 0.0;
        return true;
    }
   
    // this is the ideal LOD for the requested resolution:
    unsigned int idealLevel = resolution > 0.0
        ? _mapf.getProfile()->getLevelOfDetailForHorizResolution( resolution, _tileSize )
        : _maxDataLevel;        

    // based on the heightfields available, this is the best we can theorically do:
    unsigned int bestAvailLevel = osg::minimum( idealLevel, _maxDataLevel );
    if (_maxLevelOverride >= 0)
    {
        bestAvailLevel = osg::minimum(bestAvailLevel, (unsigned int)_maxLevelOverride);
    }
    
    // transform the input coords to map coords:
    double map_x = x, map_y = y;
    if ( srs && !srs->isEquivalentTo( _mapf.getProfile()->getSRS() ) )
    {
        if ( !srs->transform2D( x, y, _mapf.getProfile()->getSRS(), map_x, map_y ) )
        {
            OE_WARN << LC << "Fail: coord transform failed" << std::endl;
            return false;
        }
    }

    osg::ref_ptr<osg::HeightField> hf;
    osg::ref_ptr<osgTerrain::TerrainTile> tile;

    // get the tilekey corresponding to the tile we need:
    TileKey key = _mapf.getProfile()->createTileKey( map_x, map_y, bestAvailLevel );
    if ( !key.valid() )
    {
        OE_WARN << LC << "Fail: coords fall outside map" << std::endl;
        return false;
    }

    // now, see if we already have this tile loaded somewhere:
    osgTerrain::TileID tileId = key.getTileId();

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
        //OE_NOTICE << "ElevationManager: cache miss" << std::endl;

        // generate the heightfield corresponding to the tile key, automatically falling back
        // on lower resolution if necessary:
        _mapf.getHeightField( key, true, hf, 0L );

        // bail out if we could not make a heightfield a all.
        if ( !hf.valid() )
        {
            OE_WARN << "ElevationManager: unable to create heightfield" << std::endl;
            return false;
        }

        GeoLocator* locator = GeoLocator::createForKey( key, _mapf.getMapInfo() );

        tile = new osgTerrain::TerrainTile();

        osgTerrain::HeightFieldLayer* layer = new osgTerrain::HeightFieldLayer( hf.get() );
        layer->setLocator( locator );

        tile->setElevationLayer( layer );
        tile->setRequiresNormals( false );
        tile->setTerrainTechnique( new osgTerrain::GeometryTechnique );

        // store it in the local tile cache.
        // TODO: limit the size of the cache with a parallel FIFO list.
        _tileCache[tileId] = tile.get();
        _tileCacheFIFO.push_back( tileId );

        // prune the cache. this is a terrible pruning method.
        if ( (int)_tileCache.size() > _maxCacheSize )
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
    if ( _technique == TECHNIQUE_PARAMETRIC )
    {
        const GeoExtent& extent = key.getExtent();
        double xInterval = extent.width()  / (double)(hf->getNumColumns()-1);
        double yInterval = extent.height() / (double)(hf->getNumRows()-1);
        out_elevation = (double) HeightFieldUtils::getHeightAtLocation( hf.get(), map_x, map_y, extent.xMin(), extent.yMin(), xInterval, yInterval );
        return true;
    }
    else // ( _technique == TECHNIQUE_GEOMETRIC )
    {
        osg::Vec3d start, end, zero;

        if ( _mapf.getMapInfo().isGeocentric() )
        {
            const osg::EllipsoidModel* ellip = _mapf.getProfile()->getSRS()->getEllipsoid();

            ellip->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( map_y ),
                osg::DegreesToRadians( map_x ),
                50000,
                start.x(), start.y(), start.z() );

            ellip->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( map_y ),
                osg::DegreesToRadians( map_x ),
                -50000,
                end.x(), end.y(), end.z() );

            ellip->convertLatLongHeightToXYZ(
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

        OE_WARN << "ElevationManager: no intersections" << std::endl;
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
    sync();

    const SpatialReference* mapSRS = _mapf.getProfile()->getSRS();

    // transform the input coords to map coords:
    double map_x = x, map_y = y;
    if ( srs && !srs->isEquivalentTo( mapSRS ) )
    {
        if ( !srs->transform2D( x, y, mapSRS, map_x, map_y ) )
        {
            OE_WARN << LC << "getPlacementMatrix: coord transform failed" << std::endl;
            return false;
        }
    }

    // get the elevation under those coordinates:
    if ( !getElevationImpl( map_x, map_y, resolution, mapSRS, out_elevation, out_resolution) )
    {
        OE_WARN << LC << "getPlacementMatrix: getElevation failed" << std::endl;
        return false;
    }

    if ( _mapf.getMapInfo().isGeocentric() )
    {
        mapSRS->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(
            osg::DegreesToRadians( map_y ),
            osg::DegreesToRadians( map_x ),
            out_elevation + z,
            out_matrix );
    }
    else
    {
        out_matrix = osg::Matrixd::translate( x, y, out_elevation + z );
    }

    return true;
}
