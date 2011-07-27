#include <osgEarth/ElevationQuery>
#include <osgEarth/Locators>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[ElevationQuery] "

using namespace osgEarth;
using namespace OpenThreads;

ElevationQuery::ElevationQuery( const Map* map ) :
_mapf( map, Map::ELEVATION_LAYERS )
{
    postCTOR();
}

ElevationQuery::ElevationQuery( const MapFrame& mapFrame ) :
_mapf( mapFrame )
{
    postCTOR();
}

void
ElevationQuery::postCTOR()
{
    _tileSize         = 0;
    _maxDataLevel     = 0;
    _technique        = TECHNIQUE_PARAMETRIC;
    _interpolation    = INTERP_BILINEAR;
    _maxLevelOverride = -1;

    // Limit the size of the cache we'll use to cache heightfields. This is an
    // LRU cache.
    _tileCache.setMaxSize( 50 );
}

void
ElevationQuery::sync()
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

ElevationQuery::Technique
ElevationQuery::getTechnique() const
{
    return _technique;
}

void
ElevationQuery::setTechnique( ElevationQuery::Technique technique )
{
    _technique = technique;
}

void
ElevationQuery::setMaxTilesToCache( int value )
{
    _tileCache.setMaxSize( value );
}

int
ElevationQuery::getMaxTilesToCache() const
{
    return _tileCache.getMaxSize();
}

void
ElevationQuery::setInterpolation( ElevationInterpolation interp)
{
    _interpolation = interp;
}

ElevationInterpolation
ElevationQuery::getElevationInterpolation() const
{
    return _interpolation;
}

bool
ElevationQuery::getElevation(const osg::Vec3d&       point,
                             const SpatialReference* pointSRS,
                             double&                 out_elevation,
                             double                  desiredResolution,
                             double*                 out_actualResolution)
{
    sync();
    return getElevationImpl( point, pointSRS, out_elevation, desiredResolution, out_actualResolution );
}

bool
ElevationQuery::getElevations(std::vector<osg::Vec3d>& points,
                              const SpatialReference*  pointsSRS,
                              bool                     ignoreZ,
                              double                   desiredResolution )
{
    sync();
    for( osg::Vec3dArray::iterator i = points.begin(); i != points.end(); ++i )
    {
        double elevation;
        double z = (*i).z();
        if ( getElevationImpl( *i, pointsSRS, elevation, desiredResolution ) )
        {
            (*i).z() = ignoreZ ? elevation : elevation + z;
        }
    }
    return true;
}

bool
ElevationQuery::getElevationImpl(const osg::Vec3d&       point,
                                 const SpatialReference* pointSRS,
                                 double&                 out_elevation,
                                 double                  desiredResolution,
                                 double*                 out_actualResolution)
{
    if ( _maxDataLevel == 0 || _tileSize == 0 )
    {
        // this means there are no heightfields.
        out_elevation = 0.0;
        return true;
    }
   
    // this is the ideal LOD for the requested resolution:
    unsigned int idealLevel = desiredResolution > 0.0
        ? _mapf.getProfile()->getLevelOfDetailForHorizResolution( desiredResolution, _tileSize )
        : _maxDataLevel;        

    // based on the heightfields available, this is the best we can theorically do:
    unsigned int bestAvailLevel = osg::minimum( idealLevel, _maxDataLevel );
    if (_maxLevelOverride >= 0)
    {
        bestAvailLevel = osg::minimum(bestAvailLevel, (unsigned int)_maxLevelOverride);
    }
    
    // transform the input coords to map coords:
    osg::Vec3d mapPoint = point;
    if ( pointSRS && !pointSRS->isEquivalentTo( _mapf.getProfile()->getSRS() ) )
    {
        if ( !pointSRS->transform2D( point.x(), point.y(), _mapf.getProfile()->getSRS(), mapPoint.x(), mapPoint.y() ) )
        {
            OE_WARN << LC << "Fail: coord transform failed" << std::endl;
            return false;
        }
    }

    osg::ref_ptr<osg::HeightField> hf;
    osg::ref_ptr<osgTerrain::TerrainTile> tile;

    // get the tilekey corresponding to the tile we need:
    TileKey key = _mapf.getProfile()->createTileKey( mapPoint.x(), mapPoint.y(), bestAvailLevel );
    if ( !key.valid() )
    {
        OE_WARN << LC << "Fail: coords fall outside map" << std::endl;
        return false;
    }

    // Check the tile cache. Note that the TileSource already likely has a MemCache
    // attached to it. We employ a secondary cache here for a couple reasons. One, this
    // cache will store not only the heightfield, but also the tesselated tile in the event
    // that we're using GEOMETRIC mode. Second, since the call the getHeightField can 
    // fallback on a lower resolution, this cache will hold the final resolution heightfield
    // instead of trying to fetch the higher resolution one each tiem.

    TileCache::Record record = _tileCache.get( key );
    if ( record.valid() )
        tile = record.value().get();
         
    // if we found it, make sure it has a heightfield in it:
    if ( tile.valid() )
    {
        osgTerrain::HeightFieldLayer* layer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
        if ( layer )
            hf = layer->getHeightField();

        if ( !hf.valid() )
            tile = 0L;
    }

    // if we didn't find it (or it didn't have heightfield data), build it.
    if ( !tile.valid() )
    {
        // generate the heightfield corresponding to the tile key, automatically falling back
        // on lower resolution if necessary:
        _mapf.getHeightField( key, true, hf, 0L, _interpolation );

        // bail out if we could not make a heightfield a all.
        if ( !hf.valid() )
        {
            OE_WARN << LC << "Unable to create heightfield for key " << key.str() << std::endl;
            return false;
        }

        // All this stuff is requires for GEOMETRIC mode. An optimization would be to
        // defer this so that PARAMETRIC mode doesn't waste time
        GeoLocator* locator = GeoLocator::createForKey( key, _mapf.getMapInfo() );

        tile = new osgTerrain::TerrainTile();

        osgTerrain::HeightFieldLayer* layer = new osgTerrain::HeightFieldLayer( hf.get() );
        layer->setLocator( locator );

        tile->setElevationLayer( layer );
        tile->setRequiresNormals( false );
        tile->setTerrainTechnique( new osgTerrain::GeometryTechnique );

        // store it in the local tile cache.
        _tileCache.insert( key, tile.get() );
    }

    OE_DEBUG << LC << "LRU Cache, hit ratio = " << _tileCache.getHitRatio() << std::endl;

    // see what the actual resolution of the heightfield is.
    if ( out_actualResolution )
        *out_actualResolution = (double)hf->getXInterval();

    // finally it's time to get a height value:
    if ( _technique == TECHNIQUE_PARAMETRIC )
    {
        const GeoExtent& extent = key.getExtent();
        double xInterval = extent.width()  / (double)(hf->getNumColumns()-1);
        double yInterval = extent.height() / (double)(hf->getNumRows()-1);
        out_elevation = (double) HeightFieldUtils::getHeightAtLocation( 
            hf.get(), mapPoint.x(), mapPoint.y(), extent.xMin(), extent.yMin(), xInterval, yInterval );
        return true;
    }
    else // ( _technique == TECHNIQUE_GEOMETRIC )
    {
        osg::Vec3d start, end, zero;

        if ( _mapf.getMapInfo().isGeocentric() )
        {
            const SpatialReference* mapSRS = _mapf.getProfile()->getSRS();

            mapSRS->transformToECEF( osg::Vec3d(mapPoint.y(), mapPoint.x(),  50000.0), start );
            mapSRS->transformToECEF( osg::Vec3d(mapPoint.y(), mapPoint.x(), -50000.0), end );
            mapSRS->transformToECEF( osg::Vec3d(mapPoint.y(), mapPoint.x(),      0.0), zero );
        }
        else // PROJECTED
        {
            start.set( mapPoint.x(), mapPoint.y(),  50000.0 );
            end.set  ( mapPoint.x(), mapPoint.y(), -50000.0 );
            zero.set ( mapPoint.x(), mapPoint.y(),      0.0 );
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

        OE_DEBUG << LC << "No intersection" << std::endl;
        return false;
    }
}
