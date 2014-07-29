#include <osgEarth/ElevationQuery>
#include <osgEarth/Locators>
#include <osgEarth/HeightFieldUtils>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[ElevationQuery] "

using namespace osgEarth;
using namespace OpenThreads;

namespace
{
    int nextPowerOf2(int x) {
        --x;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x+1;
    }
}

ElevationQuery::ElevationQuery( const Map* map ) :
_mapf( map, Map::TERRAIN_LAYERS )
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
    _maxLevelOverride = -1;
    _queries          = 0.0;
    _totalTime        = 0.0;  
    _cache.setMaxSize( 500 );
}

void
ElevationQuery::sync()
{
    if ( _mapf.needsSync() )
    {
        _mapf.sync();
        _cache.clear();
    }
}

unsigned
ElevationQuery::getMaxLevel( double x, double y, const SpatialReference* srs, const Profile* profile ) const
{
    int targetTileSizePOT = nextPowerOf2((int)_mapf.getMapOptions().elevationTileSize().get());

    int maxLevel = 0;
    for( ElevationLayerVector::const_iterator i = _mapf.elevationLayers().begin(); i != _mapf.elevationLayers().end(); ++i )
    {
        const ElevationLayer* layer = i->get();

        // skip disabled layers
        if ( !layer->getEnabled() || !layer->getVisible() )
            continue;

        int layerMaxLevel = 0;

        osgEarth::TileSource* ts = layer->getTileSource();
        if ( ts )
        {
            // TileSource is good; check for optional data extents:
            if ( ts->getDataExtents().size() > 0 )
            {
                osg::Vec3d tsCoord(x, y, 0);

                const SpatialReference* tsSRS = ts->getProfile() ? ts->getProfile()->getSRS() : 0L;
                if ( srs && tsSRS )
                    srs->transform(tsCoord, tsSRS, tsCoord);
                else
                    tsSRS = srs;
                
                for (osgEarth::DataExtentList::iterator j = ts->getDataExtents().begin(); j != ts->getDataExtents().end(); j++)
                {
                    if (j->maxLevel().isSet() && j->maxLevel() > layerMaxLevel && j->contains( tsCoord.x(), tsCoord.y(), tsSRS ))
                    {
                        layerMaxLevel = j->maxLevel().value();
                    }
                }            
            }
            else
            {
                // Just use the default max level.  Without any data extents we don't know the actual max
                layerMaxLevel = (int)(*layer->getTerrainLayerRuntimeOptions().maxLevel());
            }

            // cap the max to the layer's express max level (if set).
            if ( layer->getTerrainLayerRuntimeOptions().maxLevel().isSet() )
            {
                layerMaxLevel = std::min( layerMaxLevel, (int)(*layer->getTerrainLayerRuntimeOptions().maxLevel()) );
            }

            // Need to convert the layer max of this TileSource to that of the actual profile
            layerMaxLevel = profile->getEquivalentLOD( ts->getProfile(), layerMaxLevel );
        }
        else
        {
            // no TileSource? probably in cache-only mode. Use the layer max (or its default).
            layerMaxLevel = (int)(layer->getTerrainLayerRuntimeOptions().maxLevel().value());
        }

        // Adjust for the tile size resolution differential, if supported by the layer.
        int layerTileSize = layer->getTileSize();
        if (layerTileSize > targetTileSizePOT)
        {
            int oldMaxLevel = layerMaxLevel;
            int temp = std::max(targetTileSizePOT, 2);
            while(temp < layerTileSize) {
                temp *= 2;
                ++layerMaxLevel;
            }
        }

        if (layerMaxLevel > maxLevel)
        {
            maxLevel = layerMaxLevel;
        }
    }

    return maxLevel;
}

void
ElevationQuery::setMaxTilesToCache( int value )
{
    _cache.setMaxSize( value );   
}

int
ElevationQuery::getMaxTilesToCache() const
{
    return _cache.getMaxSize();    
}
        
void
ElevationQuery::setMaxLevelOverride(int maxLevelOverride)
{
    _maxLevelOverride = maxLevelOverride;
}

int
ElevationQuery::getMaxLevelOverride() const
{
    return _maxLevelOverride;
}

bool
ElevationQuery::getElevation(const GeoPoint&         point,
                             double&                 out_elevation,
                             double                  desiredResolution,
                             double*                 out_actualResolution)
{
    sync();
    return getElevationImpl( point, out_elevation, desiredResolution, out_actualResolution );
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
        GeoPoint p(pointsSRS, *i, ALTMODE_ABSOLUTE);
        if ( getElevationImpl( p, elevation, desiredResolution ) )
        {
            (*i).z() = ignoreZ ? elevation : elevation + z;
        }
    }
    return true;
}

bool
ElevationQuery::getElevations(const std::vector<osg::Vec3d>& points,
                              const SpatialReference*        pointsSRS,
                              std::vector<double>&           out_elevations,
                              double                         desiredResolution )
{
    sync();
    for( osg::Vec3dArray::const_iterator i = points.begin(); i != points.end(); ++i )
    {
        double elevation;
        GeoPoint p(pointsSRS, *i, ALTMODE_ABSOLUTE);

        if ( getElevationImpl(p, elevation, desiredResolution) )
        {
            out_elevations.push_back( elevation );
        }
        else
        {
            out_elevations.push_back( 0.0 );
        }
    }
    return true;
}

bool
ElevationQuery::getElevationImpl(const GeoPoint& point,
                                 double&         out_elevation,
                                 double          desiredResolution,
                                 double*         out_actualResolution)
{
    osg::Timer_t start = osg::Timer::instance()->tick();

    if ( _mapf.elevationLayers().empty() )
    {
        // this means there are no heightfields.
        out_elevation = 0.0;
        return true;        
    }

    // tile size (resolution of elevation tiles)
    unsigned tileSize = std::max(_mapf.getMapOptions().elevationTileSize().get(), 2u);

    //This is the max resolution that we actually have data at this point
    unsigned int bestAvailLevel = getMaxLevel( point.x(), point.y(), point.getSRS(), _mapf.getProfile());

    if (desiredResolution > 0.0)
    {
        unsigned int desiredLevel = _mapf.getProfile()->getLevelOfDetailForHorizResolution( desiredResolution, tileSize );
        if (desiredLevel < bestAvailLevel) bestAvailLevel = desiredLevel;
    }

    OE_DEBUG << LC << "Best available data level " << point.x() << ", " << point.y() << " = "  << bestAvailLevel << std::endl;

    // transform the input coords to map coords:
    GeoPoint mapPoint = point;
    if ( point.isValid() && !point.getSRS()->isHorizEquivalentTo( _mapf.getProfile()->getSRS() ) )
    {
        mapPoint = point.transform(_mapf.getProfile()->getSRS());
        if ( !mapPoint.isValid() )
        {
            OE_WARN << LC << "Fail: coord transform failed" << std::endl;
            return false;
        }
    }    

    // get the tilekey corresponding to the tile we need:
    TileKey key = _mapf.getProfile()->createTileKey( mapPoint.x(), mapPoint.y(), bestAvailLevel );
    if ( !key.valid() )
    {
        OE_WARN << LC << "Fail: coords fall outside map" << std::endl;
        return false;
    }
        
    bool result = false;      
    while (!result)
    {      
        GeoHeightField geoHF;
        TileCache::Record record;
        // Try to get the hf from the cache
        if ( _cache.get( key, record ) )
        {                        
            geoHF = record.value();
        }
        else
        {
            // Create it            
            osg::ref_ptr<osg::HeightField> hf = new osg::HeightField();
            hf->allocate( tileSize, tileSize );

            // Initialize the heightfield to nodata
            for (unsigned int i = 0; i < hf->getFloatArray()->size(); i++)
            {
                hf->getFloatArray()->at( i ) = NO_DATA_VALUE;
            }   

            if (_mapf.populateHeightField(hf, key, false))
            {                
                geoHF = GeoHeightField( hf.get(), key.getExtent() );
                _cache.insert( key, geoHF );
            }
        }

        if (geoHF.valid())
        {            
            float elevation = 0.0f;                 
            result = geoHF.getElevation( mapPoint.getSRS(), mapPoint.x(), mapPoint.y(), _mapf.getMapInfo().getElevationInterpolation(), mapPoint.getSRS(), elevation);                              
            if (result && elevation != NO_DATA_VALUE)
            {                        
                // see what the actual resolution of the heightfield is.
                if ( out_actualResolution )
                    *out_actualResolution = geoHF.getXInterval(); 
                out_elevation = (double)elevation;                
                break;
            }
            else
            {                               
                result = false;
            }
        }

        if (!result)
        {
            key = key.createParentKey();                        
            if (!key.valid())
            {
                break;
            }
        }         
    }

         

    osg::Timer_t end = osg::Timer::instance()->tick();
    _queries++;
    _totalTime += osg::Timer::instance()->delta_s( start, end );

    return result;
}
