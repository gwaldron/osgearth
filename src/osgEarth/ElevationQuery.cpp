#include <osgEarth/ElevationQuery>
#include <osgEarth/Locators>
#include <osgEarth/HeightFieldUtils>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[ElevationQuery] "

using namespace osgEarth;
using namespace OpenThreads;

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
    _tileSize         = 0;
    _maxLevelOverride = -1;
    _queries          = 0.0;
    _totalTime        = 0.0;  
    _maxTilesToCache = 500;
}

void
ElevationQuery::sync()
{
    if ( _mapf.sync() || _tileSize == 0  )
    {
        _tileSize = 0;        

        for( ElevationLayerVector::const_iterator i = _mapf.elevationLayers().begin(); i != _mapf.elevationLayers().end(); ++i )
        {
            // we need the maximum tile size
            int layerTileSize = i->get()->getTileSize();
            if ( layerTileSize > _tileSize )
                _tileSize = layerTileSize;
        }

        for (unsigned int i = 0; i < _caches.size(); i++)
        {
            delete _caches[i];
        }

        _caches.clear();

        for (unsigned int i = 0; i < _mapf.elevationLayers().size(); ++i)
        {
            _caches.push_back( new TileCache(_maxTilesToCache) );
        }
        

    }
}

unsigned int
ElevationQuery::getMaxLevel( double x, double y, const SpatialReference* srs, const Profile* profile ) const
{
    unsigned int maxLevel = 0;
    for( ElevationLayerVector::const_iterator i = _mapf.elevationLayers().begin(); i != _mapf.elevationLayers().end(); ++i )
    {
        // skip disabled layers
        if ( !i->get()->getEnabled() )
            continue;

        unsigned int layerMax = 0;

        osgEarth::TileSource* ts = i->get()->getTileSource();
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
                    if (j->maxLevel().isSet() && j->maxLevel() > layerMax && j->contains( tsCoord.x(), tsCoord.y(), tsSRS ))
                    {
                        layerMax = j->maxLevel().value();
                    }
                }

                //Need to convert the layer max of this TileSource to that of the actual profile
                layerMax = profile->getEquivalentLOD( ts->getProfile(), layerMax );            
            }

            // cap the max to the layer's express max level (if set).
            if ( i->get()->getTerrainLayerRuntimeOptions().maxLevel().isSet() )
            {
                layerMax = std::min( layerMax, *i->get()->getTerrainLayerRuntimeOptions().maxLevel() );
            }
        }
        else
        {
            // no TileSource? probably in cache-only mode. Use the layer max (or its default).
            layerMax = i->get()->getTerrainLayerRuntimeOptions().maxLevel().value();
        }

        if (layerMax > maxLevel) maxLevel = layerMax;
    }    

    // need to check the image layers too, because if image layers go deeper than elevation layers,
    // upsampling occurs that can change the formation of the terrain skin.
    // NOTE: this doesn't happen in "triangulation" interpolation mode.
    if ( _mapf.getMapInfo().getElevationInterpolation() != osgEarth::INTERP_TRIANGULATE )
    {
        for( ImageLayerVector::const_iterator i = _mapf.imageLayers().begin(); i != _mapf.imageLayers().end(); ++i )
        {
            // skip disabled layers
            if ( !i->get()->getEnabled() )
                continue;

            unsigned int layerMax = 0;
            osgEarth::TileSource* ts = i->get()->getTileSource();
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
                        if (j->maxLevel().isSet()  && j->maxLevel() > layerMax && j->contains( tsCoord.x(), tsCoord.y(), tsSRS ))
                        {
                            layerMax = j->maxLevel().value();
                        }
                    }

                    // Need to convert the layer max of this TileSource to that of the actual profile
                    layerMax = profile->getEquivalentLOD( ts->getProfile(), layerMax );            
                }        
                
                if ( i->get()->getTerrainLayerRuntimeOptions().maxLevel().isSet() )
                {
                    layerMax = std::min( layerMax, *i->get()->getTerrainLayerRuntimeOptions().maxLevel() );
                }
            }
            else
            {
                // no TileSource? probably in cache-only mode. Use the layer max (or its default).
                layerMax = i->get()->getTerrainLayerRuntimeOptions().maxLevel().value();
            }

            if (layerMax > maxLevel)
                maxLevel = layerMax;
        }
    }

    if (maxLevel == 0) 
    {
        //This means we had no data extents on any of our layers and no max levels are set
    }

    return maxLevel;
}

void
ElevationQuery::setMaxTilesToCache( int value )
{
    if (_maxTilesToCache != value)
    {
        _maxTilesToCache = value;
        for (unsigned int i = 0; i < _caches.size(); i++)
        {
            _caches[i]->setMaxSize( _maxTilesToCache );
        }
    }
}

int
ElevationQuery::getMaxTilesToCache() const
{
    return _maxTilesToCache;    
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

    //This is the max resolution that we actually have data at this point
    unsigned int bestAvailLevel = getMaxLevel( point.x(), point.y(), point.getSRS(), _mapf.getProfile());

    if (desiredResolution > 0.0)
    {
        unsigned int desiredLevel = _mapf.getProfile()->getLevelOfDetailForHorizResolution( desiredResolution, _tileSize );
        if (desiredLevel < bestAvailLevel) bestAvailLevel = desiredLevel;
    }

    OE_DEBUG << LC << "Best available data level " << point.x() << ", " << point.y() << " = "  << bestAvailLevel << std::endl;

    // transform the input coords to map coords:
    GeoPoint mapPoint = point;
    if ( point.isValid() && !point.getSRS()->isEquivalentTo( _mapf.getProfile()->getSRS() ) )
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
        unsigned int index = 0;
        // Loop over each elevation layer and try to get a sample from them at the location.
        for (ElevationLayerVector::const_reverse_iterator i = _mapf.elevationLayers().rbegin(); i != _mapf.elevationLayers().rend(); i++)
        {            
            // See if this layer has data for the requested tile key
            ElevationLayer* layer = i->get();
            //OE_DEBUG << LC << "Trying " << layer->getName() << std::endl;
            if (layer->getEnabled() && layer->getVisible())
            {
                if (layer->getTileSource() == 0 || layer->getTileSource()->hasData( key ) )
                {
                    GeoHeightField hf;
                    TileCache::Record record;
                    if (_caches[index]->get( key, record))
                    {
                        //OE_NOTICE << "Hit " << key.str() << std::endl;
                        hf = record.value();
                    }
                    else
                    {
                        //OE_NOTICE << "Miss " << key.str() << std::endl;
                        hf = layer->createHeightField( key );
                        if (hf.valid())
                        {
                            _caches[index]->insert( key, hf );
                        }
                    }
                    if (hf.valid())
                    {                    
                        float elevation = 0.0f;                 
                        result = hf.getElevation( mapPoint.getSRS(), mapPoint.x(), mapPoint.y(), _mapf.getMapInfo().getElevationInterpolation(), mapPoint.getSRS(), elevation);                     
                        if (result && elevation != NO_DATA_VALUE)
                        {                        
                            // see what the actual resolution of the heightfield is.
                            if ( out_actualResolution )
                                *out_actualResolution = hf.getXInterval(); 
                            out_elevation = (double)elevation;
                            //OE_NOTICE << "Got elevation " <<  out_elevation << " from layer " << layer->getName() << std::endl;
                            break;
                        }
                        else
                        {                               
                            result = false;                     
                        }
                    } 
                }
                else
                {
                    //OE_NOTICE << "Skipping hf creation for layer " << layer->getName() << std::endl;
                }
            }
            index++;
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
