/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/CacheSeed>
#include <osgEarth/ImageLayer>

#define LC "[CacheSeed] "

using namespace osgEarth;
using namespace osgEarth::Contrib;
using namespace osgEarth::Util;

CacheTileHandler::CacheTileHandler( TileLayer* layer, const Map* map ):
_layer( layer ),
_map( map )
{
}

bool CacheTileHandler::handleTile(const TileKey& key, const TileVisitor& tv)
{        
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );    

    // Just call createImage or createHeightField on the layer and the it will be cached!
    if (imageLayer)
    {                
        GeoImage image = imageLayer->createImage( key );
        if (image.valid())
        {                
            return true;
        }            
    }
    else if (elevationLayer )
    {
        GeoHeightField hf = elevationLayer->createHeightField(key, 0L);
        if (hf.valid())
        {                
            return true;
        }            
    }

    // If we didn't produce a result but the key isn't within range then we should continue to 
    // traverse the children b/c a min level was set.
    if (!_layer->isKeyInLegalRange(key))
    {
        return true;
    }

    return false;        
}   

bool CacheTileHandler::hasData( const TileKey& key ) const
{
    return _layer->mayHaveData(key);
}

std::string CacheTileHandler::getProcessString() const
{
    std::stringstream buf;
    ImageLayer* imageLayer = dynamic_cast< ImageLayer* >( _layer.get() );
    ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer* >( _layer.get() );    

    unsigned index = _map->getIndexOfLayer(_layer.get());
    if (index < _map->getNumLayers())
    {
        buf << "osgearth_cache --seed ";
        if (imageLayer)
        {
            buf << " --image " << index << " ";
        }
        else if (elevationLayer)
        {
            buf << " --elevation " << index << " ";
        }
    }
    return buf.str();
}



/***************************************************************************************/

CacheSeed::CacheSeed():
_visitor(new TileVisitor())
{
}

TileVisitor* CacheSeed::getVisitor() const
{
    return _visitor.get();
}

void CacheSeed::setVisitor(TileVisitor* visitor)
{
    _visitor = visitor;
}

void CacheSeed::run( TileLayer* layer, const Map* map )
{
    _visitor->setTileHandler( new CacheTileHandler( layer, map ) );
    _visitor->run( map->getProfile() );
}