/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/TileHandler>


using namespace osgEarth;
using namespace osgEarth::Util;

bool TileHandler::handleTile(const TileKey& key, const TileVisitor& tv)
{
    return true;    
}

bool TileHandler::hasData( const TileKey& key ) const
{
    return true;
}
        
std::string TileHandler::getProcessString() const
{
    return "";
}
