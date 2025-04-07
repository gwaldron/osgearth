/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "DetailExtension"
#include "DetailTerrainEffect"
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::Detail;

#define LC "[DetailExtension] "


DetailExtension::DetailExtension()
{
    //nop
}

DetailExtension::DetailExtension(const DetailOptions& options) :
_options( options )
{
    //nop
}

DetailExtension::~DetailExtension()
{
    //nop
}

void
DetailExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
DetailExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

    _effect = new DetailTerrainEffect( _options );

    mapNode->getTerrainEngine()->addEffect( _effect.get() );
    
    //OE_INFO << LC << "Installed!\n";

    return true;
}

bool
DetailExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
    }
    _effect = 0L;
    return true;
}

