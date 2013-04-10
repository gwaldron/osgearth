/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include "ParallelKeyNodeFactory"
#include <osgEarth/Registry>
#include <osg/PagedLOD>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[ParallelKeyNodeFactory] "

//--------------------------------------------------------------------------

ParallelKeyNodeFactory::ParallelKeyNodeFactory(TileBuilder*             builder,
                                               const OSGTerrainOptions& options,
                                               const MapInfo&           mapInfo,
                                               TerrainNode*         terrain,
                                               UID                      engineUID ) :

SerialKeyNodeFactory( builder, options, mapInfo, terrain, engineUID )
{
    //NOP
}

osg::Node*
ParallelKeyNodeFactory::createRootNode( const TileKey& key )
{
    // NYI
    return 0L;
}

osg::Node*
ParallelKeyNodeFactory::createNode( const TileKey& key )
{
    // An event for synchronizing the completion of all requests:
    Threading::MultiEvent semaphore;

    // Collect all the jobs that can run in parallel (from all 4 subtiles)
    osg::ref_ptr<TileBuilder::Job> jobs[4];
    unsigned numTasks = 0;
    for( unsigned i=0; i<4; ++i )
    {
        jobs[i] = _builder->createJob( key.createChildKey(i), semaphore );
        if ( jobs[i].valid() )
            numTasks += jobs[i]->_tasks.size();
    }

    // Set up the sempahore to block for the correct number of tasks:
    semaphore.reset( numTasks );

    // Run all the tasks in parallel:
    for( unsigned i=0; i<4; ++i )
        if ( jobs[i].valid() )
            _builder->runJob( jobs[i].get() );

    // Wait for them to complete:
    semaphore.wait();

    // Now postprocess them and assemble into a tile group.
    osg::Group* root = new osg::Group();

    for( unsigned i=0; i<4; ++i )
    {
        if ( jobs[i].valid() )
        {
            osg::ref_ptr<Tile> tile;
            bool hasRealData;
            bool hasLodBlending;
            _builder->finalizeJob( jobs[i].get(), tile, hasRealData, hasLodBlending );
            if ( tile.valid() )
                addTile( tile.get(), hasRealData, hasLodBlending, root );
        }
    }

    //TODO: need to check to see if the group is empty, and do something different.
    return root;
}
