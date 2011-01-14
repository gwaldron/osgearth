/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[ParallelKeyNodeFactory] "

//--------------------------------------------------------------------------

namespace
{
    struct TileBuildRequest : public TaskRequest
    {
        TileBuildRequest( const TileKey& key, TileBuilder* builder ) 
            : _key(key), _builder(builder) { }

        void operator()( ProgressCallback* pc )
        {
            _builder->createTile( _key, _tile, _hasRealData );
        }

        osg::ref_ptr<TileBuilder> _builder;
        const TileKey _key;
        osg::ref_ptr<CustomTile> _tile;
        bool _hasRealData;
    };

    struct SignalProgress : public ProgressCallback
    {
        SignalProgress( Threading::MultiEvent* ev ) : _ev( ev ) { }

        void onCompleted() {
            if ( _ev )
                _ev->notify();
        }

        Threading::MultiEvent* _ev;
    };
}

//--------------------------------------------------------------------------

ParallelKeyNodeFactory::ParallelKeyNodeFactory(TileBuilder* builder,
                                               const TerrainOptions& options,
                                               CustomTerrain* terrain,
                                               UID engineUID ) :
SerialKeyNodeFactory( builder, options, terrain, engineUID )
{
    //NOP
}

osg::Node*
ParallelKeyNodeFactory::createNode( const TileKey& key )
{
    // An event for synchronizing the completion of all requests:
    Threading::MultiEvent semaphore(4);
    osg::ref_ptr<ProgressCallback> prog = new SignalProgress( &semaphore );

    // Build all four subtiles in parallel:
    osg::ref_ptr<TileBuildRequest> _requests[4];
    for( unsigned i = 0; i < 4; ++i )
    {
        TileKey child = key.createChildKey( i );
        _requests[i] = new TileBuildRequest( child, _builder );
        _requests[i]->setProgressCallback( prog.get() );
        _requests[i]->setPriority( -(float)child.getLevelOfDetail() ); // lower LODs get higher priority
        _builder->getTaskService()->add( _requests[i].get() );
    }

    // Wait for all requests to complete:
    semaphore.wait();

    // Now postprocess them and assemble into a tile group.
    osg::Group* root = new osg::Group();

    for( unsigned i = 0; i < 4; ++i )
    {
        CustomTile* tile = _requests[i]->_tile.get();
        if ( tile )
        {
            addTile( tile, _requests[i]->_hasRealData, root );
        }
    }

    //TODO: need to check to see if the group is empty, and do something different.
    return root;
}

