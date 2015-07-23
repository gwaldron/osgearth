/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include "ExpireTiles"
#include <osg/NodeVisitor>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[ExpireTiles] "

namespace
{
    // traverses a node graph and moves any TileNodes from the LIVE
    // registry to the DEAD registry.
    struct ExpirationCollector : public osg::NodeVisitor
    {
        TileNodeRegistry* _live;
        TileNodeRegistry* _dead;
        unsigned          _count;

        ExpirationCollector(TileNodeRegistry* live, TileNodeRegistry* dead)
            : _live(live), _dead(dead), _count(0)
        {
            // set up to traverse the entire subgraph, ignoring node masks.
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            setNodeMaskOverride( ~0 );
        }

        void apply(osg::Node& node)
        {
            // TODO: perhaps check again that the tile is still dormant, so we don't expire
            // a tile that will immediately be recreated. For this we will need a valid framestamp
            TileNode* tn = dynamic_cast<TileNode*>( &node );
            if ( tn && _live )
            {
                _live->move( tn, _dead );
                _count++;
            }
            traverse(node);
        }
    };
}

//............................................................................


ExpireTiles::ExpireTiles(TileNode* tilenode, EngineContext* context) :
_tilenode(tilenode),
_context(context)
{
    //nop
}

void
ExpireTiles::apply()
{
    osg::ref_ptr<TileNode> tilenode;
    if ( _tilenode.lock(tilenode) )
    {
        // Collect and report all the expired tiles:
        unsigned count = 0;
        ExpirationCollector collector( _context->liveTiles(), _context->deadTiles() );
        for(unsigned i=0; i<tilenode->getNumChildren(); ++i)
        {
            tilenode->getChild(i)->accept( collector );
            count += collector._count;
        }

        // Remove them from the node.
        tilenode->removeChildren( 0, tilenode->getNumChildren() );

        OE_DEBUG << LC << "Expired " << count << " children; live = "
            << _context->liveTiles()->size()
            << "\n";
    }
}
