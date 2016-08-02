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
#include "Unloader"
#include "TileNode"
#include "TileNodeRegistry"

using namespace osgEarth::Drivers::RexTerrainEngine;


//........................................................................

namespace
{
    // traverses a node graph and moves any TileNodes from the LIVE
    // registry to the DEAD registry.
    struct ExpirationCollector : public osg::NodeVisitor
    {
        TileNodeRegistry*      _tiles;
        unsigned               _count;

        ResourceReleaser::ObjectList _nodes;

        ExpirationCollector(TileNodeRegistry* tiles)
            : _tiles(tiles), _count(0)
        {
            // set up to traverse the entire subgraph, ignoring node masks.
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            setNodeMaskOverride( ~0 );
        }

        void apply(osg::Node& node)
        {
            // Make sure the tile is still dormat before releasing it
            TileNode* tn = dynamic_cast<TileNode*>( &node );
            if ( tn )
            {
                _nodes.push_back(tn);
                _tiles->remove( tn );
                _count++;
            }
            traverse(node);
        }
    };
}

//........................................................................


#undef  LC
#define LC "[UnloaderGroup] "

UnloaderGroup::UnloaderGroup(TileNodeRegistry* tiles) :
_tiles(tiles),
_threshold( INT_MAX )
{
    this->setNumChildrenRequiringUpdateTraversal( 1u );
}

void
UnloaderGroup::unloadChildren(const std::vector<TileKey>& keys)
{
    _mutex.lock();
    for(std::vector<TileKey>::const_iterator i = keys.begin(); i != keys.end(); ++i)
        _parentKeys.push_back( *i );
    _mutex.unlock();
}

void
UnloaderGroup::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        if ( _parentKeys.size() > _threshold )
        {
            unsigned unloaded=0, notFound=0, notDormant=0;
            Threading::ScopedMutexLock lock( _mutex );
            for(std::vector<TileKey>::const_iterator parentKey = _parentKeys.begin(); parentKey != _parentKeys.end(); ++parentKey)
            {
                osg::ref_ptr<TileNode> parentNode;
                if ( _tiles->get(*parentKey, parentNode) )
                {
                    // re-check for dormancy in case something has changed
                    if ( parentNode->areSubTilesDormant(nv.getFrameStamp()) )
                    {
                        // find and move all tiles to be unloaded to the dead pile.
                        ExpirationCollector collector( _tiles );
                        for(unsigned i=0; i<parentNode->getNumChildren(); ++i)
                            parentNode->getSubTile(i)->accept( collector );
                        unloaded += collector._count;

                        // submit all collected nodes for GL resource release:
                        if (!collector._nodes.empty() && _releaser.valid())
                            _releaser->push(collector._nodes);

                        parentNode->removeSubTiles();
                    }
                    else notDormant++;
                }
                else notFound++;
            }

            //OE_NOTICE << LC << "Total=" << _parentKeys.size() << "; threshold=" << _threshold << "; unloaded=" << unloaded << "; notDormant=" << notDormant << "; notFound=" << notFound << "; live=" << _live->size() << "\n";
            _parentKeys.clear();
        }
    }
    osg::Group::traverse( nv );
}

