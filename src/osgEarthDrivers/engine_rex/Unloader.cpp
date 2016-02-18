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
        TileNodeRegistry*      _live;
        TileNodeRegistry*      _dead;
        unsigned               _count;

        ExpirationCollector(TileNodeRegistry* live, TileNodeRegistry* dead)
            : _live(live), _dead(dead), _count(0)
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
                _live->move( tn, _dead );
                _count++;
            }
            traverse(node);
        }
    };
}

//........................................................................

#undef  LC
#define LC "[UnloaderGroup] "

UnloaderGroup::UnloaderGroup(TileNodeRegistry* live, TileNodeRegistry* dead) :
_live     ( live ),
_dead     ( dead ),
_threshold( INT_MAX )
{
    this->setNumChildrenRequiringUpdateTraversal( 1u );
}

#if 0
void
UnloaderGroup::unloadChildren(const TileKey& key)
{
    _mutex.lock();
    ///_parentKeys.insert(key);
    _parentKeys.push_back( key ); 
    _mutex.unlock();
}
#endif

void
UnloaderGroup::unloadChildren(const std::vector<TileKey>& keys)
{
    _mutex.lock();
    for(std::vector<TileKey>::const_iterator i = keys.begin(); i != keys.end(); ++i)
        _parentKeys.push_back( *i );
    //_parentKeys.insert(keys.begin(), keys.end());
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
                if ( _live->get(*parentKey, parentNode) )
                {
                    // re-check for dormancy in case something has changed
                    if ( parentNode->areSubTilesDormant(nv.getFrameStamp()) )
                    {
                        // find and move all tiles to be unloaded to the dead pile.
                        ExpirationCollector collector( _live, _dead );
                        for(unsigned i=0; i<parentNode->getNumChildren(); ++i)
                            parentNode->getSubTile(i)->accept( collector );
                        unloaded += collector._count;
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

