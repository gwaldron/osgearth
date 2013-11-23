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
#include "TilePagedLOD"
#include "TileNodeRegistry"
#include <osg/Version>
#include <cassert>

using namespace osgEarth_engine_mp;
using namespace osgEarth;

#define LC "[TilePagedLOD] "

//#define OE_TEST OE_INFO
#define OE_TEST OE_NULL

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
            TileNode* tn = dynamic_cast<TileNode*>( &node );
            if ( tn && _live )
            {
                _live->move( tn, _dead );
                _count++;
                //OE_NOTICE << "Expired " << tn->getKey().str() << std::endl;
            }
            traverse(node);
        }
    };
}


TilePagedLOD::TilePagedLOD(const UID&        engineUID,
                           TileNodeRegistry* live,
                           TileNodeRegistry* dead) :
osg::PagedLOD(),
_engineUID( engineUID ),
_live     ( live ),
_dead     ( dead )
{
    //nop
}

TilePagedLOD::~TilePagedLOD()
{
    // need this here b/c it's possible for addChild() to get called from
    // a pager dispatch even after the PLOD in question has been "expired"
    // so we still need to process the live/dead list.
    ExpirationCollector collector( _live.get(), _dead.get() );
    this->accept( collector );
}

TileNode*
TilePagedLOD::getTileNode()
{
    return _children.size() > 0 ? static_cast<TileNode*>(_children[0].get()) : 0L;
}

void
TilePagedLOD::setTileNode(TileNode* tilenode)
{
    // if the new tile has a culling callback, remove it and put it on the
    // PagedLOD itself as nature intended.
    if ( tilenode->getCullCallback() )
    {
        this->setCullCallback( tilenode->getCullCallback() );
        tilenode->setCullCallback( 0L );
    }
    setChild( 0, tilenode );
}

// The osgDB::DatabasePager will call this method when merging a new child
// into the scene graph.
bool
TilePagedLOD::addChild(osg::Node* node)
{
    if ( node )
    {
        // if we see an invalid tile marker, disable the paged lod.
        if ( dynamic_cast<InvalidTileNode*>(node) )
        {
            this->setFileName( 1, "" );
            this->setRange( 1, 0, 0 );
            this->setRange( 0, 0.0f, FLT_MAX );
            return true;
        }

        // If it's a TileNode, this is the simple first addition of the 
        // static TileNode child (not from the pager).
        TileNode* tilenode = dynamic_cast<TileNode*>( node );
        if ( tilenode && _live.get() )
        {
            _live->add( tilenode );
        }

        return osg::PagedLOD::addChild( node );
    }

    return false;
}


// The osgDB::DatabasePager will call this automatically to purge expired
// tiles from the scene graph.
bool
TilePagedLOD::removeExpiredChildren(double         expiryTime, 
                                    unsigned       expiryFrame, 
                                    osg::NodeList& removedChildren)
{
    if (_children.size()>_numChildrenThatCannotBeExpired)
    {
        unsigned cindex = _children.size() - 1;

        double   minExpiryTime   = 0.0;
        unsigned minExpiryFrames = 0;

        // these were added in osg 3.1.0+
#if OSG_VERSION_GREATER_OR_EQUAL(3,1,0)
        minExpiryTime   = _perRangeDataList[cindex]._minExpiryTime;
        minExpiryFrames = _perRangeDataList[cindex]._minExpiryFrames;
#endif

        if (!_perRangeDataList[cindex]._filename.empty() &&
            _perRangeDataList[cindex]._timeStamp   + minExpiryTime   < expiryTime &&
            _perRangeDataList[cindex]._frameNumber + minExpiryFrames < expiryFrame)
        {
            osg::Node* nodeToRemove = _children[cindex].get();
            removedChildren.push_back(nodeToRemove);

            ExpirationCollector collector( _live.get(), _dead.get() );
            nodeToRemove->accept( collector );

            OE_DEBUG << LC << "Expired " << collector._count << std::endl;

            return Group::removeChildren(cindex,1);
        }
    }
    return false;
}
