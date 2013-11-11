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
            if ( tn )
            {
                if ( _live ) _live->remove( tn );
                if ( _dead ) _dead->add( tn );
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
    ExpirationCollector collector( _live, _dead );
    this->accept( collector );
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

        // register new additions.
        TileNode* tilenode = dynamic_cast<TileNode*>( node );
        if ( tilenode )
        {
            _live->add( tilenode );
        }

        return osg::PagedLOD::addChild( node );
    }

    // TODO: incremental.
    return false;
}


void
TilePagedLOD::traverse(osg::NodeVisitor& nv)
{
    // Only traverse the TileNode if our neighbors (the other members of
    // our group of four) are ready as well.
    if ( _children.size() > 0 && nv.getVisitorType() == nv.CULL_VISITOR )
    {
        TileNode* tilenode = static_cast<TileNode*>(_children[0].get());

        // Check whether the TileNode is marked dirty. If so, install a new pager request 
        // to reload and replace the TileNode.
        if (this->getNumFileNames() < 2 && tilenode->isOutOfDate())
        {
            // lock keeps multiple traversals from doing the same thing
            Threading::ScopedMutexLock exclusive( _updateMutex );

            if ( this->getNumFileNames() < 2 ) // double-check pattern
            {
                //OE_DEBUG << LC << "Queuing request for replacement: " << _container->getTileNode()->getKey().str() << std::endl;
                //this->setFileName( 2, Stringify() << _prefix << ".osgearth_engine_mp_standalone_tile" );
                //this->setRange   ( 2, 0, FLT_MAX );
            }
        }
    }

    osg::PagedLOD::traverse( nv );
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

            ExpirationCollector collector( _live, _dead );
            nodeToRemove->accept( collector );

            OE_DEBUG << LC << "Expired " << collector._count << std::endl;

            return Group::removeChildren(cindex,1);
        }
    }
    return false;
}
