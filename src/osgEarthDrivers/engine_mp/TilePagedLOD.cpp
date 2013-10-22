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


TilePagedLOD::TilePagedLOD(TileGroup*        tilegroup,
                           const TileKey&    subkey,
                           const UID&        engineUID,
                           TileNodeRegistry* live,
                           TileNodeRegistry* dead) :
osg::PagedLOD(),
_tilegroup   ( tilegroup ),
_live        ( live ),
_dead        ( dead ),
_isUpsampled ( false ),
_isCanceled  ( false ),
_familyReady ( false )
{
    _numChildrenThatCannotBeExpired = 0;

    // set up the paging properties:
    _prefix = Stringify() << subkey.str() << "." << engineUID << ".";
    this->setRange   ( 0, 0.0f, FLT_MAX );
    this->setFileName( 0, Stringify() << _prefix << ".osgearth_engine_mp_tile" );
}


// The osgDB::DatabasePager will call this method when merging a new child
// into the scene graph.
bool
TilePagedLOD::addChild(osg::Node* node)
{
    // First check whether this is a new TileGroup (a group that contains a TileNode
    // and children paged LODs). If so, add it normally and inform our parent.
    TileGroup* subtilegroup = dynamic_cast<TileGroup*>(node);
    if ( subtilegroup )
    {
        TileNode* subtile = subtilegroup->getTileNode();
        _isUpsampled = !subtile->getTileModel()->hasRealData();
        _live->add( subtile );
        return osg::PagedLOD::addChild( node );
    }

    // If that fails, check whether this is a simple TileNode. This means that 
    // this is a leaf node in the graph (no children), or it's a replacement
    // tile for our existing tileNode, or possibly that it has no data at all 
    // (and we need to create an upsampled child to complete the required set of
    // four).
    TileNode* subtile = dynamic_cast<TileNode*>(node);
    if ( subtile )
    {
        if ( subtile->getTileModel()->getMapRevision() < _live->getMapRevision() )
        {
            //OE_NOTICE << LC << "Tile " << subtile->getKey().str() << " received data but it's already out of date...requeuing"
            //    << std::endl;
            return false;
        }
        else
        {
            if ( _children.size() == 0 )
            {
                _isUpsampled = !subtile->getTileModel()->hasRealData();

                // The initial valid tile node we've been waiting for. Insert it.
                _live->add( subtile );
                return osg::PagedLOD::addChild( node );
            }
            else
            {
                // A replacement tile. Replace the tile node we have with this
                // new version and update the registry.
                _isUpsampled = !subtile->getTileModel()->hasRealData();
                //if ( _isUpsampled )
                //    OE_NOTICE << LC << "Replaced UPSAMPLED leaf at " << _prefix << std::endl;

                if ( dynamic_cast<TileGroup*>(_children[0].get()) )
                {
                    subtile->setCullCallback( 0L );
                    static_cast<TileGroup*>(_children[0].get())->setTileNode( subtile );
                }
                else // TileNode
                {
                    this->setChild(0, subtile);
                }

                // remove the tile-replacement filename.
                _rangeList.resize( 1 );
                _perRangeDataList.resize( 1 );

                // update the registry. don't need to remove the old entry since add() will
                // replace the existing entry (they have the same key)
                _live->add( subtile );

                return true;
            }
        }
    }
    
    // Getting here means the Tile dies somewhere in the pager while the pager was
    // trying to add it. From what I can tell, this is normal and just happens sometimes
    if ( !node )
    {
        OE_DEBUG << LC << "TilePagedLOD: got an addChild(NULL) on " << _prefix << std::endl;
        _isCanceled = true;
    }

    return false;
}


void
TilePagedLOD::traverse(osg::NodeVisitor& nv)
{
    // Only traverse the TileNode if our neighbors (the other members of
    // our group of four) are ready as well.
    if ( _children.size() > 0 )
    {
        _children[0]->setNodeMask( _familyReady ? ~0 : 0 );

        // find our tile node:
        TileNode* tilenode = dynamic_cast<TileGroup*>(_children[0].get()) ? 
            static_cast<TileGroup*>(_children[0].get())->getTileNode() :
            static_cast<TileNode*>(_children[0].get());

        // Check whether the TileNode is marked dirty. If so, install a new pager request 
        // to reload and replace the TileNode.
        if (nv.getVisitorType() == nv.CULL_VISITOR &&
            this->getNumFileNames() < 2 && 
            tilenode->isOutOfDate() )
        {
            // lock keeps multiple CullVisitors from doing the same thing
            Threading::ScopedMutexLock exclusive( _updateMutex );

            if ( this->getNumFileNames() < 2 ) // double-check pattern
            {
                //OE_WARN << LC << "Queuing request for replacement: " << _container->getTileNode()->getKey().str() << std::endl;
                this->setFileName( 1, Stringify() << _prefix << ".osgearth_engine_mp_standalone_tile" );
                this->setRange( 1, 0, FLT_MAX );
            }
        }
    }

    osg::PagedLOD::traverse( nv );
}

namespace
{
    // traverses a node graph and moves any TileNodes from the LIVE
    // registry to the DEAD registry.
    struct ExpirationCollector : public osg::NodeVisitor
    {
        TileNodeRegistry* _live;
        TileNodeRegistry* _dead;

        ExpirationCollector(TileNodeRegistry* live, TileNodeRegistry* dead)
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), _live(live), _dead(dead) { }

        void apply(osg::Node& node)
        {
            TileNode* tn = 0L;
            TileGroup* tg = dynamic_cast<TileGroup*>(&node);
            tn = tg ? tg->getTileNode() : dynamic_cast<TileNode*>(&node);
            if ( tn )
            {
                if ( _live ) _live->remove( tn );
                if ( _dead ) _dead->add( tn );
                //OE_NOTICE << "Expired " << tn->getKey().str() << std::endl;
            }
            traverse(node);
        }
    };
}


// The osgDB::DatabasePager will call this automatically to purge expired
// tiles from the scene grpah.
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

            return Group::removeChildren(cindex,1);
        }
    }
    return false;
}
