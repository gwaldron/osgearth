/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "TilePagedLOD"
#include "TileNodeRegistry"
#include <osg/Version>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <cassert>

using namespace osgEarth::Drivers::MPTerrainEngine;
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


bool
TilePagedLOD::MyProgressCallback::isCanceled()
{
    if (!ProgressCallback::isCanceled() &&
        _frameOfLastCull > 0 && 
        ((int)_tiles->getTraversalFrame() - (int)_frameOfLastCull > 2))
    {
        _frameOfLastCull = 0;
        cancel();
        stats().clear();
    }
    return ProgressCallback::isCanceled();
}

void
TilePagedLOD::MyProgressCallback::update(unsigned frame)
{
    if ( ProgressCallback::isCanceled() )
    {
        // this lets up re-use a progress callback if the tile was previously
        // canceled and then queued up again later without being expired
        reset();
        _frameOfLastCull = 0;
    }
    else
    {
        _frameOfLastCull = frame;
    }
}


TilePagedLOD::TilePagedLOD(const UID&        engineUID,
                           TileNodeRegistry* live,
                           TileNodeRegistry* dead) :
osg::PagedLOD(),
_engineUID( engineUID ),
_live     ( live ),
_dead     ( dead ),
_debug    ( false )
{
    if ( live )
    {
        _progress = new MyProgressCallback();
        _progress->_frameOfLastCull = 0;
        _progress->_tiles = live;
        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setUserData( _progress.get() );
        setDatabaseOptions( options );
    }
}

TilePagedLOD::~TilePagedLOD()
{
    // need this here b/c it's possible for addChild() to get called from
    // a pager dispatch even after the PLOD in question has been "expired"
    // so we still need to process the live/dead list.
    ExpirationCollector collector( _live.get(), _dead.get() );
    this->accept( collector );
}

osgDB::Options*
TilePagedLOD::getOrCreateDBOptions()
{
    if ( !getDatabaseOptions() )
        setDatabaseOptions( Registry::instance()->cloneOrCreateOptions() );
    return static_cast<osgDB::Options*>(getDatabaseOptions());
}

void
TilePagedLOD::setChildBoundingBoxAndMatrix(int                     childNum,
                                           const osg::BoundingBox& bbox,
                                           const osg::Matrix&      matrix)
{
    _childBBoxes.resize(childNum+1);
    _childBBoxes[childNum] = bbox;
    _childBBoxMatrices.resize(childNum+1);
    _childBBoxMatrices[childNum] = matrix;
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
        // if we see an invalid tile marker, disable the paged lod slot.
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

            // Listen for out east and south neighbors.
            const TileKey& key = tilenode->getKey();
            _live->listenFor( key.createNeighborKey(1, 0), tilenode );
            _live->listenFor( key.createNeighborKey(0, 1), tilenode );
        }

        return osg::PagedLOD::addChild( node );
    }

    return false;
}


// MOST of this is copied and pasted from OSG's osg::PagedLOD::traverse,
// except where otherwise noted with an "osgEarth" comment.
void
TilePagedLOD::traverse(osg::NodeVisitor& nv)
{
    // set the frame number of the traversal so that external nodes can find out how active this
    // node is.
    if (nv.getFrameStamp() &&
        nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
    {
        setFrameNumberOfLastTraversal(nv.getFrameStamp()->getFrameNumber());
        
        // osgEarth: update our progress tracker to prevent tile cancelation.
        if (_progress.valid())
        {
            _progress->update( nv.getFrameStamp()->getFrameNumber() );
        }
    }

    double timeStamp = nv.getFrameStamp()?nv.getFrameStamp()->getReferenceTime():0.0;
    unsigned int frameNumber = nv.getFrameStamp()?nv.getFrameStamp()->getFrameNumber():0;
    bool updateTimeStamp = nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR;

    switch(nv.getTraversalMode())
    {
    case(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN):
            std::for_each(_children.begin(),_children.end(),osg::NodeAcceptOp(nv));
            break;
        case(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN):
        {
            float required_range = 0;
            if (_rangeMode==DISTANCE_FROM_EYE_POINT)
            {
                required_range = nv.getDistanceToViewPoint(getCenter(),true);
            }
            else
            {
                osg::CullStack* cullStack = dynamic_cast<osg::CullStack*>(&nv);
                if (cullStack && cullStack->getLODScale()>0.0f)
                {
                    required_range = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();
                }
                else
                {
                    // fallback to selecting the highest res tile by
                    // finding out the max range
                    for(unsigned int i=0;i<_rangeList.size();++i)
                    {
                        required_range = osg::maximum(required_range,_rangeList[i].first);
                    }
                }
            }

            int lastChildTraversed = -1;
            bool needToLoadChild = false;
            for(unsigned int i=0;i<_rangeList.size();++i)
            {
                if (_rangeList[i].first<=required_range && required_range<_rangeList[i].second)
                {
                    if (i<_children.size())
                    {
                        if (updateTimeStamp)
                        {
                            _perRangeDataList[i]._timeStamp=timeStamp;
                            _perRangeDataList[i]._frameNumber=frameNumber;
                        }

                        _children[i]->accept(nv);
                        lastChildTraversed = (int)i;
                    }
                    else
                    {
                        needToLoadChild = true;
                    }
                }
            }

            if (needToLoadChild)
            {
                unsigned int numChildren = _children.size();

                // select the last valid child.
                if (numChildren>0 && ((int)numChildren-1)!=lastChildTraversed)
                {
                    if (updateTimeStamp)
                    {
                        _perRangeDataList[numChildren-1]._timeStamp=timeStamp;
                        _perRangeDataList[numChildren-1]._frameNumber=frameNumber;
                    }
                    _children[numChildren-1]->accept(nv);
                }

                // now request the loading of the next unloaded child.
                if (!_disableExternalChildrenPaging &&
                    nv.getDatabaseRequestHandler() &&
                    numChildren<_perRangeDataList.size())
                {
                    // osgEarth: Perform a tile visibility check before requesting the new tile.
                    // Intersect the tile's earth-aligned bounding box with the current culling frustum.
                    bool tileIsVisible = true;

                    if (nv.getVisitorType() == nv.CULL_VISITOR &&
                        numChildren < _childBBoxes.size() &&
                        _childBBoxes[numChildren].valid())
                    {
                        osgUtil::CullVisitor* cv = Culling::asCullVisitor( nv );
                        // wish that CullStack::createOrReuseRefMatrix() was public
                        osg::ref_ptr<osg::RefMatrix> mvm = new osg::RefMatrix(*cv->getModelViewMatrix());
                        mvm->preMult( _childBBoxMatrices[numChildren] );
                        cv->pushModelViewMatrix( mvm.get(), osg::Transform::RELATIVE_RF );
                        tileIsVisible = !cv->isCulled( _childBBoxes[numChildren] );
                        cv->popModelViewMatrix();
                    }

                    if ( tileIsVisible )
                    {
                        // [end:osgEarth]

                        // compute priority from where abouts in the required range the distance falls.
                        float priority = (_rangeList[numChildren].second-required_range)/(_rangeList[numChildren].second-_rangeList[numChildren].first);

                        // invert priority for PIXEL_SIZE_ON_SCREEN mode
                        if(_rangeMode==PIXEL_SIZE_ON_SCREEN)
                        {
                            priority = -priority;
                        }

                        // modify the priority according to the child's priority offset and scale.
                        priority = _perRangeDataList[numChildren]._priorityOffset + priority * _perRangeDataList[numChildren]._priorityScale;

                        if (_databasePath.empty())
                        {
                            nv.getDatabaseRequestHandler()->requestNodeFile(_perRangeDataList[numChildren]._filename,nv.getNodePath(),priority,nv.getFrameStamp(), _perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
                        }
                        else
                        {
                            // prepend the databasePath to the child's filename.
                            nv.getDatabaseRequestHandler()->requestNodeFile(_databasePath+_perRangeDataList[numChildren]._filename,nv.getNodePath(),priority,nv.getFrameStamp(), _perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
                        }
                    }
                }
            }

           break;
        }
        default:
            break;
    }
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

            if ( _debug )
            {
                TileNode* tileNode = getTileNode();
                std::string key = tileNode ? tileNode->getKey().str() : "unk";
                OE_NOTICE 
                    << LC << "Tile " << key << " : expiring " << collector._count << " children; "
                    << "TS = " << _perRangeDataList[cindex]._timeStamp
                    << ", MET = " << minExpiryTime
                    << ", ET = " << expiryTime
                    << "; FN = " << _perRangeDataList[cindex]._frameNumber
                    << ", MEF = " << minExpiryFrames
                    << ", EF = " << expiryFrame
                    << "\n";
            }

            return Group::removeChildren(cindex,1);
        }
    }
    return false;
}
