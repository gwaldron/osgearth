/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include "TileGroup"
#include "TilePagedLOD"
#include "TileNode"

#include <osg/NodeVisitor>

using namespace osgEarth::Drivers::MPTerrainEngine;

#define LC "[TileGroup] "

namespace
{
    struct UpdateAgent : public osg::PagedLOD
    {
        UpdateAgent(TileGroup* tilegroup) : _tilegroup(tilegroup)
        {
            std::string fn = Stringify()
                << tilegroup->getKey().str()
                << "." << tilegroup->getEngineUID()
                << ".osgearth_engine_mp_standalone_tile";

            this->setFileName(0, fn);
            this->setRange   (0, 0, FLT_MAX);
            this->setCenter  (tilegroup->getBound().center());
        }

        virtual bool addChild(osg::Node* node)
        {
            if ( node )
            {
                osg::ref_ptr<TileGroup> tilegroup;
                if ( _tilegroup.lock(tilegroup) )
                {
                    tilegroup->applyUpdate( node );
                    this->_perRangeDataList.resize(0);
                }
            }
            else
            {
                OE_DEBUG << LC << "Internal: UpdateAgent for " << _tilegroup->getKey().str() << "received a NULL add."
                    << std::endl;
            }
            return true;
        }

        osg::observer_ptr<TileGroup> _tilegroup;
    };
}

//------------------------------------------------------------------------

TileGroup::TileGroup(const TileKey&    key, 
                     const UID&        engineUID,
                     TileNodeRegistry* live,
                     ResourceReleaser* releaser) :
_key      ( key ),
_engineUID( engineUID ),
_live     ( live ),
_releaser ( releaser )
{
    this->setName( key.str() );
}

TileNode*
TileGroup::getTileNode(unsigned q)
{
    osg::Node* child = getChild(q);
    TilePagedLOD* plod = dynamic_cast<TilePagedLOD*>( child );
    if ( plod ) return plod->getTileNode();
    return static_cast<TileNode*>( child );
}

void
TileGroup::applyUpdate(osg::Node* node)
{
    if ( node )
    {
        OE_DEBUG << LC << "Update received for tile " << _key.str() << std::endl;

        InvalidTileNode* invalid = dynamic_cast<InvalidTileNode*>( node );
        if ( invalid )
        {
            OE_WARN << LC << "Invalid node received (" << _key.str() << ")\n";
            return;
        }

        TileGroup* update = dynamic_cast<TileGroup*>( node );
        if ( !update )
        {
            OE_WARN << LC << "Internal error: update was not a TileGroup" << std::endl;
            return;
        }

        if ( update->getNumChildren() < 4 )
        {
            OE_WARN << LC << "Internal error: update did not have 4 children" << std::endl;
            return;
        }

        for(unsigned i=0; i<4; ++i)
        {
            TileNode* newTileNode = dynamic_cast<TileNode*>( update->getChild(i) );
            if ( !newTileNode )
            {
                OE_WARN << LC << "Internal error; update child was not a TileNode" << std::endl;
                return;
            }

            osg::ref_ptr<TileNode> oldTileNode = 0L;

            TilePagedLOD* plod = dynamic_cast<TilePagedLOD*>(_children[i].get());
            if ( plod )
            {
                oldTileNode = plod->getTileNode();
                plod->setTileNode( newTileNode );
                if ( _live.valid() )
                {
                    _live->remove(oldTileNode.get());
                    _releaser->push(oldTileNode.get());
                }
            }
            else
            {
                // must be a TileNode leaf, so replace it here.
                oldTileNode = dynamic_cast<TileNode*>(_children[i].get());
                if ( !oldTileNode.valid() )
                {
                    OE_WARN << LC << "Internal error; existing child was not a TilePagedLOD or a TileNode" << std::endl;
                    return;
                }

                this->setChild( i, newTileNode );
                if ( _live.valid() )
                {
                    _live->remove(oldTileNode.get());
                    _releaser->push(oldTileNode.get());
                }
            }

            if ( _live.valid() )
            {
                _live->add( newTileNode );
            }
        }
    }

    // deactivate the update agent
    _updateAgent = 0L;
}

void
TileGroup::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // only check for update if an update isn't already in progress:
        if ( !_updateAgent.valid() )
        {
            bool updateRequired = false;
            for( unsigned q=0; q<4; ++q)
            {
                if ( getTileNode(q)->isOutOfDate() )
                {
                    updateRequired = true;
                    break;
                }
            }

            if ( updateRequired )
            {
                // lock keeps multiple traversals from doing the same thing
                Threading::ScopedMutexLock exclusive( _updateMutex );

                // double check to prevent a race condition:
                if ( !_updateAgent.valid() )
                {
                    _updateAgent = new UpdateAgent(this);
                }
            }
        }

        if ( _updateAgent.valid() )
        {
            _updateAgent->accept( nv );
        }
    }

    osg::Group::traverse( nv );
}
