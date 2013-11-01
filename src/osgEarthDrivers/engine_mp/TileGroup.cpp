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
#include "TileGroup"
#include "TileNodeRegistry"
#include "TilePagedLOD"

using namespace osgEarth_engine_mp;
using namespace osgEarth;

#define LC "[TileGroup] "

//#define OE_TEST OE_INFO
#define OE_TEST OE_NULL

RootTileGroup::RootTileGroup()
{
    //nop
}

void
RootTileGroup::addRootKey(const TileKey&    key,
                          osg::Node*        node,
                          const UID&        engineUID,
                          TileNodeRegistry* live,
                          TileNodeRegistry* dead,
                          osgDB::Options*   dbOptions)
{
    TilePagedLOD* lod = new TilePagedLOD(this, key, engineUID, live, dead);
    lod->setDatabaseOptions( dbOptions );
    lod->addChild( node );
    lod->setNumChildrenThatCannotBeExpired( 1 );
    lod->setFamilyReady( true );
    this->addChild( lod );
}

//----------------------------------------------------------------

TileGroup::TileGroup() :
_tilenode      ( 0L ),
_ignoreSubtiles( false ),
_subtileRange  ( FLT_MAX ),
_forceSubdivide( false )
{
    //nop
}


TileGroup::TileGroup(TileNode*         tilenode,
                     const UID&        engineUID,
                     TileNodeRegistry* live,
                     TileNodeRegistry* dead,
                     osgDB::Options*   dbOptions) :
_ignoreSubtiles( false ),
_subtileRange  ( FLT_MAX ),
_forceSubdivide( false )
{
    this->addChild( tilenode );
    _tilenode = tilenode;

    for(unsigned q=0; q<4; ++q)
    {
        TileKey subkey = tilenode->getKey().createChildKey(q);
        TilePagedLOD* lod = new TilePagedLOD(this, subkey, engineUID, live, dead);
        lod->setDatabaseOptions( dbOptions );
        lod->setCenter( tilenode->getBound().center() );
        lod->setRadius( tilenode->getBound().radius() );
        this->addChild( lod );
    }
}

void
TileGroup::setForceSubdivide(bool value)
{
    _forceSubdivide = value;
}

void
TileGroup::setTileNode(TileNode* tilenode)
{
    _tilenode = tilenode;
    this->setChild( 0, tilenode );

    // Should not really need to do this, but ok
    for(unsigned q=0; q<4; ++q)
    {
        TilePagedLOD* lod = static_cast<TilePagedLOD*>(_children[1+q].get());
        lod->setCenter( tilenode->getBound().center() );
        lod->setRadius( tilenode->getBound().radius() );
    }
}


void
TileGroup::setSubtileRange(float range)
{
    _subtileRange = range;
}


osg::BoundingSphere
TileGroup::computeBound() const
{
    if ( _tilenode )
        return _tilenode->computeBound();
    else
        return osg::Group::computeBound();
}


void
TileGroup::traverse(osg::NodeVisitor& nv)
{
    if ( _tilenode && nv.getTraversalMode() == nv.TRAVERSE_ACTIVE_CHILDREN )
    {
        float range = nv.getDistanceToViewPoint( getBound().center(), true );

        // collect information about the paged children:
        bool     considerSubtiles      = false;
        bool     subtileFamilyReady    = false;

        if ( range <= _subtileRange )
        {
            // if we're ignoring subtiles (because we preivously determined that they
            // were all upsampled), check to see if we need to re-access.
            if ( _ignoreSubtiles )
            {
                if ( getTileNode()->isOutOfDate() )
                {
                    _ignoreSubtiles = false;
                }
            }

            // if we're in range, consider whether to use the subtiles.
            if ( !_ignoreSubtiles )
            {
                unsigned numSubtilesLoaded     = 0;
                unsigned numSubtilesUpsampled  = 0;
                unsigned numSubtilesLoading    = 0;

                considerSubtiles = true;

                // collect stats on the (potential) subtiles:
                subtileFamilyReady = true;

                for( unsigned q=0; q<4; ++q )
                {
                    TilePagedLOD* plod = static_cast<TilePagedLOD*>(_children[1+q].get());

                    if ( plod->isCanceled() )
                    {
                        considerSubtiles = false;
                        break;
                    }
                    if ( plod->isLoaded() || plod->getNumChildrenThatCannotBeExpired() > 0 )
                        ++numSubtilesLoaded;

                    if ( plod->isUpsampled() )
                        ++numSubtilesUpsampled;

                    if ( plod->isLoading() )
                        ++numSubtilesLoading;
                }

                // if we don't have a complete set of loaded subtiles, OR is ALL
                // subtiles are upsampled, don't use them. (NOTE: numSubtilesLoading
                // also includes tiles that are updating/replacing their data, so do NOT
                // include this in the test.)
                if ( numSubtilesLoaded < 4 )
                {
                    subtileFamilyReady = false;
                }

                // if all the subtiles contain upsampled data, and none of them are trying
                // to load new data, we can ignore them all. (..unless "force" is on, which is the
                // case if we are trying to read a minLOD for the terrain.)
                if ( numSubtilesUpsampled >= 4 && numSubtilesLoading == 0 && !_forceSubdivide )
                {
                    considerSubtiles = false;
                    _ignoreSubtiles = true;
                }
            }
        }

        if ( considerSubtiles )
        {
            for( unsigned q=0; q<4; ++q )
            {
                TilePagedLOD* plod = static_cast<TilePagedLOD*>(_children[1+q].get());
                plod->setFamilyReady( subtileFamilyReady );
                plod->accept( nv );
            }

            // update the TileNode so it knows what frame we're in.
            if ( nv.getFrameStamp() )
            {
                _tilenode->setLastTraversalFrame( nv.getFrameStamp()->getFrameNumber() );
            }
        }

        if ( !considerSubtiles || !subtileFamilyReady || range > _subtileRange )
        {
            _tilenode->accept( nv );
        }
    }
    else
    {
        osg::Group::traverse( nv );
    }
}
