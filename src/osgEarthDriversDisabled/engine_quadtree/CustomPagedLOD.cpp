/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "CustomPagedLOD"

using namespace osgEarth_engine_quadtree;
using namespace osgEarth;

#define LC "[CustomPagedLOD] "


//----------------------------------------------------------------------------

CustomPagedLOD::CustomPagedLOD(TileNodeRegistry* live,
                               TileNodeRegistry* dead) :
_live( live ),
_dead( dead )
{
    //nop
}


CustomPagedLOD::~CustomPagedLOD()
{
    if ( _live.valid() || _dead.valid() )
    {
        for( unsigned i=0; i < getNumChildren(); ++i )
        {
            osg::ref_ptr<TileNode> node = dynamic_cast<TileNode*>( getChild(i) );
            if ( node.valid() )
            {
                if ( _live.valid() )
                    _live->remove( node.get() );
                if ( _dead.valid() )
                    _dead->add( node.get() );
            }
        }
    }
}


bool
CustomPagedLOD::addChild( osg::Node* child )
{
    bool ok = osg::PagedLOD::addChild( child );
    if ( ok && _live.valid() )
    {
        TileNodeGroup* tileGroup = dynamic_cast<TileNodeGroup*>( child );
        if ( tileGroup )
        {
            TileNodeVector tileNodes;
            tileNodes.reserve( 4 );

            for( unsigned i=0; i<tileGroup->getNumChildren(); ++i )
            {
                osg::Node* subChild = tileGroup->getChild(i);
                TileNode* tileNode = 0L;
                osg::Group* group = dynamic_cast<osg::PagedLOD*>(subChild);
                if ( group && group->getNumChildren() > 0 )
                    tileNode = dynamic_cast<TileNode*>( group->getChild(0) );
                else
                    tileNode = dynamic_cast<TileNode*>( child );
                if ( tileNode )
                    tileNodes.push_back( tileNode );
            }

            if ( !tileNodes.empty() )
                _live->add( tileNodes );
        }
    }
    return ok;
}


bool 
CustomPagedLOD::removeChildren(unsigned pos, unsigned numChildrenToRemove)
{
    if ( _live.valid() || _dead.valid() )
    {
        for( unsigned i=pos; i<pos+numChildrenToRemove; ++i )
        {
            if ( i < getNumChildren() )
            {
                osg::ref_ptr<TileNode> tile = dynamic_cast<TileNode*>( getChild(i) );
                if ( tile.valid() )
                {
                    if ( _live.valid() )
                        _live->remove( tile.get() );
                    if ( _dead.valid() )
                        _dead->add( tile.get() );
                }
            }
        }
    }
    return osg::PagedLOD::removeChildren( pos, numChildrenToRemove );
}
