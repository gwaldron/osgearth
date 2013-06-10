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


TileGroup::TileGroup(TileNode*         tilenode,
                     const UID&        engineUID,
                     TileNodeRegistry* live,
                     TileNodeRegistry* dead,
                     osgDB::Options*   dbOptions)
{
    _numSubtilesUpsampling = 0;
    _numSubtilesLoaded     = 0;
    _traverseSubtiles      = true;

    this->addChild( tilenode );
    _tilenode = tilenode;

    for(unsigned q=0; q<4; ++q)
    {
        TileKey subkey = tilenode->getKey().createChildKey(q);
        TilePagedLOD* lod = new TilePagedLOD(this, subkey, engineUID, live, dead);
        lod->setDatabaseOptions( dbOptions );
#if 0
        GeoPoint centerMap, centerWorld;
        subkey.getExtent().getCentroid( centerMap );
        centerMap.transform( tilenode->getTileModel()->_map->getWorldSRS(), centerWorld );
        lod->setCenter( centerWorld.vec3d() );
        lod->setRadius( tilenode->getBound().radius() * 0.5 );
#else
        lod->setCenter( tilenode->getBound().center() );
        lod->setRadius( tilenode->getBound().radius() );
#endif
        this->addChild( lod );
    }
}


void
TileGroup::setSubtileRange(float range)
{
    _subtileRange = range;
}


void
TileGroup::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getTraversalMode() == nv.TRAVERSE_ACTIVE_CHILDREN )
    {
        float range = 0.0f;
        if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
            range = nv.getDistanceFromEyePoint( getBound().center(), true );
        }

        // if all four subtiles have reported that they are upsampling, 
        // don't use any of them.
        if ( _traverseSubtiles && _numSubtilesUpsampling == 4 )
        {
            _traverseSubtiles = false;
        }

        // if we are out of subtile range, or we're in range but the subtiles are
        // not all loaded yet, or we are skipping subtiles, draw the current tile.
        if ( range > _subtileRange || _numSubtilesLoaded < 4 || !_traverseSubtiles )
        {
            _tilenode->accept( nv );
        }

        // if we're in range, traverse the subtiles.
        if ( _traverseSubtiles && range <= _subtileRange )
        {
            for( unsigned q=0; q<4; ++q )
            {
                getChild(1+q)->accept( nv );
            }
        }
    }
    else
    {
        osg::Group::traverse( nv );
    }
}
