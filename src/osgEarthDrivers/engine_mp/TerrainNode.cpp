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
#include "TerrainNode"
#include "QuickReleaseGLObjects"

#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/NodeUtils>
#include <osgEarth/ThreadingUtils>

#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osgGA/EventVisitor>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TerrainNode] "

//----------------------------------------------------------------------------

TerrainNode::TerrainNode(TileNodeRegistry* removedTiles ) :
_tilesToQuickRelease            ( removedTiles ),
_quickReleaseCallbackInstalled  ( false )
{
    // tick the update count to install the quick release callback:
    if ( _tilesToQuickRelease.valid() )
    {
        ADJUST_UPDATE_TRAV_COUNT( this, 1 );
    }
}


void
TerrainNode::traverse( osg::NodeVisitor &nv )
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        // if the terrain engine requested "quick release", install the quick release
        // draw callback now.
        if ( !_quickReleaseCallbackInstalled && _tilesToQuickRelease.valid() )
        {
            osg::Camera* cam = findFirstParentOfType<osg::Camera>( this );
            if ( cam )
            {
                // get the installed PDC so we can nest them:
                osg::Camera::DrawCallback* cbToNest = cam->getPostDrawCallback();

                // if it's another QR callback, we'll just replace it.
                QuickReleaseGLObjects* previousQR = dynamic_cast<QuickReleaseGLObjects*>(cbToNest);
                if ( previousQR )
                    cbToNest = previousQR->_next.get();

                cam->setPostDrawCallback( new QuickReleaseGLObjects(
                    _tilesToQuickRelease.get(),
                    cbToNest ) );

                _quickReleaseCallbackInstalled = true;
                OE_INFO << LC << "Quick release enabled" << std::endl;

                // knock down the trav count set in the constructor.
                ADJUST_UPDATE_TRAV_COUNT( this, -1 );
            }
        }
    }

    osg::Group::traverse( nv );
}
