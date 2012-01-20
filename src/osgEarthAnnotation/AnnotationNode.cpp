/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarth/FindNode>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::Annotation;

//-------------------------------------------------------------------

namespace osgEarth { namespace Annotation
{
    struct AutoClampCallback : public TerrainCallback
    {
        void onTileAdded( const TileKey& key, osg::Node* tile, TerrainCallbackContext& context )
        {
            AnnotationNode* anno = dynamic_cast<AnnotationNode*>(context.getClientData());
            anno->reclamp( key, tile );
        }
    };
}  }

//-------------------------------------------------------------------

AnnotationNode::AnnotationNode(MapNode* mapNode) :
_mapNode    ( mapNode ),
_dynamic    ( false ),
_autoclamp  ( false ),
_activeDs   ( 0L )
{
    //nop
}

AnnotationNode::~AnnotationNode()
{
    osg::ref_ptr<MapNode> mapNodeSafe = _mapNode.get();
    if ( mapNodeSafe.get() )
    {
        mapNodeSafe->getTerrain()->removeTerrainCallbacksWithClientData(this);
    }
}

void
AnnotationNode::setAnnotationData( AnnotationData* data )
{
    _annoData = data;
}

void
AnnotationNode::setDynamic( bool value )
{
    _dynamic = value;
}

void
AnnotationNode::setAutoClamp( bool value )
{
    osg::ref_ptr<MapNode> mapNode_safe = _mapNode.get();
    if ( mapNode_safe.valid() )
    {
        if ( !_autoclamp && value )
        {
            setDynamic( true );
            mapNode_safe->getTerrain()->addTerrainCallback(new AutoClampCallback(), this);
        }
        else if ( _autoclamp && !value )
        {
            mapNode_safe->getTerrain()->removeTerrainCallbacksWithClientData(this);
        }

        _autoclamp = value;
    }
}

void
AnnotationNode::installDecoration( const std::string& name, Decoration* ds )
{
    if ( _activeDs )
    {
        clearDecoration();
    }

    if ( ds == 0L )
    {
        _dsMap.erase( name );
    }
    else
    {
        _dsMap[name] = ds->copyOrClone();
    }
}

void
AnnotationNode::uninstallDecoration( const std::string& name )
{
    clearDecoration();
    _dsMap.erase( name );
}

void
AnnotationNode::setDecoration( const std::string& name )
{
    // already active?
    if ( _activeDs && _activeDsName == name )
        return;

    // is a different one active? if so kill it
    if ( _activeDs )
        clearDecoration();

    // try to find and enable the new one
    DecorationMap::iterator i = _dsMap.find(name);
    if ( i != _dsMap.end() )
    {
        Decoration* ds = i->second.get();
        if ( ds )
        {
            if ( this->accept(ds, true) ) 
            {
                _activeDs = ds;
                _activeDsName = name;
            }
        }
    }
}

void
AnnotationNode::clearDecoration()
{
    if ( _activeDs )
    {
        this->accept(_activeDs, false);
        _activeDs = 0L;
    }
}

bool
AnnotationNode::hasDecoration( const std::string& name ) const
{
    return _dsMap.find(name) != _dsMap.end();
}

osg::Group*
AnnotationNode::getAttachPoint()
{
    osg::Transform* t = osgEarth::findTopMostNodeOfType<osg::Transform>(this);
    return t ? (osg::Group*)t : (osg::Group*)this;
}
