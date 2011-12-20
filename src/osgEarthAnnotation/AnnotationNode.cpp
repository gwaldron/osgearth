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
#include <osgEarth/Registry>

using namespace osgEarth;
using namespace osgEarth::Annotation;

AnnotationNode::AnnotationNode() :
_dynamic ( false ),
_activeDs( 0L )
{
    //nop
}

AnnotationNode::AnnotationNode(const AnnotationNode& rhs, const osg::CopyOp& op) :
osg::Switch(rhs, op)
{
    _dynamic   = rhs._dynamic;
    _annoData  = rhs._annoData.get();
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
AnnotationNode::installAltDrawState( const std::string& name, DrawState* ds )
{
    if ( _activeDs )
    {
        clearAltDrawState();
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
AnnotationNode::uninstallAltDrawState( const std::string& name )
{
    clearAltDrawState();
    _dsMap.erase( name );
}

void
AnnotationNode::setAltDrawState( const std::string& name )
{
    // already active?
    if ( _activeDs && _activeDsName == name )
        return;

    // is a different one active? if so kill it
    if ( _activeDs )
        clearAltDrawState();

    // try to find and enable the new one
    DrawStateMap::iterator i = _dsMap.find(name);
    if ( i != _dsMap.end() )
    {
        DrawState* ds = i->second.get();
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
AnnotationNode::clearAltDrawState()
{
    if ( _activeDs )
    {
        this->accept(_activeDs, false);
        _activeDs = 0L;
    }
}

bool
AnnotationNode::hasAltDrawState( const std::string& name ) const
{
    return _dsMap.find(name) != _dsMap.end();
}

osg::Group*
AnnotationNode::getAttachPoint()
{
    osg::Transform* t = osgEarth::findTopMostNodeOfType<osg::Transform>(this);
    return t ? (osg::Group*)t : (osg::Group*)this;
}
