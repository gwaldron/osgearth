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

#include <osgEarthAnnotation/Decoration>

#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/LocalizedNode>
#include <osgEarthAnnotation/OrthoNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/TrackNode>

using namespace osgEarth::Annotation;

//---------------------------------------------------------------------------

void
DecorationInstaller::apply(osg::Node& node)
{
    if ( dynamic_cast<AnnotationNode*>(&node) )
    {
        if ( _tech.valid() )
            static_cast<AnnotationNode*>(&node)->installDecoration( _name, _tech );
        else if ( _callback.valid() )
            _callback->operator()( static_cast<AnnotationNode*>(&node) );
    }
    traverse(node);
}

//---------------------------------------------------------------------------

bool
Decoration::apply(class AnnotationNode& node, bool enable)
{
    return false;
}

bool
Decoration::apply(class LocalizedNode& node, bool enable)
{ 
    return apply(static_cast<AnnotationNode&>(node), enable);
}

bool
Decoration::apply(class OrthoNode& node, bool enable)
{
    return apply(static_cast<AnnotationNode&>(node), enable);
}

//---------------------------------------------------------------------------

InjectionDecoration::InjectionDecoration( osg::Group* group ) :
_injectionGroup( group )
{
    if ( !_injectionGroup.valid() )
        _injectionGroup = new osg::Group();
}

bool
InjectionDecoration::apply(AnnotationNode& node, bool enable)
{
    bool success = apply( node.getChildAttachPoint(), enable );
    return success ? true : Decoration::apply(node, enable);
}

bool
InjectionDecoration::apply(osg::Group* ap, bool enable)
{
    if ( _injectionGroup.valid() && ap )
    {
        if ( enable )
        {
            for( unsigned i=0; i<ap->getNumChildren(); ++i )
            {
                _injectionGroup->addChild( ap->getChild(i) );
            }
            ap->removeChildren(0, ap->getNumChildren() );
            ap->addChild( _injectionGroup.get() );
        }
        else // if ( !enable)
        {
            for( unsigned i=0; i<_injectionGroup->getNumChildren(); ++i )
            {
                ap->addChild( _injectionGroup->getChild(i) );
            }
            ap->removeChild(0, 1);
            _injectionGroup->removeChildren(0, _injectionGroup->getNumChildren());
        }
        return true;
    }
    return false;
}
