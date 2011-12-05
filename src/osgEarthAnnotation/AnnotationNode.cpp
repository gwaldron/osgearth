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
#include <osgEarth/Registry>

using namespace osgEarth;
using namespace osgEarth::Annotation;

AnnotationNode::AnnotationNode() :
_dynamic( false )
{
    //nop
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
AnnotationNode::setHighlight( bool value )
{
    _highlight = value;

    // update the selection uniform.
    if ( Registry::instance()->getCapabilities().supportsGLSL() )
    {
        osg::StateSet* stateSet = getOrCreateStateSet();
        osg::Uniform* u = stateSet->getUniform( AnnotationUtils::UNIFORM_HIGHLIGHT() );
        if ( !u )
        {
            u = new osg::Uniform( osg::Uniform::BOOL, AnnotationUtils::UNIFORM_HIGHLIGHT() );
            stateSet->addUniform( u );
        }
        u->set( value );
    }
}
