/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/Annotation>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

Annotation::Annotation( long fid ) :
Feature( fid )
{
    //nop
}

Annotation::Annotation( const Annotation& rhs, const osg::CopyOp& op ) :
Feature( rhs, op )
{
    //nop
}

/***************************************************************************/

TextAnnotation::TextAnnotation( long fid ) :
Annotation( fid )
{
    //nop
}

TextAnnotation::TextAnnotation( const TextAnnotation& rhs, const osg::CopyOp& op ) :
Annotation( rhs, op ),
_text( rhs._text )
{
    //nop
}

