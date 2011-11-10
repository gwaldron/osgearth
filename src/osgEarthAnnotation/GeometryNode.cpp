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

#include <osgEarthAnnotation/GeometryNode>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarth/Utils>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;


LocalGeometryNode::LocalGeometryNode(MapNode*     mapNode,
                                     Geometry*    geom,
                                     const Style& style,
                                     bool         draped,
                                     osg::Group*  parent ) :
DrapeableNode( mapNode, draped )
{
    osg::ref_ptr<Feature> feature = new Feature( geom );

    GeometryCompiler compiler;
    FilterContext cx( 0L );
    osg::Node* node = compiler.compile( feature.get(), style, cx );
    if ( node )
    {
        if ( parent )
        {
            parent->addChild( node );
            setNode( parent );
        }
        else
        {
            setNode( node );
        }
    }
}
