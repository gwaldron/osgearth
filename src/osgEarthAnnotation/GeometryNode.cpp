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
#include <osgEarthFeatures/MeshClamper>
#include <osgEarth/DrapeableNode>
#include <osgEarth/Utils>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;


GeometryNode::GeometryNode(MapNode*     mapNode,
                           Geometry*    geom,
                           const Style& style,
                           bool         draped ) :
LocalizedNode( mapNode )
{
    osg::ref_ptr<Feature> feature = new Feature( geom, 0L );

    GeometryCompiler compiler;
    FilterContext cx( mapNode ? new Session(mapNode->getMap()) : 0L );
    osg::Node* node = compiler.compile( feature.get(), style, cx );
    if ( node )
    {
        getTransform()->addChild( node );
        if ( draped && mapNode )
        {
            DrapeableNode* dn = new DrapeableNode(mapNode);
            dn->addChild( getTransform() );
            this->addChild( dn );
        }
        else
        {
            this->addChild( getTransform() );
        }

        // prep for clamping
        applyStyle( style, draped );
    }
}

GeometryNode::GeometryNode(MapNode*     mapNode,
                           osg::Node*   content,
                           const Style& style,
                           bool         draped ) :
LocalizedNode( mapNode )
{
    if ( content )
    {
        getTransform()->addChild( content );
        if ( draped )
        {
            DrapeableNode* dn = new DrapeableNode(mapNode);
            dn->addChild( getTransform() );
            this->addChild( dn );
        }
        else
        {
            this->addChild( getTransform() );
        }

        // this will activate the clamping logic
        applyStyle( style, draped );
    }
}
