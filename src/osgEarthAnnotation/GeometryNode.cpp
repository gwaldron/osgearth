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
    osg::ref_ptr<Feature> feature = new Feature( geom, 0L ); //todo:consider the SRS

    GeometryCompiler compiler;
    FilterContext cx( 0L );
    osg::Node* node = compiler.compile( feature.get(), style, cx );
    if ( node )
    {
        getTransform()->addChild( node );
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

void
GeometryNode::reclamp( const TileKey& key, osg::Node* tile, const Terrain* terrain )
{
    // since a GeometyNode is always local-tangent plane, we only need to reclamp
    // the reference position (and not all the verts)
    osg::Vec3d mapPos = getPosition();
    if ( key.getExtent().contains(mapPos.x(), mapPos.y()) )
    {
        double height;
        if ( terrain->getHeight(mapPos, height, tile) )
        {
            if ( _altitude.valid() )
            {
                height *= _altitude->verticalScale()->eval();
                height += _altitude->verticalOffset()->eval();
            }
            setPosition( osg::Vec3d(mapPos.x(), mapPos.y(), height) );
        }
    }
}
