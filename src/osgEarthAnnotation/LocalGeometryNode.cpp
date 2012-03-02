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

#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthFeatures/MeshClamper>
#include <osgEarth/DrapeableNode>
#include <osgEarth/Utils>

#define LC "[GeometryNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;


LocalGeometryNode::LocalGeometryNode(MapNode*     mapNode,
                                     Geometry*    geom,
                                     const Style& style,
                                     bool         draped ) :
LocalizedNode( mapNode ),
_geom        ( geom ),
_draped      ( draped )
{
    _style = style;
    init();
}

LocalGeometryNode::LocalGeometryNode(MapNode*     mapNode,
                                     osg::Node*   content,
                                     const Style& style,
                                     bool         draped ) :
LocalizedNode( mapNode ),
_draped      ( draped )
{
    _style = style;

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


void
LocalGeometryNode::init()
{
    if ( _geom.valid() )
    {
        osg::ref_ptr<Feature> feature = new Feature( _geom.get(), 0L );
        feature->style() = *_style;

        GeometryCompiler compiler;
        FilterContext cx( _mapNode.valid() ? new Session(_mapNode->getMap()) : 0L );
        osg::Node* node = compiler.compile( feature.get(), cx );
        if ( node )
        {
            getTransform()->addChild( node );
            if ( _draped && _mapNode.valid() )
            {
                DrapeableNode* dn = new DrapeableNode(_mapNode.get());
                dn->addChild( getTransform() );
                this->addChild( dn );
            }
            else
            {
                this->addChild( getTransform() );
            }

            // prep for clamping
            applyStyle( *_style, _draped );
        }
    }
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( local_geometry, osgEarth::Annotation::LocalGeometryNode );


LocalGeometryNode::LocalGeometryNode(MapNode*      mapNode,
                                     const Config& conf) :
LocalizedNode( mapNode )
{
    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        _geom = GeometryUtils::geometryFromWKT( geomconf.value() );
        if ( _geom.valid() )
        {
            conf.getObjIfSet( "style", _style );
            _draped = conf.value<bool>("draped",false);
            init();
            if ( conf.hasChild("position") )
                setPosition( GeoPoint(conf.child("position")) );
        }
    }
}

Config
LocalGeometryNode::getConfig() const
{
    Config conf("local_geometry");

    if ( _geom.valid() )
    {
        conf.add( Config("geometry", GeometryUtils::geometryToWKT(_geom.get())) );
        conf.addObjIfSet( "style", _style );
        conf.addObj( "position", getPosition() );
    }
    else
    {
        OE_WARN << LC << "Cannot serialize GeometryNode because it contains no geometry" << std::endl;
    }

    return conf;
}
