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
                                     const Style& style) :
LocalizedNode( mapNode ),
_geom        ( geom ),
_style       ( style )
{
    _xform = new osg::MatrixTransform();
    init( 0L );
}


LocalGeometryNode::LocalGeometryNode(MapNode*     mapNode,
                                     osg::Node*   node,
                                     const Style& style) :
LocalizedNode( mapNode ),
_node        ( node ),
_style       ( style )
{
    _xform = new osg::MatrixTransform();
    init( 0L );
}


void
LocalGeometryNode::initNode()
{
    // reset
    osgEarth::clearChildren( this );
    osgEarth::clearChildren( _xform.get() );
    this->addChild( _xform.get() );

    if ( _node.valid() )
    {
        _xform->addChild( _node );
        // activate clamping if necessary
        replaceChild( _xform.get(), applyAltitudePolicy(_xform.get(), _style) );

        applyGeneralSymbology( _style );
        setLightingIfNotSet( _style.has<ExtrusionSymbol>() );
    }
}


void
LocalGeometryNode::initGeometry(const osgDB::Options* dbOptions)
{
    // reset
    osgEarth::clearChildren( this );
    osgEarth::clearChildren( _xform.get() );
    this->addChild( _xform.get() );

    if ( _geom.valid() )
    {
        Session* session = 0L;
        if ( getMapNode() )
            session = new Session(getMapNode()->getMap(), 0L, 0L, dbOptions);
        FilterContext cx( session );

        GeometryCompiler gc;
        osg::Node* node = gc.compile( _geom.get(), _style, cx );
        if ( node )
        {
            _xform->addChild( node );
            // activate clamping if necessary
            replaceChild( _xform.get(), applyAltitudePolicy(_xform.get(), _style) );

            applyGeneralSymbology( _style );
        }
    }
}


void 
LocalGeometryNode::init(const osgDB::Options* options )
{
    this->clearDecoration();
    
    if ( _node.valid() )
    {
        initNode();
    }
    else
    {
        initGeometry( options );
    }
}


void
LocalGeometryNode::setStyle( const Style& style )
{
    _style = style;
    init( 0L );
}


void
LocalGeometryNode::setNode( osg::Node* node )
{
    _node = node;
    _geom = 0L;
    initNode();
}


void
LocalGeometryNode::setGeometry( Geometry* geom )
{
    _geom = geom;
    _node = 0L;
    initGeometry(0L);
}

//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( local_geometry, osgEarth::Annotation::LocalGeometryNode );


LocalGeometryNode::LocalGeometryNode(MapNode*              mapNode,
                                     const Config&         conf,
                                     const osgDB::Options* dbOptions) :
LocalizedNode( mapNode, conf )
{
    _xform = new osg::MatrixTransform();

    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        _geom = GeometryUtils::geometryFromWKT( geomconf.value() );
        if ( _geom.valid() )
        {
            conf.getObjIfSet( "style", _style );

            init( dbOptions );
        }
    }
}

Config
LocalGeometryNode::getConfig() const
{
    Config conf = LocalizedNode::getConfig();
    conf.key() = "local_geometry";

    if ( _geom.valid() )
    {
        conf.add( Config("geometry", GeometryUtils::geometryToWKT(_geom.get())) );
        if ( !_style.empty() )
            conf.addObj( "style", _style );
    }
    else
    {
        OE_WARN << LC << "Cannot serialize GeometryNode because it contains no geometry" << std::endl;
    }

    return conf;
}
