/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
_style       ( style ),
_draped      ( draped )
{
    init( 0L );
    this->addChild( getRoot() );
}


LocalGeometryNode::LocalGeometryNode(MapNode*     mapNode,
                                     osg::Node*   node,
                                     const Style& style,
                                     bool         draped ) :
LocalizedNode( mapNode ),
_node        ( node ),
_style       ( style ),
_draped      ( draped )
{
    init( 0L );
    this->addChild( getRoot() );
}


void
LocalGeometryNode::initNode()
{
    // reset
    getChildAttachPoint()->removeChildren( 0, getChildAttachPoint()->getNumChildren() );

    if ( _node.valid() )
    {
        getChildAttachPoint()->addChild( _node.get() );
        getOverlay()->setActive( _draped );

        // this will activate the clamping logic
        applyStyle( _style );

        setLightingIfNotSet( _style.has<ExtrusionSymbol>() );
    }
}


void
LocalGeometryNode::initGeometry(const osgDB::Options* dbOptions)
{
    // reset
    getChildAttachPoint()->removeChildren( 0, getChildAttachPoint()->getNumChildren() );

    if ( _geom.valid() )
    {
        osg::ref_ptr<Feature> feature = new Feature( _geom.get(), 0L );
        feature->style() = _style;

        GeometryCompiler compiler;
        
        Session* session = 0L;
        if ( getMapNode() )
            session = new Session(getMapNode()->getMap(), 0L, 0L, dbOptions);

        FilterContext cx( session );

        osg::Node* node = compiler.compile( feature.get(), cx );
        if ( node )
        {
            getChildAttachPoint()->addChild( node );
            getOverlay()->setActive( _draped );

            // prep for clamping
            applyStyle( _style );
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
    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        _geom = GeometryUtils::geometryFromWKT( geomconf.value() );
        if ( _geom.valid() )
        {
            conf.getObjIfSet( "style", _style );
            _draped = conf.value<bool>("draped",false);

            init( dbOptions );

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
        if ( !_style.empty() )
            conf.addObj( "style", _style );
        conf.addObj( "position", getPosition() );
    }
    else
    {
        OE_WARN << LC << "Cannot serialize GeometryNode because it contains no geometry" << std::endl;
    }

    return conf;
}
